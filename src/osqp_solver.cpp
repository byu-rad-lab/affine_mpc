#include "affine_mpc/osqp_solver.hpp"

#include <Eigen/Core>
#include <cassert>
#include <cstddef>
#include <memory>
#include <osqp_api_constants.h>
#include <osqp_api_functions.h>
#include <osqp_api_types.h>
#include <ostream>
#include <type_traits>
#include <vector>

#include "affine_mpc/detail/qp.hpp"
#include "affine_mpc/osqp_settings.hpp"
#include "affine_mpc/solve_status.hpp"
#include "osqp_solver_impl.hpp"

namespace affine_mpc {

// The MPC classes store the QP as double precision Eigen objects, which would
// be silently converted at every update by a single precision OSQP build
// (OSQP_USE_FLOAT), and the solution is viewed as doubles. Fail at compile
// time instead. OSQP's index type may differ from detail::QPIndex: the
// indices are converted once at initialization.
static_assert(std::is_same_v<OSQPFloat, double>,
              "affine_mpc requires OSQP built with double precision "
              "(OSQPFloat must be double; do not define OSQP_USE_FLOAT).");

namespace {

SolveStatus osqpStatusToSolveStatus(const OSQPInt osqp_status)
{
  switch (osqp_status) {
  case OSQP_SOLVED:
    return SolveStatus::Success;
  case OSQP_SOLVED_INACCURATE:
    return SolveStatus::SolvedInaccurate;
  case OSQP_PRIMAL_INFEASIBLE:
  case OSQP_PRIMAL_INFEASIBLE_INACCURATE:
    return SolveStatus::PrimalInfeasible;
  case OSQP_DUAL_INFEASIBLE:
  case OSQP_DUAL_INFEASIBLE_INACCURATE:
    return SolveStatus::DualInfeasible;
  case OSQP_MAX_ITER_REACHED:
    return SolveStatus::MaxIterReached;
  case OSQP_TIME_LIMIT_REACHED:
    return SolveStatus::TimeLimitReached;
  case OSQP_NON_CVX:
  case OSQP_SIGINT:
  case OSQP_UNSOLVED:
  default:
    return SolveStatus::OtherFailure;
  }
}

} // namespace

// The enum values are OSQP's, so the conversion is a cast
static_assert(static_cast<int>(OSQPSettings::LinsysSolverType::UnknownSolver)
              == OSQP_UNKNOWN_SOLVER);
static_assert(static_cast<int>(OSQPSettings::LinsysSolverType::DirectSolver)
              == OSQP_DIRECT_SOLVER);
static_assert(static_cast<int>(OSQPSettings::LinsysSolverType::IndirectSolver)
              == OSQP_INDIRECT_SOLVER);
static_assert(
    static_cast<int>(OSQPSettings::PreconditionerType::NoPreconditioner)
    == OSQP_NO_PRECONDITIONER);
static_assert(
    static_cast<int>(OSQPSettings::PreconditionerType::DiagonalPreconditioner)
    == OSQP_DIAGONAL_PRECONDITIONER);
static_assert(static_cast<int>(OSQPSettings::AdaptiveRhoType::Disabled)
              == OSQP_ADAPTIVE_RHO_UPDATE_DISABLED);
static_assert(static_cast<int>(OSQPSettings::AdaptiveRhoType::Iterations)
              == OSQP_ADAPTIVE_RHO_UPDATE_ITERATIONS);
static_assert(static_cast<int>(OSQPSettings::AdaptiveRhoType::Time)
              == OSQP_ADAPTIVE_RHO_UPDATE_TIME);
static_assert(static_cast<int>(OSQPSettings::AdaptiveRhoType::KKTError)
              == OSQP_ADAPTIVE_RHO_UPDATE_KKT_ERROR);

::OSQPSettings toOSQP(const OSQPSettings& settings) noexcept
{
  ::OSQPSettings s;
  s.device = settings.device;
  s.linsys_solver =
      static_cast<osqp_linsys_solver_type>(settings.linsys_solver);
  s.allocate_solution = settings.allocate_solution;
  s.verbose = settings.verbose;
  s.profiler_level = settings.profiler_level;
  s.warm_starting = settings.warm_starting;
  s.scaling = settings.scaling;
  s.polishing = settings.polishing;
  s.rho = settings.rho;
  s.rho_is_vec = settings.rho_is_vec;
  s.sigma = settings.sigma;
  s.alpha = settings.alpha;
  s.cg_max_iter = settings.cg_max_iter;
  s.cg_tol_reduction = settings.cg_tol_reduction;
  s.cg_tol_fraction = settings.cg_tol_fraction;
  s.cg_precond = static_cast<osqp_precond_type>(settings.cg_precond);
  s.adaptive_rho = static_cast<OSQPInt>(settings.adaptive_rho);
  s.adaptive_rho_interval = settings.adaptive_rho_interval;
  s.adaptive_rho_fraction = settings.adaptive_rho_fraction;
  s.adaptive_rho_tolerance = settings.adaptive_rho_tolerance;
  s.max_iter = settings.max_iter;
  s.eps_abs = settings.eps_abs;
  s.eps_rel = settings.eps_rel;
  s.eps_prim_inf = settings.eps_prim_inf;
  s.eps_dual_inf = settings.eps_dual_inf;
  s.scaled_termination = settings.scaled_termination;
  s.check_termination = settings.check_termination;
  s.check_dualgap = settings.check_dualgap;
  s.time_limit = settings.time_limit;
  s.delta = settings.delta;
  s.polish_refine_iter = settings.polish_refine_iter;
  return s;
}

OSQPSolver::OSQPSolver(int num_variables, int num_constraints) :
    OSQPSolver(num_variables, num_constraints, getRecommendedSettings())
{}

OSQPSolver::OSQPSolver(int num_variables,
                       int num_constraints,
                       const OSQPSettings& settings) :
    impl_{std::make_unique<Impl>(num_variables, num_constraints)},
    settings_{settings},
    // OSQP's cost scaling is 1 / max(mean column norm of P, ||q||_inf) after
    // equilibration, but it treats ||q||_inf below OSQP_MIN_SCALING as 1, so
    // q = 0 caps the scaling at 1 and a weak P (small weights) stays poorly
    // scaled. Just above that threshold, q is too small to set the scaling,
    // so it comes from P alone. If equilibration shrinks q below the
    // threshold, OSQP treats it as q = 0 again.
    setup_q_{Eigen::VectorXd::Constant(num_variables, 2 * OSQP_MIN_SCALING)}
{}

OSQPSolver::~OSQPSolver() = default;

OSQPSolver::Impl::Impl(int num_variables, int num_constraints) :
    n_{num_variables},
    m_{num_constraints},
    P_p_{num_variables + 1},
    A_p_{num_variables + 1},
    solver_{nullptr, osqp_cleanup},
    P_{nullptr, OSQPCscMatrix_free},
    A_{nullptr, OSQPCscMatrix_free},
    l_buf_{num_constraints},
    u_buf_{num_constraints}
{
  assert(num_variables > 0 && num_constraints > 0);
}

OSQPSettings OSQPSolver::getDefaultSettings() noexcept { return {}; }

OSQPSettings
OSQPSolver::getRecommendedSettings(bool polish_near_boundaries) noexcept
{
  OSQPSettings settings;
  settings.warm_starting = true; // OSQP's default too

  // Over-relaxation parameter.
  //   - alpha = 1.0 -> standard ADMM
  //   - alpha > 1.0 -> over-relaxation (can improve convergence speed)
  //   - alpha < 1.0 -> under-relaxation (rarely useful)
  // If iters is high, consider increasing to 1.6-1.8.
  // If oscillations occur, consider decreasing to 1.4-1.5.
  // Can also try 1.0 for standard ADMM if convergence is an issue.
  settings.alpha = 1.6; // default is 1.6

  // Disable printing to console at each solve
  settings.verbose = false;

  // These control the accuracy of the solution/constraints.
  // If solution takes a while to converge, consider loosening these.
  settings.eps_rel = 1e-6; // default is 1e-3
  settings.eps_abs = 1e-6; // default is 1e-3

  // An MPC cost is a sum of weighted squares, so it is bounded below and the
  // QP is never dual infeasible. OSQP's default (1e-4) accepts a nearly flat
  // direction of an ill-conditioned P as an unbounded ray, so a solvable QP
  // can stop as DualInfeasible. eps_prim_inf keeps OSQP's default: tightening
  // it delays the detection of QPs that are genuinely primal infeasible.
  settings.eps_dual_inf = 1e-6; // default is 1e-4

  // Adds a polishing step (extra solve after convergence) that can improve
  // solution accuracy when the solution is near the boundaries of the
  // constraints. This can be helpful for MPC when the solution saturates inputs
  // or states, but it does add some extra solve time.
  if (polish_near_boundaries)
    settings.polishing = true;
  return settings;
}

const Eigen::Map<const OSQPSolver::Impl::VectorXF>
OSQPSolver::Impl::getSolutionMap() const noexcept
{
  if (initialized_)
    return Eigen::Map<const VectorXF>{solver_->solution->x, n_, 1};
  else
    return Eigen::Map<const VectorXF>{nullptr, 0, 1};
}

SolveInfo OSQPSolver::Impl::getSolveInfo() const noexcept
{
  if (!initialized_)
    return SolveInfo{};
  const OSQPInfo& info{*solver_->info};
  // mpc_solve_time and prepare_time are only known to MPC
  SolveInfo result{osqpStatusToSolveStatus(static_cast<int>(info.status_val)),
                   static_cast<int>(info.iter),
                   info.obj_val,
                   info.solve_time + info.polish_time,
                   0.0,
                   0.0};
  if (info.status_val != OSQP_UNSOLVED) {
    result.primal_residual = info.prim_res;
    result.dual_residual = info.dual_res;
  }
  return result;
}

bool OSQPSolver::Impl::getDualSolution(Eigen::Ref<Eigen::VectorXd> y) const
{
  assert(y.size() == m_);
  if (!initialized_)
    return false;
  y = Eigen::Map<const VectorXF>{solver_->solution->y, m_, 1};
  return true;
}

SolveStatus OSQPSolver::Impl::solve(Eigen::Ref<VectorXF> solution)
{
  assert(solution.size() == n_);
  const SolveStatus status{solve()};
  if (status == SolveStatus::NotInitialized)
    return status;
  solution = Eigen::Map<const VectorXF>{solver_->solution->x, n_, 1};
  return status;
}

SolveStatus OSQPSolver::Impl::solve()
{
  if (!initialized_)
    return SolveStatus::NotInitialized;
  osqp_solve(solver_.get());
  return osqpStatusToSolveStatus(solver_->info->status_val);
}

void OSQPSolver::Impl::coldStart() noexcept
{
  if (initialized_)
    osqp_cold_start(solver_.get());
}

bool OSQPSolver::Impl::initialize(const Eigen::Ref<const MatrixXF>& P,
                                  const Eigen::Ref<const MatrixXF>& A,
                                  const Eigen::Ref<const VectorXF>& q,
                                  const Eigen::Ref<const VectorXF>& l,
                                  const Eigen::Ref<const VectorXF>& u,
                                  const OSQPSettings& settings)
{
  assert(q.size() == n_ && l.size() == m_ && u.size() == m_);
  initializeCostMatrix(P);
  initializeConstraintMatrix(A);
  sparse_input_ = false;
  return setup(q, l, u, settings);
}

bool OSQPSolver::Impl::initialize(const SparseMatrixXF& P,
                                  const SparseMatrixXF& A,
                                  const Eigen::Ref<const VectorXF>& q,
                                  const Eigen::Ref<const VectorXF>& l,
                                  const Eigen::Ref<const VectorXF>& u,
                                  const OSQPSettings& settings)
{
  assert(q.size() == n_ && l.size() == m_ && u.size() == m_);
  assert(P.rows() == n_ && P.cols() == n_);
  assert(A.rows() == m_ && A.cols() == n_);
  if (!P.isCompressed() || !A.isCompressed() || !initializeCostMatrix(P))
    return false;
  initializeConstraintMatrix(A);
  sparse_input_ = true;
  return setup(q, l, u, settings);
}

bool OSQPSolver::Impl::setup(const Eigen::Ref<const VectorXF>& q,
                             const Eigen::Ref<const VectorXF>& l,
                             const Eigen::Ref<const VectorXF>& u,
                             const OSQPSettings& settings)
{
  // the solution is read from the buffers OSQP allocates
  if (!settings.allocate_solution)
    return false;
  clampBounds(l, u);

  const ::OSQPSettings osqp_settings{toOSQP(settings)};
  ::OSQPSolver* raw_solver = nullptr;
  OSQPInt exitflag{osqp_setup(&raw_solver, P_.get(), q.data(), A_.get(),
                              l_buf_.data(), u_buf_.data(), m_, n_,
                              &osqp_settings)};
  solver_.reset(raw_solver);
  if (exitflag == 0)
    initialized_ = true;
  return initialized_;
}

bool OSQPSolver::Impl::updateCostMatrix(const Eigen::Ref<const MatrixXF>& P)
{
  if (!initialized_ || sparse_input_)
    return false;
  assert(P.rows() == n_ && P.cols() == n_);

  fillCostValues(P);
  OSQPInt exit_status{osqp_update_data_mat(
      solver_.get(), P_x_.data(), OSQP_NULL, P_nnz_, OSQP_NULL, OSQP_NULL, 0)};
  return exit_status == 0;
}

bool OSQPSolver::Impl::updateConstraintMatrix(
    const Eigen::Ref<const MatrixXF>& A)
{
  if (!initialized_ || sparse_input_)
    return false;
  assert(A.rows() == m_ && A.cols() == n_);

  fillConstraintValues(A);
  OSQPInt exit_status{osqp_update_data_mat(solver_.get(), OSQP_NULL, OSQP_NULL,
                                           0, A_x_.data(), OSQP_NULL, A_nnz_)};
  return exit_status == 0;
}

bool OSQPSolver::Impl::updateMatrices(const Eigen::Ref<const MatrixXF>& P,
                                      const Eigen::Ref<const MatrixXF>& A)
{
  if (!initialized_ || sparse_input_)
    return false;
  assert(P.rows() == n_ && P.cols() == n_);
  assert(A.rows() == m_ && A.cols() == n_);

  fillCostValues(P);
  fillConstraintValues(A);
  OSQPInt exit_status{osqp_update_data_mat(solver_.get(), P_x_.data(),
                                           OSQP_NULL, P_nnz_, A_x_.data(),
                                           OSQP_NULL, A_nnz_)};
  return exit_status == 0;
}

bool OSQPSolver::Impl::updateCostMatrix(const SparseMatrixXF& P)
{
  if (!initialized_ || !sparse_input_)
    return false;
  OSQPInt exit_status{osqp_update_data_mat(solver_.get(), costValues(P),
                                           OSQP_NULL, P_nnz_, OSQP_NULL,
                                           OSQP_NULL, 0)};
  return exit_status == 0;
}

bool OSQPSolver::Impl::updateConstraintMatrix(const SparseMatrixXF& A)
{
  if (!initialized_ || !sparse_input_)
    return false;
  OSQPInt exit_status{osqp_update_data_mat(solver_.get(), OSQP_NULL, OSQP_NULL,
                                           0, constraintValues(A), OSQP_NULL,
                                           A_nnz_)};
  return exit_status == 0;
}

bool OSQPSolver::Impl::updateMatrices(const SparseMatrixXF& P,
                                      const SparseMatrixXF& A)
{
  if (!initialized_ || !sparse_input_)
    return false;
  OSQPInt exit_status{
      osqp_update_data_mat(solver_.get(), costValues(P), OSQP_NULL, P_nnz_,
                           constraintValues(A), OSQP_NULL, A_nnz_)};
  return exit_status == 0;
}

namespace {

// OSQP's values gathered from the positions src in values
void gather(const std::vector<OSQPInt>& src,
            const OSQPFloat* values,
            OSQPFloat* dst)
{
  const std::size_t size{src.size()};
  for (std::size_t k{0}; k < size; ++k)
    dst[k] = values[src[k]];
}

// The positions col * stride + row of the entries of a CSC pattern
void densePositions(const OSQPSolver::Impl::VectorXI& p,
                    const OSQPSolver::Impl::VectorXI& i,
                    const Eigen::Index stride,
                    std::vector<OSQPInt>& src)
{
  src.resize(static_cast<std::size_t>(p(p.size() - 1)));
  for (Eigen::Index col{0}; col + 1 < p.size(); ++col)
    for (OSQPInt k{p(col)}; k < p(col + 1); ++k)
      src[static_cast<std::size_t>(k)] =
          static_cast<OSQPInt>(col * stride) + i(k);
}

// Whether every nonzero stored value of M (with row <= col if upper) is at an
// index in src (increasing)
bool storedNonzerosIn(const OSQPSolver::Impl::SparseMatrixXF& M,
                      const std::vector<OSQPInt>& src,
                      const bool upper)
{
  using detail::QPIndex;
  std::size_t next{0};
  for (QPIndex col{0}; col < M.outerSize(); ++col) {
    for (QPIndex k{M.outerIndexPtr()[col]}; k < M.outerIndexPtr()[col + 1];
         ++k) {
      if (next < src.size() && src[next] == k)
        ++next;
      else if (M.valuePtr()[k] != 0.0 && !(upper && M.innerIndexPtr()[k] > col))
        return false;
    }
  }
  return true;
}

} // namespace

const OSQPFloat* OSQPSolver::Impl::costValues(const SparseMatrixXF& P)
{
  assert(P.rows() == n_ && P.cols() == n_ && P.isCompressed());
  if (P_direct_)
    return P.valuePtr();
  gather(P_src_, P.valuePtr(), P_x_.data());
  return P_x_.data();
}

const OSQPFloat* OSQPSolver::Impl::constraintValues(const SparseMatrixXF& A)
{
  assert(A.rows() == m_ && A.cols() == n_ && A.isCompressed());
  if (A_direct_)
    return A.valuePtr();
  gather(A_src_, A.valuePtr(), A_x_.data());
  return A_x_.data();
}

void OSQPSolver::Impl::fillCostValues(const Eigen::Ref<const MatrixXF>& P)
{
  if (P.outerStride() == P_stride_) {
    gather(P_src_, P.data(), P_x_.data());
    return;
  }
  int idx{0}, idx_diff, row;
  for (int col{0}; col < n_; ++col) {
    idx_diff = P_p_(col + 1) - P_p_(col);
    while (idx_diff != 0) {
      row = P_i_(idx);
      P_x_(idx++) = P(row, col);
      --idx_diff;
    }
  }
}

void OSQPSolver::Impl::fillConstraintValues(const Eigen::Ref<const MatrixXF>& A)
{
  if (A.outerStride() == A_stride_) {
    gather(A_src_, A.data(), A_x_.data());
    return;
  }
  int idx{0}, idx_diff, row;
  for (int col{0}; col < n_; ++col) {
    idx_diff = A_p_(col + 1) - A_p_(col);
    while (idx_diff != 0) {
      row = A_i_(idx);
      A_x_(idx++) = A(row, col);
      --idx_diff;
    }
  }
}

bool OSQPSolver::Impl::updateCostVector(const Eigen::Ref<const VectorXF>& q)
{
  assert(q.size() == n_);
  if (!initialized_)
    return false;

  OSQPInt exit_status{
      osqp_update_data_vec(solver_.get(), q.data(), OSQP_NULL, OSQP_NULL)};
  return exit_status == 0;
}

bool OSQPSolver::Impl::updateBounds(const Eigen::Ref<const VectorXF>& l,
                                    const Eigen::Ref<const VectorXF>& u)
{
  assert(l.size() == m_ && u.size() == m_);
  if (!initialized_)
    return false;

  clampBounds(l, u);
  OSQPInt exit_status{osqp_update_data_vec(solver_.get(), OSQP_NULL,
                                           l_buf_.data(), u_buf_.data())};
  return exit_status == 0;
}

void OSQPSolver::Impl::clampBounds(const Eigen::Ref<const VectorXF>& l,
                                   const Eigen::Ref<const VectorXF>& u)
{
  l_buf_ = l.cwiseMax(-OSQP_INFTY);
  u_buf_ = u.cwiseMin(OSQP_INFTY);
}

bool OSQPSolver::Impl::isWithinSparsityPattern(
    const Eigen::Ref<const MatrixXF>& P,
    const Eigen::Ref<const MatrixXF>& A) const
{
  if (!initialized_ || sparse_input_)
    return false;
  assert(P.rows() == n_ && P.cols() == n_);
  assert(A.rows() == m_ && A.cols() == n_);

  // walk each column alongside its stored row indices (sorted ascending)
  for (OSQPInt col{0}; col < n_; ++col) {
    OSQPInt idx{P_p_(col)};
    for (OSQPInt row{0}; row <= col; ++row) {
      if (idx < P_p_(col + 1) && P_i_(idx) == row)
        ++idx;
      else if (P(row, col) != 0.0)
        return false;
    }
    idx = A_p_(col);
    for (OSQPInt row{0}; row < m_; ++row) {
      if (idx < A_p_(col + 1) && A_i_(idx) == row)
        ++idx;
      else if (A(row, col) != 0.0)
        return false;
    }
  }
  return true;
}

bool OSQPSolver::Impl::isWithinSparsityPattern(const SparseMatrixXF& P,
                                               const SparseMatrixXF& A) const
{
  if (!initialized_ || !sparse_input_)
    return false;
  assert(P.rows() == n_ && P.cols() == n_ && P.isCompressed());
  assert(A.rows() == m_ && A.cols() == n_ && A.isCompressed());
  return storedNonzerosIn(P, P_src_, true)
         && storedNonzerosIn(A, A_src_, false);
}

int OSQPSolver::Impl::countUpperTriangle(const Eigen::Ref<const MatrixXF>& mat)
{
  // the diagonal is always stored (see initializeCostMatrix())
  int count{0};
  for (int col{0}; col < n_; ++col) {
    for (int row{0}; row < n_; ++row) {
      if (row > col)
        break;
      if (mat(row, col) != 0.0 || row == col)
        ++count;
    }
  }
  return count;
}

void OSQPSolver::Impl::initializeCostMatrix(const Eigen::Ref<const MatrixXF>& P)
{
  assert(!P_is_set_);
  assert(P.rows() == n_ && P.cols() == n_);
  P_nnz_ = countUpperTriangle(P);
  P_x_.setZero(P_nnz_);
  P_i_.setZero(P_nnz_);

  // Store the whole diagonal even where it is zero now, so a diagonal weight
  // that is zero at initialization can become nonzero later. This costs
  // nothing in solves: OSQP's KKT system always includes P's diagonal (it adds
  // sigma to it).
  P_nnz_ = 0;
  OSQPFloat val;
  for (int col{0}; col < n_; ++col) {
    P_p_(col) = P_nnz_;
    for (int row{0}; row < n_; ++row) {
      if (row > col)
        break;
      val = P(row, col);
      if (val != 0.0 || row == col) {
        P_x_(P_nnz_) = val;
        P_i_(P_nnz_++) = row;
      }
    }
  }
  P_p_(n_) = P_nnz_;
  P_.reset(
      OSQPCscMatrix_new(n_, n_, P_nnz_, P_x_.data(), P_i_.data(), P_p_.data()));
  P_is_set_ = true;
  P_stride_ = P.outerStride();
  densePositions(P_p_, P_i_, P_stride_, P_src_);
}

bool OSQPSolver::Impl::initializeCostMatrix(const SparseMatrixXF& P)
{
  assert(!P_is_set_);
  // The stored entries of the upper triangle, zero or not: the pattern is
  // the matrix's structure, so later values may be nonzero anywhere in it
  // (the indices are converted to OSQP's index type here, once)
  using detail::QPIndex;
  const QPIndex* outer{P.outerIndexPtr()};
  const QPIndex* inner{P.innerIndexPtr()};
  const OSQPFloat* values{P.valuePtr()};
  auto kept = [&](const QPIndex k, const QPIndex col) {
    return inner[k] <= col;
  };
  P_nnz_ = 0;
  for (QPIndex col{0}; col < n_; ++col) {
    bool diagonal{false};
    for (QPIndex k{outer[col]}; k < outer[col + 1]; ++k) {
      diagonal |= inner[k] == col;
      P_nnz_ += kept(k, col) ? 1 : 0;
    }
    if (!diagonal)
      return false;
  }
  P_x_.setZero(P_nnz_);
  P_i_.setZero(P_nnz_);
  P_src_.resize(static_cast<std::size_t>(P_nnz_));
  P_nnz_ = 0;
  for (QPIndex col{0}; col < n_; ++col) {
    P_p_(col) = P_nnz_;
    for (QPIndex k{outer[col]}; k < outer[col + 1]; ++k) {
      if (kept(k, col)) {
        P_x_(P_nnz_) = values[k];
        P_i_(P_nnz_) = static_cast<OSQPInt>(inner[k]);
        P_src_[static_cast<std::size_t>(P_nnz_++)] = static_cast<OSQPInt>(k);
      }
    }
  }
  P_p_(n_) = P_nnz_;
  P_direct_ = P_nnz_ == P.nonZeros();
  P_.reset(
      OSQPCscMatrix_new(n_, n_, P_nnz_, P_x_.data(), P_i_.data(), P_p_.data()));
  P_is_set_ = true;
  return true;
}

void OSQPSolver::Impl::initializeConstraintMatrix(
    const Eigen::Ref<const MatrixXF>& A)
{
  assert(!A_is_set_);
  assert(A.rows() == m_ && A.cols() == n_);
  A_nnz_ = A.count();
  A_x_.setZero(A_nnz_);
  A_i_.setZero(A_nnz_);

  A_nnz_ = 0;
  OSQPFloat val;
  for (int col{0}; col < n_; ++col) {
    A_p_(col) = A_nnz_;
    for (int row{0}; row < m_; ++row) {
      val = A(row, col);
      if (val != 0.0) {
        A_x_(A_nnz_) = val;
        A_i_(A_nnz_++) = row;
      }
    }
  }
  A_p_(n_) = A_nnz_;
  A_.reset(
      OSQPCscMatrix_new(m_, n_, A_nnz_, A_x_.data(), A_i_.data(), A_p_.data()));
  A_is_set_ = true;
  A_stride_ = A.outerStride();
  densePositions(A_p_, A_i_, A_stride_, A_src_);
}

void OSQPSolver::Impl::initializeConstraintMatrix(const SparseMatrixXF& A)
{
  assert(!A_is_set_);
  // every stored entry, zero or not (as for P)
  using detail::QPIndex;
  const QPIndex* outer{A.outerIndexPtr()};
  const QPIndex* inner{A.innerIndexPtr()};
  const OSQPFloat* values{A.valuePtr()};
  A_nnz_ = static_cast<OSQPInt>(outer[n_]);
  A_x_.setZero(A_nnz_);
  A_i_.setZero(A_nnz_);
  A_src_.resize(static_cast<std::size_t>(A_nnz_));
  A_nnz_ = 0;
  for (QPIndex col{0}; col < n_; ++col) {
    A_p_(col) = A_nnz_;
    for (QPIndex k{outer[col]}; k < outer[col + 1]; ++k) {
      A_x_(A_nnz_) = values[k];
      A_i_(A_nnz_) = static_cast<OSQPInt>(inner[k]);
      A_src_[static_cast<std::size_t>(A_nnz_++)] = static_cast<OSQPInt>(k);
    }
  }
  A_p_(n_) = A_nnz_;
  A_direct_ = A_nnz_ == A.nonZeros();
  A_.reset(
      OSQPCscMatrix_new(m_, n_, A_nnz_, A_x_.data(), A_i_.data(), A_p_.data()));
  A_is_set_ = true;
}

bool OSQPSolver::initialize(const detail::QP& qp, const QPBlockLayout&)
{
  // OSQP fixes its cost scaling at setup, from P and ||q||_inf. Set up with
  // setup_q_ so it depends on P alone, not on the x0 the QP was assembled
  // with (MPC::initializeSolver() uses a placeholder), then pass the actual q.
  Impl& impl{*impl_};
  const bool ok{
      qp.storage == detail::QPStorage::Sparse
          ? impl.initialize(qp.P_csc, qp.A_csc, setup_q_, qp.l, qp.u, settings_)
          : impl.initialize(qp.P, qp.A, setup_q_, qp.l, qp.u, settings_)};
  return ok && impl.updateCostVector(qp.q);
}

bool OSQPSolver::isInitialized() const noexcept
{
  return impl_->isInitialized();
}

bool OSQPSolver::updateMatrices(const detail::QP& qp,
                                const bool cost_matrix,
                                const bool constraint_matrix)
{
  if (!cost_matrix && !constraint_matrix)
    return true;
  Impl& impl{*impl_};
  // With scaling, a matrix update recomputes OSQP's scaling, the cost scaling
  // from the q it holds, so hold setup_q_ during the update (as at setup) and
  // pass the actual q again after it
  const bool rescales{impl.isInitialized() && settings_.scaling != 0};
  if (rescales && !impl.updateCostVector(setup_q_))
    return false;
  bool ok;
  if (qp.storage == detail::QPStorage::Sparse) {
    if (cost_matrix && constraint_matrix)
      ok = impl.updateMatrices(qp.P_csc, qp.A_csc);
    else if (cost_matrix)
      ok = impl.updateCostMatrix(qp.P_csc);
    else
      ok = impl.updateConstraintMatrix(qp.A_csc);
  } else if (cost_matrix && constraint_matrix)
    ok = impl.updateMatrices(qp.P, qp.A);
  else if (cost_matrix)
    ok = impl.updateCostMatrix(qp.P);
  else
    ok = impl.updateConstraintMatrix(qp.A);
  // restore q even if the matrix update failed
  if (rescales)
    ok = impl.updateCostVector(qp.q) && ok;
  return ok;
}

bool OSQPSolver::updateCostVector(const detail::QP& qp)
{
  return impl_->updateCostVector(qp.q);
}

bool OSQPSolver::updateBounds(const detail::QP& qp)
{
  return impl_->updateBounds(qp.l, qp.u);
}

SolveStatus OSQPSolver::solve() { return impl_->solve(); }

const Eigen::Map<const Eigen::VectorXd>
OSQPSolver::getSolutionMap() const noexcept
{
  return impl_->getSolutionMap();
}

bool OSQPSolver::getDualSolution(Eigen::Ref<Eigen::VectorXd> y) const
{
  return impl_->getDualSolution(y);
}

SolveInfo OSQPSolver::getSolveInfo() const noexcept
{
  return impl_->getSolveInfo();
}

void OSQPSolver::coldStart() noexcept { impl_->coldStart(); }

bool OSQPSolver::isWithinSparsityPattern(const detail::QP& qp) const
{
  if (qp.storage == detail::QPStorage::Sparse)
    return impl_->isWithinSparsityPattern(qp.P_csc, qp.A_csc);
  return impl_->isWithinSparsityPattern(qp.P, qp.A);
}

std::ostream& operator<<(std::ostream& os, const OSQPSettings& settings)
{
  const auto flags{os.flags()};
  os << std::boolalpha;

  using Linsys = OSQPSettings::LinsysSolverType;
  auto linsysSolverStr = [](const Linsys solver) {
    switch (solver) {
    case Linsys::DirectSolver:
      return "DirectSolver";
    case Linsys::IndirectSolver:
      return "IndirectSolver";
    default:
      return "UnknownSolver";
    }
  };
  auto precondStr = [](const OSQPSettings::PreconditionerType precond) {
    switch (precond) {
    case OSQPSettings::PreconditionerType::NoPreconditioner:
      return "NoPreconditioner";
    default:
      return "DiagonalPreconditioner";
    }
  };
  using AdaptiveRho = OSQPSettings::AdaptiveRhoType;
  auto adaptiveRhoStr = [](const AdaptiveRho type) {
    switch (type) {
    case AdaptiveRho::Disabled:
      return "Disabled";
    case AdaptiveRho::Time:
      return "Time";
    case AdaptiveRho::KKTError:
      return "KKTError";
    default:
      return "Iterations";
    }
  };

  os << "OSQP Solver Settings:\n"
     << "  adaptive_rho = " << adaptiveRhoStr(settings.adaptive_rho) << "\n"
     << "  adaptive_rho_fraction = " << settings.adaptive_rho_fraction << "\n"
     << "  adaptive_rho_interval = " << settings.adaptive_rho_interval << "\n"
     << "  adaptive_rho_tolerance = " << settings.adaptive_rho_tolerance << "\n"
     << "  allocate_solution = " << settings.allocate_solution << "\n"
     << "  alpha = " << settings.alpha << "\n"
     << "  cg_max_iter = " << settings.cg_max_iter << "\n"
     << "  cg_precond = " << precondStr(settings.cg_precond) << "\n"
     << "  cg_tol_fraction = " << settings.cg_tol_fraction << "\n"
     << "  cg_tol_reduction = " << settings.cg_tol_reduction << "\n"
     << "  check_dualgap = " << settings.check_dualgap << "\n"
     << "  check_termination = " << settings.check_termination << "\n"
     << "  delta = " << settings.delta << "\n"
     << "  device = " << settings.device << "\n"
     << "  eps_abs = " << settings.eps_abs << "\n"
     << "  eps_dual_inf = " << settings.eps_dual_inf << "\n"
     << "  eps_prim_inf = " << settings.eps_prim_inf << "\n"
     << "  eps_rel = " << settings.eps_rel << "\n"
     << "  linsys_solver = " << linsysSolverStr(settings.linsys_solver) << "\n"
     << "  max_iter = " << settings.max_iter << "\n"
     << "  polishing = " << settings.polishing << "\n"
     << "  polish_refine_iter = " << settings.polish_refine_iter << "\n"
     << "  profiler_level = " << settings.profiler_level << "\n"
     << "  rho = " << settings.rho << "\n"
     << "  rho_is_vec = " << settings.rho_is_vec << "\n"
     << "  scaled_termination = " << settings.scaled_termination << "\n"
     << "  scaling = " << settings.scaling << "\n"
     << "  sigma = " << settings.sigma << "\n"
     << "  time_limit = " << settings.time_limit << "\n"
     << "  verbose = " << settings.verbose << "\n"
     << "  warm_starting = " << settings.warm_starting;
  os.flags(flags);
  return os;
}

} // namespace affine_mpc
