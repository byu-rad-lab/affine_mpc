#include "affine_mpc/daqp_solver.hpp"

#include <Eigen/Core>
#include <algorithm>
#include <cassert>
#include <memory>
#include <utility>
#include <vector>

#include "affine_mpc/detail/qp.hpp"
#include "affine_mpc/qp_solver.hpp"
#include "affine_mpc/solve_status.hpp"

// DAQP's C API (private: these headers are not installed with affine_mpc)
extern "C" {
#include <daqp/api.h>
#include <daqp/utils.h>
}

namespace affine_mpc {

namespace {

SolveStatus daqpExitFlagToSolveStatus(const int exitflag)
{
  switch (exitflag) {
  case DAQP_EXIT_OPTIMAL:
  case DAQP_EXIT_SOFT_OPTIMAL:
    return SolveStatus::Success;
  case DAQP_EXIT_OPTIMAL_INEXACT:
    return SolveStatus::SolvedInaccurate;
  case DAQP_EXIT_INFEASIBLE:
    return SolveStatus::PrimalInfeasible;
  case DAQP_EXIT_UNBOUNDED:
    return SolveStatus::DualInfeasible;
  case DAQP_EXIT_ITERLIMIT:
    return SolveStatus::MaxIterReached;
  case DAQP_EXIT_TIMELIMIT:
    return SolveStatus::TimeLimitReached;
  default: // cycling, nonconvex, overdetermined, unsupported, not solved
    return SolveStatus::OtherFailure;
  }
}

// DAQP treats bounds beyond DAQP_INF as infinite
double clampInfinity(const double value)
{
  return std::clamp(value, -DAQP_INF, DAQP_INF);
}

} // namespace

struct DAQPSolver::Impl
{
  using RowMajorMatrix =
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

  // A row of A with a single nonzero, a bound on one variable
  struct SimpleRow
  {
    Eigen::Index row, var;
  };

  explicit Impl(const DAQPSettings& s) : settings{s} {}

  ~Impl()
  {
    // DAQP frees the settings it owns; these are ours
    work.settings = nullptr;
    free_daqp_workspace(&work);
    free_daqp_ldp(&work);
  }

  Impl(const Impl&) = delete;
  Impl& operator=(const Impl&) = delete;

  // The general rows of A into DAQP's row-major buffer
  void copyGeneralRows(const detail::QP& qp)
  {
    const Eigen::Index n{qp.A.cols()};
    for (Eigen::Index i{0}; i < num_general; ++i) {
      const Eigen::Index row{general_rows[static_cast<std::size_t>(i)]};
      for (Eigen::Index j{0}; j < n; ++j)
        A_general(i, j) = qp.A(row, j);
    }
  }

  // l <= A z <= u as DAQP's bounds: the simple rows intersected per variable,
  // then the general rows
  void convertBounds(const detail::QP& qp)
  {
    bu.head(num_vars).setConstant(DAQP_INF);
    bl.head(num_vars).setConstant(-DAQP_INF);
    for (const SimpleRow& s : simple_rows) {
      const double a{qp.A(s.row, s.var)};
      if (a == 0.0)
        continue; // outside the pattern (see isWithinSparsityPattern())
      double lo{qp.l(s.row) / a}, hi{qp.u(s.row) / a};
      if (a < 0.0)
        std::swap(lo, hi);
      bl(s.var) = std::max(bl(s.var), clampInfinity(lo));
      bu(s.var) = std::min(bu(s.var), clampInfinity(hi));
    }
    for (Eigen::Index i{0}; i < num_general; ++i) {
      const Eigen::Index row{general_rows[static_cast<std::size_t>(i)]};
      bl(num_vars + i) = clampInfinity(qp.l(row));
      bu(num_vars + i) = clampInfinity(qp.u(row));
    }
  }

  // Form DAQP's problem from the parts in mask and those pending
  int update(const int mask)
  {
    const int flag{daqp_update_ldp(mask | pending, &work, &problem)};
    // DAQP keeps the parts of a failed update pending itself
    pending = 0;
    return flag;
  }

  DAQPSettings settings;
  ::DAQPSettings daqp_settings{};
  ::DAQPProblem problem{};
  ::DAQPWorkspace work{};
  ::DAQPResult result{};
  bool initialized{false};
  Eigen::Index num_vars{0}, num_general{0};
  std::vector<SimpleRow> simple_rows;
  std::vector<Eigen::Index> general_rows;
  RowMajorMatrix A_general;
  Eigen::VectorXd bu, bl; // simple bounds, then general rows
  Eigen::VectorXd x, lam;
  Eigen::VectorXd Px;     // P x for the objective of proximal iterations
  std::vector<int> sense; // zero: no constraint is marked active
  int pending{0};         // DAQP_UPDATE_* parts to form before the next solve
};

DAQPSolver::DAQPSolver(const DAQPSettings& settings) :
    impl_{std::make_unique<Impl>(settings)}
{}

DAQPSolver::~DAQPSolver() = default;

bool DAQPSolver::initialize(const detail::QP& qp, const QPBlockLayout&)
{
  Impl& d{*impl_};
  if (d.initialized || qp.storage != detail::QPStorage::Dense)
    return false;
  const Eigen::Index n{qp.P.rows()}, m{qp.A.rows()};
  assert(qp.P.cols() == n && qp.A.cols() == n && qp.q.size() == n);
  assert(qp.l.size() == m && qp.u.size() == m);

  // Rows with a single nonzero are bounds on their variable
  d.simple_rows.clear();
  d.general_rows.clear();
  for (Eigen::Index row{0}; row < m; ++row) {
    Eigen::Index count{0}, var{0};
    for (Eigen::Index j{0}; j < n; ++j) {
      if (qp.A(row, j) != 0.0) {
        ++count;
        var = j;
      }
    }
    if (count == 1)
      d.simple_rows.push_back({row, var});
    else
      d.general_rows.push_back(row);
  }
  d.num_vars = n;
  d.num_general = static_cast<Eigen::Index>(d.general_rows.size());
  d.A_general.resize(d.num_general, n);
  d.copyGeneralRows(qp);
  // The bounds of the QP passed here depend on a placeholder initial state
  // and may be infeasible, so DAQP is set up without bounds and receives them
  // with the first solve
  d.bu.setConstant(n + d.num_general, DAQP_INF);
  d.bl.setConstant(n + d.num_general, -DAQP_INF);
  d.x.setZero(n);
  d.Px.setZero(n);
  d.lam.setZero(n + d.num_general);
  d.sense.assign(static_cast<std::size_t>(n + d.num_general), 0);

  // P is symmetric, so its column-major storage is also row-major (DAQP
  // reads (P_ij + P_ji) / 2); DAQP does not write P or q.
  ::DAQPProblem& p{d.problem};
  p.n = static_cast<int>(n);
  p.m = static_cast<int>(n + d.num_general);
  p.ms = static_cast<int>(n);
  p.H = const_cast<double*>(qp.P.data());
  p.f = const_cast<double*>(qp.q.data());
  p.A = d.A_general.data();
  p.bupper = d.bu.data();
  p.blower = d.bl.data();
  p.sense = d.sense.data();
  p.break_points = nullptr;
  p.nh = 0;
  p.problem_type = 0;

  daqp_default_settings(&d.daqp_settings);
  d.daqp_settings.primal_tol = d.settings.primal_tol;
  d.daqp_settings.dual_tol = d.settings.dual_tol;
  d.daqp_settings.iter_limit = d.settings.iter_limit;
  d.daqp_settings.time_limit = d.settings.time_limit;
  d.daqp_settings.eps_prox = d.settings.eps_prox;
  // Eliminating equality constraints allocates in the updates and is meant
  // for solves from scratch
  d.daqp_settings.eq_reduction = DAQP_EQ_REDUCTION_OFF;
  d.work.settings = &d.daqp_settings;

  double setup_time{0.0};
  // on failure DAQP frees its workspace and drops the settings pointer
  if (setup_daqp(&d.problem, &d.work, &setup_time) < 0)
    return false;
  d.result.x = d.x.data();
  d.result.lam = d.lam.data();
  d.result.exitflag = 0; // not solved yet: OtherFailure
  d.convertBounds(qp);
  d.pending = DAQP_UPDATE_d;
  d.initialized = true;
  return true;
}

bool DAQPSolver::isInitialized() const noexcept { return impl_->initialized; }

bool DAQPSolver::updateMatrices(const detail::QP& qp,
                                const bool cost_matrix,
                                const bool constraint_matrix)
{
  Impl& d{*impl_};
  if (!d.initialized)
    return false;
  int mask{0};
  if (cost_matrix)
    mask |= DAQP_UPDATE_Rinv;
  if (constraint_matrix) {
    d.copyGeneralRows(qp);
    // the coefficients of the simple rows scale their bounds
    d.convertBounds(qp);
    mask |= DAQP_UPDATE_M | DAQP_UPDATE_d;
  }
  // Factorize now, so MPC::prepare() does this work before solve().
  // Infeasible bounds (l > u after the conversion) are not a rejected matrix
  // update: DAQP keeps the update pending and solve() reports them.
  const int flag{d.update(mask)};
  return flag >= 0 || flag == DAQP_EXIT_INFEASIBLE;
}

bool DAQPSolver::updateCostVector(const detail::QP&)
{
  // q is read in place
  impl_->pending |= DAQP_UPDATE_v;
  return impl_->initialized;
}

bool DAQPSolver::updateBounds(const detail::QP& qp)
{
  Impl& d{*impl_};
  if (!d.initialized)
    return false;
  d.convertBounds(qp);
  d.pending |= DAQP_UPDATE_d;
  return true;
}

SolveStatus DAQPSolver::solve()
{
  Impl& d{*impl_};
  if (!d.initialized)
    return SolveStatus::NotInitialized;
  if (d.pending != 0 || (d.work.state & DAQP_STATE_PENDING) != 0) {
    const int flag{d.update(0)};
    if (flag < 0) {
      // e.g. bounds with l > u, or a P that is not positive definite with
      // proximal iterations disabled
      d.result.exitflag = flag;
      d.result.iter = 0;
      d.result.solve_time = 0.0;
      return daqpExitFlagToSolveStatus(flag);
    }
  }
  daqp_solve(&d.result, &d.work);
  if (d.work.n_prox > 0) {
    // DAQP's objective of the proximal iterations is approximate
    const Eigen::Map<const Eigen::MatrixXd> P{d.problem.H, d.num_vars,
                                              d.num_vars};
    const Eigen::Map<const Eigen::VectorXd> q{d.problem.f, d.num_vars};
    d.Px.noalias() = P * d.x;
    d.result.fval = 0.5 * d.x.dot(d.Px) + q.dot(d.x);
  }
  return daqpExitFlagToSolveStatus(d.result.exitflag);
}

const Eigen::Map<const Eigen::VectorXd>
DAQPSolver::getSolutionMap() const noexcept
{
  if (!impl_->initialized)
    return Eigen::Map<const Eigen::VectorXd>{nullptr, 0};
  return Eigen::Map<const Eigen::VectorXd>{impl_->x.data(), impl_->num_vars};
}

SolveInfo DAQPSolver::getSolveInfo() const noexcept
{
  const Impl& d{*impl_};
  if (!d.initialized)
    return SolveInfo{};
  // mpc_solve_time and prepare_time are only known to MPC
  return SolveInfo{daqpExitFlagToSolveStatus(d.result.exitflag),
                   d.result.iter,
                   d.result.fval,
                   d.result.solve_time,
                   0.0,
                   0.0};
}

void DAQPSolver::coldStart() noexcept
{
  // The zero sense array marks no constraint active
  impl_->pending |= DAQP_UPDATE_sense;
}

bool DAQPSolver::isWithinSparsityPattern(const detail::QP& qp) const
{
  const Impl& d{*impl_};
  if (!d.initialized || qp.storage != detail::QPStorage::Dense)
    return false;
  const Eigen::Index n{qp.A.cols()};
  for (const Impl::SimpleRow& s : d.simple_rows)
    for (Eigen::Index j{0}; j < n; ++j)
      if (j != s.var && qp.A(s.row, j) != 0.0)
        return false;
  return true;
}

const DAQPSettings& DAQPSolver::getSettings() const noexcept
{
  return impl_->settings;
}

int DAQPSolver::getNumSimpleBoundRows() const noexcept
{
  return static_cast<int>(impl_->simple_rows.size());
}

int DAQPSolver::getNumGeneralRows() const noexcept
{
  return static_cast<int>(impl_->num_general);
}

bool DAQPSolver::usesProximalIterations() const noexcept
{
  return impl_->initialized && impl_->work.n_prox > 0;
}

} // namespace affine_mpc
