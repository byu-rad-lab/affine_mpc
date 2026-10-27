#include "affine_mpc/mpc.hpp"

#include <Eigen/Core>
#include <Eigen/SparseCore>
#include <algorithm>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <osqp.h>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "affine_mpc/detail/vector_parameterization.hpp"
#include "affine_mpc/parameterization.hpp"
#include "affine_mpc/solve_status.hpp"
#include "eigen_compat.hpp" // remove this once Eigen 3.5 is required
#include "formulation.hpp"
#include "qp_builder.hpp"

using namespace Eigen;
// revert back to this once Eigen 3.5 is required
// namespace ph = Eigen::placeholders;

namespace affine_mpc {

namespace { // utilities for this file

constexpr int validateStateDim(int state_dim)
{
  if (state_dim <= 0)
    throw std::invalid_argument(
        "[MPC::validateStateDim] state_dim must be positive.");
  return state_dim;
}

constexpr int validateInputDim(int input_dim)
{
  if (input_dim <= 0)
    throw std::invalid_argument(
        "[MPC::validateInputDim] input_dim must be positive.");
  return input_dim;
}

// The state bound options need saturate_states, and the bounded states must
// be valid indices. Control-point slew relies on B-spline properties.
const Options& validateOptions(const Options& opts,
                               int state_dim,
                               const Parameterization& param)
{
  if (!opts.saturate_states
      && (opts.soft_state_bounds || !opts.bounded_states.empty()))
    throw std::invalid_argument(
        "[MPC::validateOptions] soft_state_bounds and bounded_states "
        "require saturate_states.");
  int previous{-1};
  for (const int i : opts.bounded_states) {
    if (i <= previous || i >= state_dim)
      throw std::invalid_argument(
          "[MPC::validateOptions] bounded_states must be strictly "
          "increasing state indices in [0, state_dim).");
    previous = i;
  }
  if (opts.num_general_constraints < 0 || opts.num_terminal_constraints < 0)
    throw std::invalid_argument(
        "[MPC::validateOptions] num_general_constraints and "
        "num_terminal_constraints must be non-negative.");
  if (opts.slew_control_points && param.type != Parameterization::Type::BSpline)
    throw std::invalid_argument(
        "[MPC::validateOptions] slew_control_points requires a B-spline "
        "parameterization: differences of custom basis coefficients do not "
        "bound input changes.");
  return opts;
}

// Size checks for setters, which are not on the solve path. The message is
// only built when the check fails.
void validateSize(Index size,
                  Index expected,
                  const char* func,
                  const char* name)
{
  if (size != expected)
    throw std::invalid_argument(std::string{"[MPC::"} + func + "] " + name
                                + " must have size " + std::to_string(expected)
                                + ", got " + std::to_string(size) + ".");
}

void validateShape(const Ref<const MatrixXd>& mat,
                   Index rows,
                   Index cols,
                   const char* func,
                   const char* name)
{
  if (mat.rows() != rows || mat.cols() != cols)
    throw std::invalid_argument(
        std::string{"[MPC::"} + func + "] " + name + " must be "
        + std::to_string(rows) + "x" + std::to_string(cols) + ", got "
        + std::to_string(mat.rows()) + "x" + std::to_string(mat.cols()) + ".");
}

} // namespace

MPC::MPC(int state_dim,
         int input_dim,
         const Parameterization& param,
         const Options& opts,
         Formulation formulation) :
    state_dim_{validateStateDim(state_dim)},
    input_dim_{validateInputDim(input_dim)},
    horizon_steps_{param.horizon_steps},
    num_ctrl_pts_{param.num_control_points},
    x_traj_dim_{state_dim * param.horizon_steps},
    u_traj_dim_{input_dim * param.horizon_steps},
    ctrls_dim_{input_dim * param.num_control_points},
    opts_{validateOptions(opts, state_dim_, param)},
    input_param_{input_dim_, param},
    bounded_states_{detail::boundedStates(opts_, state_dim_)},
    solution_map_{nullptr, 0},
    solver_{nullptr},
    data_{state_dim_, input_dim_, horizon_steps_, num_ctrl_pts_, opts_},
    // nothing is in the QP yet, so initializeSolver() assembles everything
    pending_{DataChangeSet::all()},
    model_set_{false},
    model_from_continuous_{false},
    u_lims_set_{false},
    x_lims_set_{false},
    x_penalties_set_{false},
    general_set_{false},
    terminal_set_{false},
    slew0_rate_set_{false},
    ctrls_slew_rate_set_{false},
    solver_initialized_{false},
    model_sparsity_{ModelSparsity::InitialNonzeros},
    mpc_solve_time_{0.0},
    prepare_time_{0.0},
    unsolved_prepare_time_{0.0},
    discretizer_{state_dim_, input_dim_},
    formulation_{formulation},
    qp_builder_{std::make_unique<detail::QPBuilder>(
        formulation, state_dim_, input_param_, opts_)}
{
  // the options decided the terms and so the QP's sizes
  solver_ = std::make_unique<OSQPSolver>(qp_builder_->numVariables(),
                                         qp_builder_->numConstraints());
  qp_builder_->writeConstantParts(data_);
}

MPC::MPC(int state_dim,
         int input_dim,
         const Parameterization& param,
         Formulation formulation) :
    MPC(state_dim, input_dim, param, Options{}, formulation)
{}

MPC::MPC(int state_dim,
         int input_dim,
         int horizon_steps,
         const Options& opts,
         Formulation formulation) :
    MPC(state_dim,
        input_dim,
        Parameterization::moveBlocking(horizon_steps, horizon_steps),
        opts,
        formulation)
{}

MPC::MPC(int state_dim,
         int input_dim,
         int horizon_steps,
         Formulation formulation) :
    MPC(state_dim, input_dim, horizon_steps, Options{}, formulation)
{}

MPC::~MPC() = default;

const detail::QP& MPC::qp() const noexcept { return qp_builder_->qp(); }

const Eigen::MatrixXd& MPC::getQPCostMatrix() const
{
  return qp_builder_->denseCostMatrix();
}

const detail::QPBuilder& MPC::qpBuilder() const noexcept
{
  return *qp_builder_;
}

namespace {

// Write the QP's nonzero entries into the public form, whichever storage the
// formulation uses: both matrices CSC, and P full symmetric, mirrored from its
// upper triangle, which is all the solver reads (a dense P from matrix
// products is symmetric only to round-off).
void toQuadraticProgram(const detail::QP& qp, QuadraticProgram& out)
{
  const auto nonzero{[](const Index&, const Index&, const double& value) {
    return value != 0.0;
  }};
  if (qp.storage == detail::QPStorage::Dense) {
    MatrixXd P{qp.P.triangularView<Upper>()};
    P.triangularView<StrictlyLower>() = P.transpose();
    out.P = P.sparseView();
    out.A = qp.A.sparseView();
  } else {
    // P_csc holds the upper triangle: mirror the entries below the diagonal
    std::vector<Triplet<double>> entries;
    entries.reserve(2 * static_cast<std::size_t>(qp.P_csc.nonZeros()));
    for (Index j{0}; j < qp.P_csc.outerSize(); ++j)
      for (detail::QP::SparseMatrix::InnerIterator it{qp.P_csc, j}; it; ++it) {
        if (it.value() == 0.0)
          continue;
        entries.emplace_back(it.row(), it.col(), it.value());
        if (it.row() != it.col())
          entries.emplace_back(it.col(), it.row(), it.value());
      }
    out.P.resize(qp.P_csc.rows(), qp.P_csc.cols());
    out.P.setFromTriplets(entries.begin(), entries.end());
    out.A = qp.A_csc;
    out.A.prune(nonzero);
  }
  out.P.makeCompressed();
  out.A.makeCompressed();
  out.q = qp.q;
  out.l = qp.l;
  out.u = qp.u;
}

} // namespace

QuadraticProgram MPC::getQP() const
{
  QuadraticProgram out;
  toQuadraticProgram(qp(), out);
  return out;
}

void MPC::assembleQP(const Ref<const VectorXd>& x0, QuadraticProgram& out)
{
  validateSize(x0.size(), state_dim_, "assembleQP", "x0");
  checkSetup("assembleQP", "assembling the QP");
  // Write the pending changes and x0 into the QP without sending them: the
  // changes stay pending, so the next prepare() or solve() writes them again
  // and sends them (the x0 parts are written by every solve anyway)
  if (!pending_.empty())
    (void)qp_builder_->apply(pending_, data_);
  data_.x0 = x0;
  qp_builder_->applyX0(data_);
  toQuadraticProgram(qp(), out);
}

std::vector<QPBlock> MPC::getQPVariableBlocks() const
{
  const int num_ctrl{input_param_.numCoefficients()};
  const int own{qp_builder_->formulation().numVariables()};
  std::vector<QPBlock> blocks{{"control_points", 0, num_ctrl}};
  if (own > num_ctrl)
    blocks.push_back({"states", num_ctrl, own - num_ctrl});
  if (qp_builder_->numVariables() > own)
    blocks.push_back({"state_slacks", own, qp_builder_->numVariables() - own});
  return blocks;
}

std::vector<QPBlock> MPC::getQPConstraintBlocks() const
{
  const detail::QPRows& rows{qp_builder_->rows()};
  std::vector<QPBlock> blocks;
  for (const auto& [name, block] :
       {std::pair<const char*, detail::RowBlock>{"dynamics", rows.dynamics},
        {"input_bounds", rows.input_bounds},
        {"initial_slew", rows.initial_slew},
        {"control_point_slew", rows.control_point_slew},
        {"state_bounds", rows.state_bounds},
        {"general_constraints", rows.general_constraints},
        {"terminal_constraints", rows.terminal_constraints}})
    if (block.rows > 0)
      blocks.push_back({name, block.offset, block.rows});
  return blocks;
}

int MPC::getNumQPVariables() const noexcept
{
  return qp_builder_->numVariables();
}

int MPC::getNumQPConstraints() const noexcept
{
  return qp_builder_->numConstraints();
}

void MPC::checkSetup(const char* caller, const char* action) const
{
  const auto missing{[&](const char* what, const char* reason) {
    throw std::logic_error(std::string{"[MPC::"} + caller + "] " + what
                           + " must be set before " + action + reason + ".");
  }};
  if (!model_set_)
    missing("Model", "");
  if (!u_lims_set_)
    missing("Input limits", "");
  if (opts_.slew_control_points && !ctrls_slew_rate_set_)
    missing("Slew rate", " (slew_control_points is enabled)");
  if (opts_.slew_initial_input && !slew0_rate_set_)
    missing("Initial slew rate", " (slew_initial_input is enabled)");
  if (opts_.saturate_states && !x_lims_set_)
    missing("State limits", " (saturate_states is enabled)");
  if (opts_.soft_state_bounds && !x_penalties_set_)
    missing("State bound penalties", " (soft_state_bounds is enabled)");
  if (opts_.num_general_constraints > 0 && !general_set_)
    missing("General constraints", " (num_general_constraints is positive)");
  if (opts_.num_terminal_constraints > 0 && !terminal_set_)
    missing("Terminal constraints", " (num_terminal_constraints is positive)");
}

bool MPC::initializeSolver(const ModelSparsity model_sparsity)
{
  return initializeSolver(OSQPSolver::getRecommendedSettings(), model_sparsity);
}

bool MPC::initializeSolver(const OSQPSettings& solver_settings,
                           const ModelSparsity model_sparsity)
{
  checkSetup("initializeSolver", "initializing solver");

  if (solver_initialized_)
    return true;

  const bool sparse{qp().storage == detail::QPStorage::Sparse};
  if (model_sparsity == ModelSparsity::Full && !sparse)
    throw std::invalid_argument(
        "[MPC::initializeSolver] ModelSparsity::Full is not supported yet "
        "by the condensed formulation; use ModelSparsity::InitialNonzeros.");
  model_sparsity_ = model_sparsity;
  // The sparse QP declares every entry of Ad and Bd at construction; with
  // InitialNonzeros, rebuild it with only the nonzero entries of the model
  // (the condensed QP is dense, and the solver takes its nonzeros below)
  if (sparse && model_sparsity == ModelSparsity::InitialNonzeros) {
    const detail::ModelEntries entries{detail::ModelEntries::nonzerosOf(data_)};
    if (!entries.Ad_full || !entries.Bd_full) {
      qp_builder_ = std::make_unique<detail::QPBuilder>(
          Formulation::Sparse, state_dim_, input_param_, opts_, entries);
      qp_builder_->writeConstantParts(data_);
      pending_ = DataChangeSet::all();
    }
  }

  // x0 only affects vector terms (q, l, u) so the values don't matter to set
  // initial sparsity structure. Setting to ones. The solver is not
  // initialized yet, so only the assembled QP matters here, not the result.
  VectorXd x_full{state_dim_};
  x_full.setOnes();
  (void)qpUpdateX0(x_full);

  const detail::QP& qp_data{qp()};
  if (qp_data.storage == detail::QPStorage::Sparse)
    solver_initialized_ =
        solver_->initialize(qp_data.P_csc, qp_data.A_csc, qp_data.q, qp_data.l,
                            qp_data.u, solver_settings);
  else
    solver_initialized_ = solver_->initialize(
        qp_data.P, qp_data.A, qp_data.q, qp_data.l, qp_data.u, solver_settings);
  // According to Eigen's documentation, use "placement new" syntax to update
  // solution_map_ with the new pointer to the solution vector. This does not
  // allocate new memory that needs to be deleted, it just updates the pointer.
  // This location in memory is managed by the solver_ object and is unchanged
  // while the solver_ is in scope. solution_map_ will function like an Eigen
  // VectorXd object that automatically updates when solve() is called. Using
  // Eigen Map allows the solution to be accessed without copying memory.
  new (&solution_map_) Map<const VectorXd>(solver_->getSolutionMap());
  return solver_initialized_;
}

SolveStatus MPC::solve(const Ref<const VectorXd>& x0)
{
  validateSize(x0.size(), state_dim_, "solve", "x0");
  if (!solver_initialized_)
    return SolveStatus::NotInitialized;

  // SolveInfo describes one cycle: the prepare() calls since the last solve
  // and this solve
  prepare_time_ = unsolved_prepare_time_;
  unsolved_prepare_time_ = 0.0;

  using Clock = std::chrono::steady_clock;
  const Clock::time_point start{Clock::now()};
  if (!applyPendingChanges() || !applyX0(x0)) {
    mpc_solve_time_ =
        std::chrono::duration<double>(Clock::now() - start).count();
    return SolveStatus::UpdateFailed;
  }
  const SolveStatus status{solver_->solve()};

  // update u_prev after solve rather than before so user can manually
  // overwrite it between solves if desired. Other statuses leave an iterate
  // that is not a solution (e.g. an infeasibility certificate), which would
  // corrupt the initial slew constraint of every later solve.
  if (usesPreviousInput()
      && (status == SolveStatus::Success
          || status == SolveStatus::SolvedInaccurate)) {
    getNextInput(data_.u_prev);
    pending_ |= DataChange::PreviousInput;
  }
  mpc_solve_time_ = std::chrono::duration<double>(Clock::now() - start).count();
  return status;
}

SolveStatus MPC::prepare()
{
  if (!solver_initialized_)
    return SolveStatus::NotInitialized;

  using Clock = std::chrono::steady_clock;
  const Clock::time_point start{Clock::now()};
  const bool applied{applyPendingChanges()};
  unsolved_prepare_time_ +=
      std::chrono::duration<double>(Clock::now() - start).count();
  return applied ? SolveStatus::Success : SolveStatus::UpdateFailed;
}

bool MPC::qpUpdateX0(const Ref<const VectorXd>& x0)
{
  const bool applied{applyPendingChanges()};
  return applyX0(x0) && applied;
}

bool MPC::applyPendingChanges()
{
  // Nothing changed is the common case, which costs only this check
  if (pending_.empty())
    return true;
  const detail::FormulationImpl::Update written{
      qp_builder_->apply(pending_, data_)};
  pending_ = {};
  // Before initializeSolver() the QP is only assembled
  if (!solver_initialized_)
    return true;

  // Send only the changed parts. When both matrices changed, one combined
  // update rescales and refactors once instead of twice. The parts that
  // applyX0() writes are sent there.
  if (!written.P.empty() || !written.A.empty()) {
    if (!sendMatrices(!written.P.empty(), !written.A.empty()))
      pending_ |= written.P | written.A;
  }
  if (!written.q.empty() && !qp_builder_->applyX0WritesCostVector()
      && !solver_->updateCostVector(qp().q))
    pending_ |= written.q;
  if (!written.bounds.empty() && !qp_builder_->applyX0WritesBounds()
      && !solver_->updateBounds(qp().l, qp().u))
    pending_ |= written.bounds;
  return pending_.empty();
}

bool MPC::applyX0(const Ref<const VectorXd>& x0)
{
  data_.x0 = x0;
  qp_builder_->applyX0(data_);
  if (!solver_initialized_)
    return true;
  bool success{true};
  if (qp_builder_->applyX0WritesCostVector())
    success &= solver_->updateCostVector(qp().q);
  if (qp_builder_->applyX0WritesBounds())
    success &= solver_->updateBounds(qp().l, qp().u);
  return success;
}

bool MPC::sendMatrices(const bool P, const bool A)
{
  const detail::QP& qp_data{qp()};
  if (qp_data.storage == detail::QPStorage::Sparse) {
    if (P && A)
      return solver_->updateMatrices(qp_data.P_csc, qp_data.A_csc);
    if (P)
      return solver_->updateCostMatrix(qp_data.P_csc);
    return solver_->updateConstraintMatrix(qp_data.A_csc);
  }
  if (P && A)
    return solver_->updateMatrices(qp_data.P, qp_data.A);
  if (P)
    return solver_->updateCostMatrix(qp_data.P);
  return solver_->updateConstraintMatrix(qp_data.A);
}

bool MPC::modelWithinTrackedEntries() const
{
  return !solver_initialized_
         || qp_builder_->formulation().modelWithinTrackedEntries(data_);
}

bool MPC::isWithinSparsityPattern() const
{
  if (!solver_initialized_)
    return true;
  if (!modelWithinTrackedEntries())
    return false;
  const detail::QP& qp_data{qp()};
  if (qp_data.storage == detail::QPStorage::Sparse)
    return solver_->isWithinSparsityPattern(qp_data.P_csc, qp_data.A_csc);
  return solver_->isWithinSparsityPattern(qp_data.P, qp_data.A);
}

void MPC::getNextInput(Ref<VectorXd> u0) const
{
  assert(solver_initialized_);
  validateSize(u0.size(), input_dim_, "getNextInput", "u0");
  // Assumes that the control points are first elements of solution
  input_param_.evaluate(0, solution_map_, u0);
}

void MPC::getInputControlPoints(Ref<VectorXd> control_points) const
{
  assert(solver_initialized_);
  validateSize(control_points.size(), ctrls_dim_, "getInputControlPoints",
               "control_points");
  // Assumes that the control points are first elements of solution
  control_points = solution_map_.head(ctrls_dim_);
}

void MPC::getInputTrajectory(Ref<VectorXd> u_traj) const
{
  assert(solver_initialized_);
  validateSize(u_traj.size(), u_traj_dim_, "getInputTrajectory", "u_traj");
  // Assumes that the control points are first elements of solution
  input_param_.evaluateTrajectory(solution_map_, u_traj);
}

void MPC::getPredictedStateTrajectory(Ref<VectorXd> x_traj) const
{
  assert(solver_initialized_);
  validateSize(x_traj.size(), x_traj_dim_, "getPredictedStateTrajectory",
               "x_traj");
  qp_builder_->predictStates(solution_map_, x_traj);
}

void MPC::propagateModel(const Ref<const VectorXd>& x0,
                         const Ref<const VectorXd>& u,
                         Ref<VectorXd> x_next) const
{
  if (!model_set_)
    throw std::logic_error(
        "[MPC::propagateModel] Model must be set before propagation");
  validateSize(x0.size(), state_dim_, "propagateModel", "x");
  validateSize(u.size(), input_dim_, "propagateModel", "u");
  validateSize(x_next.size(), state_dim_, "propagateModel", "x_next");
  // do not use noalias here since x_next could be an alias of x0
  x_next = data_.Ad * x0 + data_.Bd * u + data_.wd;
}

void MPC::setModelDiscrete(const Ref<const MatrixXd>& Ad,
                           const Ref<const MatrixXd>& Bd,
                           const Ref<const VectorXd>& wd)
{
  validateShape(Ad, state_dim_, state_dim_, "setModelDiscrete", "Ad");
  validateShape(Bd, state_dim_, input_dim_, "setModelDiscrete", "Bd");
  validateSize(wd.size(), state_dim_, "setModelDiscrete", "wd");

  data_.Ad = Ad;
  data_.Bd = Bd;
  data_.wd = wd;
  // not checked in Release, where it would cost every model update
  assert(modelWithinTrackedEntries()
         && "[MPC::setModelDiscrete] Ad or Bd has a nonzero entry the "
            "solver does not track (see ModelSparsity).");
  model_set_ = true;
  model_from_continuous_ = false;
  pending_ |= DataChange::Model | DataChange::ModelOffset;
}

void MPC::setModelContinuous2Discrete(const Ref<const MatrixXd>& Ac,
                                      const Ref<const MatrixXd>& Bc,
                                      const Ref<const VectorXd>& wc,
                                      double dt,
                                      double tol)
{
  validateShape(Ac, state_dim_, state_dim_, "setModelContinuous2Discrete",
                "Ac");
  validateShape(Bc, state_dim_, input_dim_, "setModelContinuous2Discrete",
                "Bc");
  validateSize(wc.size(), state_dim_, "setModelContinuous2Discrete", "wc");
  // Discretization stays in the setter: it is needed once per call anyway, and
  // the user controls when the setter runs. Only the QP work is deferred.
  discretizer_.discretize(Ac, Bc, wc, dt, data_.Ad, data_.Bd, data_.wd, tol);
  // not checked in Release, where it would cost every model update
  assert(modelWithinTrackedEntries()
         && "[MPC::setModelContinuous2Discrete] The discrete Ad or Bd has "
            "a nonzero entry the solver does not track (see ModelSparsity).");
  model_set_ = true;
  model_from_continuous_ = true;
  pending_ |= DataChange::Model | DataChange::ModelOffset;
}

void MPC::setModelOffsetDiscrete(const Ref<const VectorXd>& wd)
{
  validateSize(wd.size(), state_dim_, "setModelOffsetDiscrete", "wd");
  if (!model_set_)
    throw std::logic_error("[MPC::setModelOffsetDiscrete] Model must be "
                           "set before its offset.");
  data_.wd = wd;
  pending_ |= DataChange::ModelOffset;
}

void MPC::setModelOffsetContinuous2Discrete(const Ref<const VectorXd>& wc)
{
  validateSize(wc.size(), state_dim_, "setModelOffsetContinuous2Discrete",
               "wc");
  if (!model_from_continuous_)
    throw std::logic_error(
        "[MPC::setModelOffsetContinuous2Discrete] The model must be last "
        "set with setModelContinuous2Discrete(), whose discretization is "
        "reused.");
  discretizer_.discretizeOffset(wc, data_.wd);
  pending_ |= DataChange::ModelOffset;
}

void MPC::setWeights(const Ref<const VectorXd>& Q_diag,
                     const Ref<const VectorXd>& R_diag)
{
  setStateWeights(Q_diag);
  setInputWeights(R_diag);
}

void MPC::setWeights(const Ref<const VectorXd>& Q_diag,
                     const Ref<const VectorXd>& Qf_diag,
                     const Ref<const VectorXd>& R_diag)
{
  setStateWeights(Q_diag, Qf_diag);
  setInputWeights(R_diag);
}

void MPC::setStateWeights(const Ref<const VectorXd>& Q_diag)
{
  validateSize(Q_diag.size(), state_dim_, "setStateWeights", "Q_diag");
  if (Q_diag.minCoeff() < 0.0)
    throw std::invalid_argument(
        "[MPC::setStateWeights] State weights must be non-negative.");
  data_.Q_big.diagonal() = Q_diag.replicate(horizon_steps_, 1);
  pending_ |= DataChange::Weights;
}

void MPC::setStateWeights(const Ref<const VectorXd>& Q_diag,
                          const Ref<const VectorXd>& Qf_diag)
{
  validateSize(Q_diag.size(), state_dim_, "setStateWeights", "Q_diag");
  validateSize(Qf_diag.size(), state_dim_, "setStateWeights", "Qf_diag");
  if (Q_diag.minCoeff() < 0.0 || Qf_diag.minCoeff() < 0.0)
    throw std::invalid_argument(
        "[MPC::setStateWeights] State weights must be non-negative.");
  data_.Q_big.diagonal().head(state_dim_ * (horizon_steps_ - 1)) =
      Q_diag.replicate(horizon_steps_ - 1, 1);
  data_.Q_big.diagonal().tail(state_dim_) = Qf_diag;
  pending_ |= DataChange::Weights;
}

void MPC::setInputWeights(const Ref<const VectorXd>& R_diag)
{
  if (!opts_.use_input_cost)
    throw std::logic_error("[MPC::setInputWeights] Input cost is not enabled.");
  validateSize(R_diag.size(), input_dim_, "setInputWeights", "R_diag");
  if (R_diag.minCoeff() < 0.0)
    throw std::invalid_argument(
        "[MPC::setInputWeights] Input weights must be non-negative.");
  data_.R_big.diagonal() = R_diag.replicate(num_ctrl_pts_, 1);
  pending_ |= DataChange::Weights;
}

void MPC::setInputRateWeights(const Ref<const VectorXd>& R_rate_diag)
{
  if (!opts_.use_input_rate_cost)
    throw std::logic_error(
        "[MPC::setInputRateWeights] Input rate cost is not enabled.");
  validateSize(R_rate_diag.size(), input_dim_, "setInputRateWeights",
               "R_rate_diag");
  if (R_rate_diag.minCoeff() < 0.0)
    throw std::invalid_argument("[MPC::setInputRateWeights] Input rate "
                                "weights must be non-negative.");
  data_.R_rate = R_rate_diag;
  pending_ |= DataChange::Weights;
}

void MPC::setReferenceState(const Ref<const VectorXd>& x_step)
{
  validateSize(x_step.size(), state_dim_, "setReferenceState", "x_step");
  data_.x_ref = x_step.replicate(horizon_steps_, 1);
  pending_ |= DataChange::References;
}

void MPC::setReferenceStateTrajectory(const Ref<const VectorXd>& x_traj)
{
  validateSize(x_traj.size(), x_traj_dim_, "setReferenceStateTrajectory",
               "x_traj");
  data_.x_ref = x_traj;
  pending_ |= DataChange::References;
}

void MPC::setReferenceInput(const Ref<const VectorXd>& u_step)
{
  if (!opts_.use_input_cost)
    throw std::logic_error(
        "[MPC::setReferenceInput] Input cost is not enabled.");
  if (input_param_.parameterization().type != Parameterization::Type::BSpline)
    throw std::logic_error(
        "[MPC::setReferenceInput] A custom basis has no control points "
        "that represent a constant input; the input cost weights the basis "
        "coefficients, whose reference defaults to zero. Use "
        "setReferenceInputControlPoints() to set coefficient references.");
  validateSize(u_step.size(), input_dim_, "setReferenceInput", "u_step");
  data_.ctrls_ref = u_step.replicate(num_ctrl_pts_, 1);
  pending_ |= DataChange::References;
}

void MPC::setReferenceInputControlPoints(
    const Ref<const VectorXd>& control_points)
{
  if (!opts_.use_input_cost)
    throw std::logic_error("[MPC::setReferenceInputControlPoints] "
                           "Input cost is not enabled.");
  validateSize(control_points.size(), ctrls_dim_,
               "setReferenceInputControlPoints", "control_points");
  data_.ctrls_ref = control_points;
  pending_ |= DataChange::References;
}

void MPC::setInputLimits(const Ref<const VectorXd>& u_min,
                         const Ref<const VectorXd>& u_max)
{
  validateSize(u_min.size(), input_dim_, "setInputLimits", "u_min");
  validateSize(u_max.size(), input_dim_, "setInputLimits", "u_max");
  if ((u_max - u_min).minCoeff() < 0.0)
    throw std::invalid_argument(
        "[MPC::setInputLimits] u_min cannot be greater than u_max.");
  data_.u_min = u_min;
  data_.u_max = u_max;
  u_lims_set_ = true;
  pending_ |= DataChange::Limits;
}

void MPC::setStateLimits(const Ref<const VectorXd>& x_min,
                         const Ref<const VectorXd>& x_max)
{
  if (!opts_.saturate_states)
    throw std::logic_error(
        "[MPC::setStateLimits] State saturation is not enabled.");
  // one entry per bounded state
  const Index num_bounded{static_cast<Index>(bounded_states_.size())};
  validateSize(x_min.size(), num_bounded, "setStateLimits", "x_min");
  validateSize(x_max.size(), num_bounded, "setStateLimits", "x_max");
  if ((x_max - x_min).minCoeff() < 0.0)
    throw std::invalid_argument(
        "[MPC::setStateLimits] x_min cannot be greater than x_max.");
  data_.x_min = x_min;
  data_.x_max = x_max;
  x_lims_set_ = true;
  pending_ |= DataChange::Limits;
}

void MPC::setStateBoundPenalties(const Ref<const VectorXd>& l1,
                                 const Ref<const VectorXd>& l2)
{
  if (!opts_.soft_state_bounds)
    throw std::logic_error("[MPC::setStateBoundPenalties] Soft state "
                           "bounds are not enabled.");
  const Index num_bounded{static_cast<Index>(bounded_states_.size())};
  validateSize(l1.size(), num_bounded, "setStateBoundPenalties", "l1");
  validateSize(l2.size(), num_bounded, "setStateBoundPenalties", "l2");
  if (l1.minCoeff() < 0.0 || l2.minCoeff() < 0.0)
    throw std::invalid_argument("[MPC::setStateBoundPenalties] Penalties "
                                "must be non-negative.");
  data_.x_soft_l1 = l1;
  data_.x_soft_l2 = l2;
  x_penalties_set_ = true;
  pending_ |= DataChange::Weights;
}

void MPC::setGeneralConstraints(const Ref<const MatrixXd>& Hx,
                                const Ref<const MatrixXd>& Hu,
                                const Ref<const VectorXd>& h_min,
                                const Ref<const VectorXd>& h_max)
{
  if (opts_.num_general_constraints == 0)
    throw std::logic_error("[MPC::setGeneralConstraints] General "
                           "constraints are not enabled.");
  const Index p{opts_.num_general_constraints};
  validateShape(Hx, p, state_dim_, "setGeneralConstraints", "Hx");
  validateShape(Hu, p, input_dim_, "setGeneralConstraints", "Hu");
  validateSize(h_min.size(), p, "setGeneralConstraints", "h_min");
  validateSize(h_max.size(), p, "setGeneralConstraints", "h_max");
  if ((h_min.array() > h_max.array()).any())
    throw std::invalid_argument("[MPC::setGeneralConstraints] h_min "
                                "cannot be greater than h_max.");
  // new matrices refactor, so only mark them when they changed
  if (data_.Hx != Hx || data_.Hu != Hu) {
    data_.Hx = Hx;
    data_.Hu = Hu;
    pending_ |= DataChange::ConstraintMatrices;
  }
  data_.h_min = h_min;
  data_.h_max = h_max;
  general_set_ = true;
  pending_ |= DataChange::Limits;
}

void MPC::setTerminalConstraints(const Ref<const MatrixXd>& Hf,
                                 const Ref<const VectorXd>& hf_min,
                                 const Ref<const VectorXd>& hf_max)
{
  if (opts_.num_terminal_constraints == 0)
    throw std::logic_error("[MPC::setTerminalConstraints] Terminal "
                           "constraints are not enabled.");
  const Index p{opts_.num_terminal_constraints};
  validateShape(Hf, p, state_dim_, "setTerminalConstraints", "Hf");
  validateSize(hf_min.size(), p, "setTerminalConstraints", "hf_min");
  validateSize(hf_max.size(), p, "setTerminalConstraints", "hf_max");
  if ((hf_min.array() > hf_max.array()).any())
    throw std::invalid_argument("[MPC::setTerminalConstraints] hf_min "
                                "cannot be greater than hf_max.");
  if (data_.Hf != Hf) {
    data_.Hf = Hf;
    pending_ |= DataChange::ConstraintMatrices;
  }
  data_.hf_min = hf_min;
  data_.hf_max = hf_max;
  terminal_set_ = true;
  pending_ |= DataChange::Limits;
}

void MPC::setSlewRate(const Ref<const VectorXd>& control_point_slew)
{
  if (!opts_.slew_control_points)
    throw std::logic_error("[MPC::setSlewRate] Slew rate is not enabled.");
  validateSize(control_point_slew.size(), input_dim_, "setSlewRate",
               "control_point_slew");
  if (control_point_slew.minCoeff() < 0.0)
    throw std::invalid_argument(
        "[MPC::setSlewRate] Slew rate must be non-negative.");
  data_.ctrls_slew = control_point_slew;
  ctrls_slew_rate_set_ = true;
  pending_ |= DataChange::Limits;
}

void MPC::setSlewRateInitial(const Ref<const VectorXd>& u0_slew)
{
  if (!opts_.slew_initial_input)
    throw std::logic_error(
        "[MPC::setSlewRateInitial] Initial slew rate is not enabled.");
  validateSize(u0_slew.size(), input_dim_, "setSlewRateInitial", "u0_slew");
  if (u0_slew.minCoeff() < 0.0)
    throw std::invalid_argument(
        "[MPC::setSlewRateInitial] Slew rate must be non-negative.");
  data_.u0_slew = u0_slew;
  slew0_rate_set_ = true;
  pending_ |= DataChange::Limits;
}

void MPC::setPreviousInput(const Ref<const VectorXd>& u_prev)
{
  if (!usesPreviousInput())
    throw std::logic_error("[MPC::setPreviousInput] The initial slew rate "
                           "and the input rate cost are not enabled.");
  validateSize(u_prev.size(), input_dim_, "setPreviousInput", "u_prev");
  data_.u_prev = u_prev;
  pending_ |= DataChange::PreviousInput;
}

VectorXd MPC::getStateWeights() const
{
  return data_.Q_big.diagonal().head(state_dim_);
}

VectorXd MPC::getTerminalStateWeights() const
{
  return data_.Q_big.diagonal().tail(state_dim_);
}

VectorXd MPC::getInputWeights() const
{
  if (!opts_.use_input_cost)
    throw std::logic_error("[MPC::getInputWeights] Input cost is not enabled.");
  return data_.R_big.diagonal().head(input_dim_);
}

const VectorXd& MPC::getInputRateWeights() const
{
  if (!opts_.use_input_rate_cost)
    throw std::logic_error(
        "[MPC::getInputRateWeights] Input rate cost is not enabled.");
  return data_.R_rate;
}

const VectorXd& MPC::getStateMin() const
{
  if (!opts_.saturate_states)
    throw std::logic_error(
        "[MPC::getStateMin] State saturation is not enabled.");
  return data_.x_min;
}

const VectorXd& MPC::getStateMax() const
{
  if (!opts_.saturate_states)
    throw std::logic_error(
        "[MPC::getStateMax] State saturation is not enabled.");
  return data_.x_max;
}

const VectorXd& MPC::getStateBoundL1Penalties() const
{
  if (!opts_.soft_state_bounds)
    throw std::logic_error("[MPC::getStateBoundL1Penalties] Soft state "
                           "bounds are not enabled.");
  return data_.x_soft_l1;
}

const VectorXd& MPC::getStateBoundL2Penalties() const
{
  if (!opts_.soft_state_bounds)
    throw std::logic_error("[MPC::getStateBoundL2Penalties] Soft state "
                           "bounds are not enabled.");
  return data_.x_soft_l2;
}

const MatrixXd& MPC::getGeneralConstraintStateMatrix() const
{
  if (opts_.num_general_constraints == 0)
    throw std::logic_error("[MPC::getGeneralConstraintStateMatrix] "
                           "General constraints are not enabled.");
  return data_.Hx;
}

const MatrixXd& MPC::getGeneralConstraintInputMatrix() const
{
  if (opts_.num_general_constraints == 0)
    throw std::logic_error("[MPC::getGeneralConstraintInputMatrix] "
                           "General constraints are not enabled.");
  return data_.Hu;
}

const VectorXd& MPC::getGeneralConstraintMin() const
{
  if (opts_.num_general_constraints == 0)
    throw std::logic_error("[MPC::getGeneralConstraintMin] General "
                           "constraints are not enabled.");
  return data_.h_min;
}

const VectorXd& MPC::getGeneralConstraintMax() const
{
  if (opts_.num_general_constraints == 0)
    throw std::logic_error("[MPC::getGeneralConstraintMax] General "
                           "constraints are not enabled.");
  return data_.h_max;
}

const MatrixXd& MPC::getTerminalConstraintMatrix() const
{
  if (opts_.num_terminal_constraints == 0)
    throw std::logic_error("[MPC::getTerminalConstraintMatrix] Terminal "
                           "constraints are not enabled.");
  return data_.Hf;
}

const VectorXd& MPC::getTerminalConstraintMin() const
{
  if (opts_.num_terminal_constraints == 0)
    throw std::logic_error("[MPC::getTerminalConstraintMin] Terminal "
                           "constraints are not enabled.");
  return data_.hf_min;
}

const VectorXd& MPC::getTerminalConstraintMax() const
{
  if (opts_.num_terminal_constraints == 0)
    throw std::logic_error("[MPC::getTerminalConstraintMax] Terminal "
                           "constraints are not enabled.");
  return data_.hf_max;
}

const VectorXd& MPC::getSlewRate() const
{
  if (!opts_.slew_control_points)
    throw std::logic_error("[MPC::getSlewRate] Slew rate is not enabled.");
  return data_.ctrls_slew;
}

const VectorXd& MPC::getSlewRateInitial() const
{
  if (!opts_.slew_initial_input)
    throw std::logic_error(
        "[MPC::getSlewRateInitial] Initial slew rate is not enabled.");
  return data_.u0_slew;
}

const VectorXd& MPC::getPreviousInput() const
{
  if (!usesPreviousInput())
    throw std::logic_error("[MPC::getPreviousInput] The initial slew rate "
                           "and the input rate cost are not enabled.");
  return data_.u_prev;
}

const VectorXd& MPC::getReferenceInputControlPoints() const
{
  if (!opts_.use_input_cost)
    throw std::logic_error("[MPC::getReferenceInputControlPoints] "
                           "Input cost is not enabled.");
  return data_.ctrls_ref;
}

std::ostream& operator<<(std::ostream& os, const Formulation formulation)
{
  os << (formulation == Formulation::Sparse ? "Sparse" : "Condensed");
  return os;
}

std::ostream& print(std::ostream& os, const MPC& mpc, bool capitalize_bools)
{
  const Parameterization& param{mpc.getParameterization()};

  os << mpc.getClassName() << ":\n  formulation = " << mpc.getFormulation()
     << "\n  state_dim = " << mpc.getStateDim()
     << "\n  input_dim = " << mpc.getInputDim()
     << "\n  parameterization = " << param << "\n  options = ";
  print(os, mpc.getOptions(), capitalize_bools);
  os << "\n  solver_initialized = ";
  if (capitalize_bools)
    os << (mpc.isSolverInitialized() ? "True" : "False");
  else
    os << (mpc.isSolverInitialized() ? "true" : "false");

  const IOFormat fmt{
      StreamPrecision, DontAlignCols, ", ", ", ", "", "", "[", "]"};
  os << "\n  Q = " << mpc.getStateWeights().format(fmt)
     << "\n  Qf = " << mpc.getTerminalStateWeights().format(fmt);
  if (mpc.getOptions().use_input_cost)
    os << "\n  R = " << mpc.getInputWeights().format(fmt);
  if (mpc.getOptions().use_input_rate_cost)
    os << "\n  R_rate = " << mpc.getInputRateWeights().format(fmt);
  os << "\n  u_min = " << mpc.getInputMin().format(fmt)
     << "\n  u_max = " << mpc.getInputMax().format(fmt);
  if (mpc.getOptions().saturate_states)
    os << "\n  x_min = " << mpc.getStateMin().format(fmt)
       << "\n  x_max = " << mpc.getStateMax().format(fmt);
  if (mpc.getOptions().soft_state_bounds)
    os << "\n  x_soft_l1 = " << mpc.getStateBoundL1Penalties().format(fmt)
       << "\n  x_soft_l2 = " << mpc.getStateBoundL2Penalties().format(fmt);
  if (mpc.getOptions().slew_initial_input)
    os << "\n  u0_slew = " << mpc.getSlewRateInitial().format(fmt);
  if (mpc.getOptions().slew_control_points)
    os << "\n  control_point_slew = " << mpc.getSlewRate().format(fmt);
  const IOFormat mat_fmt{
      StreamPrecision, DontAlignCols, ", ", ", ", "[", "]", "[", "]"};
  if (mpc.getOptions().num_general_constraints > 0)
    os << "\n  Hx = " << mpc.getGeneralConstraintStateMatrix().format(mat_fmt)
       << "\n  Hu = " << mpc.getGeneralConstraintInputMatrix().format(mat_fmt)
       << "\n  h_min = " << mpc.getGeneralConstraintMin().format(fmt)
       << "\n  h_max = " << mpc.getGeneralConstraintMax().format(fmt);
  if (mpc.getOptions().num_terminal_constraints > 0)
    os << "\n  Hf = " << mpc.getTerminalConstraintMatrix().format(mat_fmt)
       << "\n  hf_min = " << mpc.getTerminalConstraintMin().format(fmt)
       << "\n  hf_max = " << mpc.getTerminalConstraintMax().format(fmt);
  return os;
}

std::ostream&
printInline(std::ostream& os, const MPC& mpc, bool capitalize_bools)
{
  const Parameterization& param{mpc.getParameterization()};

  os << mpc.getClassName() << "(formulation=" << mpc.getFormulation()
     << ", state_dim=" << mpc.getStateDim()
     << ", input_dim=" << mpc.getInputDim() << ", parameterization=" << param
     << ", options=";
  print(os, mpc.getOptions(), capitalize_bools);
  os << ", solver_initialized=";
  if (capitalize_bools)
    os << (mpc.isSolverInitialized() ? "True" : "False");
  else
    os << (mpc.isSolverInitialized() ? "true" : "false");

  const IOFormat fmt{
      StreamPrecision, DontAlignCols, ", ", ", ", "", "", "[", "]"};
  os << ", Q=" << mpc.getStateWeights().format(fmt)
     << ", Qf=" << mpc.getTerminalStateWeights().format(fmt);
  if (mpc.getOptions().use_input_cost)
    os << ", R=" << mpc.getInputWeights().format(fmt);
  if (mpc.getOptions().use_input_rate_cost)
    os << ", R_rate=" << mpc.getInputRateWeights().format(fmt);
  os << ", u_min=" << mpc.getInputMin().format(fmt)
     << ", u_max=" << mpc.getInputMax().format(fmt);
  if (mpc.getOptions().saturate_states)
    os << ", x_min=" << mpc.getStateMin().format(fmt)
       << ", x_max=" << mpc.getStateMax().format(fmt);
  if (mpc.getOptions().soft_state_bounds)
    os << ", x_soft_l1=" << mpc.getStateBoundL1Penalties().format(fmt)
       << ", x_soft_l2=" << mpc.getStateBoundL2Penalties().format(fmt);
  if (mpc.getOptions().slew_initial_input)
    os << ", u0_slew=" << mpc.getSlewRateInitial().format(fmt);
  if (mpc.getOptions().slew_control_points)
    os << ", control_point_slew=" << mpc.getSlewRate().format(fmt);
  const IOFormat mat_fmt{
      StreamPrecision, DontAlignCols, ", ", ", ", "[", "]", "[", "]"};
  if (mpc.getOptions().num_general_constraints > 0)
    os << ", Hx=" << mpc.getGeneralConstraintStateMatrix().format(mat_fmt)
       << ", Hu=" << mpc.getGeneralConstraintInputMatrix().format(mat_fmt)
       << ", h_min=" << mpc.getGeneralConstraintMin().format(fmt)
       << ", h_max=" << mpc.getGeneralConstraintMax().format(fmt);
  if (mpc.getOptions().num_terminal_constraints > 0)
    os << ", Hf=" << mpc.getTerminalConstraintMatrix().format(mat_fmt)
       << ", hf_min=" << mpc.getTerminalConstraintMin().format(fmt)
       << ", hf_max=" << mpc.getTerminalConstraintMax().format(fmt);
  os << ')';
  return os;
}

std::ostream& operator<<(std::ostream& os, const MPC& mpc)
{
  return print(os, mpc);
}

} // namespace affine_mpc
