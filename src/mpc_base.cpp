#include "affine_mpc/mpc_base.hpp"

#include <Eigen/Core>
#include <algorithm>
#include <cassert>
#include <chrono>
#include <cmath>
#include <osqp.h>
#include <stdexcept>
#include <string>
#include <vector>

#include "affine_mpc/parameterization.hpp"
#include "affine_mpc/solve_status.hpp"
#include "affine_mpc/vector_parameterization.hpp"
#include "eigen_compat.hpp" // remove this once Eigen 3.5 is required

using namespace Eigen;
// revert back to this once Eigen 3.5 is required
// namespace ph = Eigen::placeholders;

namespace affine_mpc {

namespace { // utilities for this file

constexpr int validateStateDim(int state_dim)
{
  if (state_dim <= 0)
    throw std::invalid_argument(
        "[MPCBase::validateStateDim] state_dim must be positive.");
  return state_dim;
}

constexpr int validateInputDim(int input_dim)
{
  if (input_dim <= 0)
    throw std::invalid_argument(
        "[MPCBase::validateInputDim] input_dim must be positive.");
  return input_dim;
}

// Sample indices whose inputs are bounded when `saturate_input_trajectory` is
// enabled. An empty result means the control points are bounded instead.
std::vector<int> inputSatSamples(const VectorParameterization& input_param,
                                 const Options& opts)
{
  if (!opts.saturate_input_trajectory)
    return {};
  return input_param.inputSaturationSamples();
}

// Size checks for setters, which are not on the solve path. The message is
// only built when the check fails.
void validateSize(Index size,
                  Index expected,
                  const char* func,
                  const char* name)
{
  if (size != expected)
    throw std::invalid_argument(std::string{"[MPCBase::"} + func + "] " + name
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
        std::string{"[MPCBase::"} + func + "] " + name + " must be "
        + std::to_string(rows) + "x" + std::to_string(cols) + ", got "
        + std::to_string(mat.rows()) + "x" + std::to_string(mat.cols()) + ".");
}

} // namespace

MPCBase::MPCBase(int state_dim,
                 int input_dim,
                 const Parameterization& param,
                 const Options& opts,
                 int num_design_vars,
                 int num_custom_constraints) :
    state_dim_{validateStateDim(state_dim)},
    input_dim_{validateInputDim(input_dim)},
    horizon_steps_{param.horizon_steps},
    num_ctrl_pts_{param.num_control_points},
    x_traj_dim_{state_dim * param.horizon_steps},
    u_traj_dim_{input_dim * param.horizon_steps},
    ctrls_dim_{input_dim * param.num_control_points},
    opts_{opts},
    input_param_{input_dim_, param},
    u_sat_samples_{inputSatSamples(input_param_, opts)},
    num_u_sat_cons_{u_sat_samples_.empty()
                        ? param.num_control_points
                        : static_cast<int>(u_sat_samples_.size())},
    u_sat_dim_{input_dim_ * num_u_sat_cons_},
    slew_dim_{(ctrls_dim_ - input_dim_) * opts.slew_control_points},
    x_sat_dim_{x_traj_dim_ * opts.saturate_states},
    u_sat_idx_{num_custom_constraints},
    slew0_idx_{u_sat_idx_ + u_sat_dim_},
    slew_idx_{slew0_idx_ + input_dim * opts.slew_initial_input},
    x_sat_idx_{slew_idx_ + slew_dim_},
    model_set_{false},
    model_from_continuous_{false},
    u_lims_set_{false},
    x_lims_set_{false},
    slew0_rate_set_{false},
    ctrls_slew_rate_set_{false},
    solver_initialized_{false},
    mpc_solve_time_{0.0},
    Ad_{state_dim, state_dim},
    Bd_{state_dim, input_dim},
    wd_{state_dim},
    discretizer_{state_dim_, input_dim_},
    Q_big_{state_dim * param.horizon_steps},
    x_ref_{state_dim * param.horizon_steps},
    u_min_{input_dim},
    u_max_{input_dim},
    solution_map_{nullptr, 0},
    solver_{nullptr}
{
  // allocate QP memory
  const int slew0_dim{input_dim * opts.slew_initial_input};
  const int num_constraints{num_custom_constraints + u_sat_dim_ + slew0_dim
                            + slew_dim_ + x_sat_dim_};
  solver_ = std::make_unique<OSQPSolver>(num_design_vars, num_constraints);
  P_.resize(num_design_vars, num_design_vars);
  A_.resize(num_constraints, num_design_vars);
  q_.resize(num_design_vars);
  l_.resize(num_constraints);
  u_.resize(num_constraints);

  // set defaults
  Q_big_.setIdentity();
  u_min_.setConstant(-std::numeric_limits<double>::infinity());
  u_max_.setConstant(std::numeric_limits<double>::infinity());

  // initialize common constraint matrix blocks
  A_.setZero();

  if (!u_sat_samples_.empty()) {
    // saturate sampled inputs (each a weighted sum of control points)
    int row{u_sat_idx_};
    for (const int k : u_sat_samples_) {
      for (int i{0}; i < param.band_width; ++i) {
        const int col{
            input_param_.coefficientIndex(0, param.band_start(k) + i)};
        A_.block(row, col, input_dim_, input_dim_)
            .diagonal()
            .setConstant(param.band_weights(i, k));
      }
      row += input_dim_;
    }
  } else {
    // saturate control points directly (much fewer constraints)
    A_.middleRows(u_sat_idx_, u_sat_dim_).diagonal().setOnes();
  }

  // allocate memory needed based on options
  if (opts_.use_input_cost) {
    R_big_.setIdentity(ctrls_dim_);
    ctrls_ref_.setZero(ctrls_dim_);
  }
  if (opts.slew_initial_input) {
    // the first sampled input u_0
    for (int i{0}; i < param.band_width; ++i) {
      const int col{input_param_.coefficientIndex(0, param.band_start(0) + i)};
      A_.block(slew0_idx_, col, input_dim_, input_dim_)
          .diagonal()
          .setConstant(param.band_weights(i, 0));
    }
    u_prev_.setZero(input_dim_);
    u0_slew_.resize(input_dim);
    u0_slew_.setConstant(std::numeric_limits<double>::infinity());
  }
  if (opts.slew_control_points) {
    const int slew_cp_dim{input_dim * (param.num_control_points - 1)};
    A_.middleRows(slew_idx_, slew_cp_dim).diagonal().setConstant(-1.0);
    A_.middleRows(slew_idx_, slew_cp_dim).diagonal(input_dim).setOnes();
    ctrls_slew_.resize(input_dim);
    ctrls_slew_.setConstant(std::numeric_limits<double>::infinity());
  }
  if (opts_.saturate_states) {
    x_min_.resize(state_dim);
    x_max_.resize(state_dim);
    x_min_.setConstant(-std::numeric_limits<double>::infinity());
    x_max_.setConstant(std::numeric_limits<double>::infinity());
  }
}

bool MPCBase::initializeSolver(const OSQPSettings& solver_settings)
{
  if (!model_set_)
    throw std::logic_error(
        "[MPCBase::initializeSolver] Model must be set before initializing "
        "solver.");
  if (!u_lims_set_)
    throw std::logic_error(
        "[MPCBase::initializeSolver] Input limits must be set before "
        "initializing solver.");
  if (opts_.slew_control_points && !ctrls_slew_rate_set_)
    throw std::logic_error(
        "[MPCBase::initializeSolver] Slew rate must be set before initializing "
        "solver (slew_control_points is enabled).");
  if (opts_.slew_initial_input && !slew0_rate_set_)
    throw std::logic_error(
        "[MPCBase::initializeSolver] Initial slew rate must be set before "
        "initializing solver (slew_initial_input is enabled).");
  if (opts_.saturate_states && !x_lims_set_)
    throw std::logic_error(
        "[MPCBase::initializeSolver] State limits must be set before "
        "initializing solver (saturate_states is enabled).");

  if (solver_initialized_)
    return true;

  // x0 only affects vector terms (q, l, u) so the values don't matter to set
  // initial sparsity structure. Setting to ones. The solver is not
  // initialized yet, so only the assembled QP matters here, not the result.
  VectorXd x_full{state_dim_};
  x_full.setOnes();
  (void)qpUpdateX0(x_full);

  solver_initialized_ =
      solver_->initialize(P_, A_, q_, l_, u_, solver_settings);
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

SolveStatus MPCBase::solve(const Ref<const VectorXd>& x0)
{
  validateSize(x0.size(), state_dim_, "solve", "x0");
  if (!solver_initialized_)
    return SolveStatus::NotInitialized;

  using Clock = std::chrono::steady_clock;
  const Clock::time_point start{Clock::now()};
  if (!qpUpdateX0(x0)) {
    mpc_solve_time_ =
        std::chrono::duration<double>(Clock::now() - start).count();
    return SolveStatus::UpdateFailed;
  }
  const SolveStatus status{solver_->solve()};

  // update u_prev after solve rather than before so user can manually
  // overwrite it between solves if desired. Other statuses leave an iterate
  // that is not a solution (e.g. an infeasibility certificate), which would
  // corrupt the initial slew constraint of every later solve.
  if (opts_.slew_initial_input
      && (status == SolveStatus::Success
          || status == SolveStatus::SolvedInaccurate)) {
    getNextInput(u_prev_);
    setPreviousInput(u_prev_);
  }
  mpc_solve_time_ = std::chrono::duration<double>(Clock::now() - start).count();
  return status;
}

bool MPCBase::isWithinSparsityPattern() const
{
  if (!solver_initialized_)
    return true;
  return solver_->isWithinSparsityPattern(P_, A_);
}

void MPCBase::getNextInput(Ref<VectorXd> u0) const
{
  assert(solver_initialized_);
  validateSize(u0.size(), input_dim_, "getNextInput", "u0");
  // Assumes that the control points are first elements of solution
  input_param_.evaluate(0, solution_map_, u0);
}

void MPCBase::getInputControlPoints(Ref<VectorXd> control_points) const
{
  assert(solver_initialized_);
  validateSize(control_points.size(), ctrls_dim_, "getInputControlPoints",
               "control_points");
  // Assumes that the control points are first elements of solution
  control_points = solution_map_.head(ctrls_dim_);
}

void MPCBase::getInputTrajectory(Ref<VectorXd> u_traj) const
{
  assert(solver_initialized_);
  validateSize(u_traj.size(), u_traj_dim_, "getInputTrajectory", "u_traj");
  // Assumes that the control points are first elements of solution
  input_param_.evaluateTrajectory(solution_map_, u_traj);
}

void MPCBase::getPredictedStateTrajectory(Ref<VectorXd> x_traj) const
{
  assert(solver_initialized_);
  validateSize(x_traj.size(), x_traj_dim_, "getPredictedStateTrajectory",
               "x_traj");
}

void MPCBase::propagateModel(const Ref<const VectorXd>& x0,
                             const Ref<const VectorXd>& u,
                             Ref<VectorXd> x_next) const
{
  if (!model_set_)
    throw std::logic_error(
        "[MPCBase::propagateModel] Model must be set before propagation");
  validateSize(x0.size(), state_dim_, "propagateModel", "x");
  validateSize(u.size(), input_dim_, "propagateModel", "u");
  validateSize(x_next.size(), state_dim_, "propagateModel", "x_next");
  // do not use noalias here since x_next could be an alias of x0
  x_next = Ad_ * x0 + Bd_ * u + wd_;
}

bool MPCBase::setModelDiscrete(const Ref<const MatrixXd>& Ad,
                               const Ref<const MatrixXd>& Bd,
                               const Ref<const VectorXd>& wd)
{
  validateShape(Ad, state_dim_, state_dim_, "setModelDiscrete", "Ad");
  validateShape(Bd, state_dim_, input_dim_, "setModelDiscrete", "Bd");
  validateSize(wd.size(), state_dim_, "setModelDiscrete", "wd");

  Ad_ = Ad;
  Bd_ = Bd;
  wd_ = wd;
  model_set_ = true;
  model_from_continuous_ = false;
  return qpUpdateModel();
}

bool MPCBase::setModelContinuous2Discrete(const Ref<const MatrixXd>& Ac,
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
  discretizer_.discretize(Ac, Bc, wc, dt, Ad_, Bd_, wd_, tol);
  model_set_ = true;
  model_from_continuous_ = true;
  return qpUpdateModel();
}

void MPCBase::setModelOffsetDiscrete(const Ref<const VectorXd>& wd)
{
  validateSize(wd.size(), state_dim_, "setModelOffsetDiscrete", "wd");
  if (!model_set_)
    throw std::logic_error("[MPCBase::setModelOffsetDiscrete] Model must be "
                           "set before its offset.");
  wd_ = wd;
  qpUpdateModelOffset();
}

void MPCBase::setModelOffsetContinuous2Discrete(const Ref<const VectorXd>& wc)
{
  validateSize(wc.size(), state_dim_, "setModelOffsetContinuous2Discrete",
               "wc");
  if (!model_from_continuous_)
    throw std::logic_error(
        "[MPCBase::setModelOffsetContinuous2Discrete] The model must be last "
        "set with setModelContinuous2Discrete(), whose discretization is "
        "reused.");
  discretizer_.discretizeOffset(wc, wd_);
  qpUpdateModelOffset();
}

void MPCBase::setWeights(const Ref<const VectorXd>& Q_diag,
                         const Ref<const VectorXd>& R_diag)
{
  setStateWeights(Q_diag);
  setInputWeights(R_diag);
}

void MPCBase::setWeights(const Ref<const VectorXd>& Q_diag,
                         const Ref<const VectorXd>& Qf_diag,
                         const Ref<const VectorXd>& R_diag)
{
  setStateWeights(Q_diag, Qf_diag);
  setInputWeights(R_diag);
}

void MPCBase::setStateWeights(const Ref<const VectorXd>& Q_diag)
{
  validateSize(Q_diag.size(), state_dim_, "setStateWeights", "Q_diag");
  if (Q_diag.minCoeff() < 0.0)
    throw std::invalid_argument(
        "[MPCBase::setStateWeights] State weights must be non-negative.");
  Q_big_.diagonal() = Q_diag.replicate(horizon_steps_, 1);
  qpUpdateWeights();
}

void MPCBase::setStateWeights(const Ref<const VectorXd>& Q_diag,
                              const Ref<const VectorXd>& Qf_diag)
{
  validateSize(Q_diag.size(), state_dim_, "setStateWeights", "Q_diag");
  validateSize(Qf_diag.size(), state_dim_, "setStateWeights", "Qf_diag");
  if (Q_diag.minCoeff() < 0.0 || Qf_diag.minCoeff() < 0.0)
    throw std::invalid_argument(
        "[MPCBase::setStateWeights] State weights must be non-negative.");
  Q_big_.diagonal().head(state_dim_ * (horizon_steps_ - 1)) =
      Q_diag.replicate(horizon_steps_ - 1, 1);
  Q_big_.diagonal().tail(state_dim_) = Qf_diag;
  qpUpdateWeights();
}

void MPCBase::setInputWeights(const Ref<const VectorXd>& R_diag)
{
  if (!opts_.use_input_cost)
    throw std::logic_error(
        "[MPCBase::setInputWeights] Input cost is not enabled.");
  validateSize(R_diag.size(), input_dim_, "setInputWeights", "R_diag");
  if (R_diag.minCoeff() < 0.0)
    throw std::invalid_argument(
        "[MPCBase::setInputWeights] Input weights must be non-negative.");
  R_big_.diagonal() = R_diag.replicate(num_ctrl_pts_, 1);
  qpUpdateWeights();
}

bool MPCBase::setReferenceState(const Ref<const VectorXd>& x_step)
{
  validateSize(x_step.size(), state_dim_, "setReferenceState", "x_step");
  x_ref_ = x_step.replicate(horizon_steps_, 1);
  return qpUpdateReferences();
}

bool MPCBase::setReferenceStateTrajectory(const Ref<const VectorXd>& x_traj)
{
  validateSize(x_traj.size(), x_traj_dim_, "setReferenceStateTrajectory",
               "x_traj");
  x_ref_ = x_traj;
  return qpUpdateReferences();
}

bool MPCBase::setReferenceInput(const Ref<const VectorXd>& u_step)
{
  if (!opts_.use_input_cost)
    throw std::logic_error(
        "[MPCBase::setReferenceInput] Input cost is not enabled.");
  validateSize(u_step.size(), input_dim_, "setReferenceInput", "u_step");
  ctrls_ref_ = u_step.replicate(num_ctrl_pts_, 1);
  return qpUpdateReferences();
}

bool MPCBase::setReferenceInputControlPoints(
    const Ref<const VectorXd>& control_points)
{
  if (!opts_.use_input_cost)
    throw std::logic_error("[MPCBase::setReferenceInputControlPoints] "
                           "Input cost is not enabled.");
  validateSize(control_points.size(), ctrls_dim_,
               "setReferenceInputControlPoints", "control_points");
  ctrls_ref_ = control_points;
  return qpUpdateReferences();
}

bool MPCBase::setInputLimits(const Ref<const VectorXd>& u_min,
                             const Ref<const VectorXd>& u_max)
{
  validateSize(u_min.size(), input_dim_, "setInputLimits", "u_min");
  validateSize(u_max.size(), input_dim_, "setInputLimits", "u_max");
  if ((u_max - u_min).minCoeff() < 0.0)
    throw std::invalid_argument(
        "[MPCBase::setInputLimits] u_min cannot be greater than u_max.");
  u_min_ = u_min;
  u_max_ = u_max;
  u_lims_set_ = true;

  l_.segment(u_sat_idx_, u_sat_dim_) = u_min_.replicate(num_u_sat_cons_, 1);
  u_.segment(u_sat_idx_, u_sat_dim_) = u_max_.replicate(num_u_sat_cons_, 1);
  return qpUpdateInputLimits();
}

bool MPCBase::setStateLimits(const Ref<const VectorXd>& x_min,
                             const Ref<const VectorXd>& x_max)
{
  if (!opts_.saturate_states)
    throw std::logic_error(
        "[MPCBase::setStateLimits] State saturation is not enabled.");
  validateSize(x_min.size(), state_dim_, "setStateLimits", "x_min");
  validateSize(x_max.size(), state_dim_, "setStateLimits", "x_max");
  if ((x_max - x_min).minCoeff() < 0.0)
    throw std::invalid_argument(
        "[MPCBase::setStateLimits] x_min cannot be greater than x_max.");
  x_min_ = x_min;
  x_max_ = x_max;
  x_lims_set_ = true;

  l_.tail(x_sat_dim_) = x_min_.replicate(horizon_steps_, 1);
  u_.tail(x_sat_dim_) = x_max_.replicate(horizon_steps_, 1);
  return qpUpdateStateLimits();
}

bool MPCBase::setSlewRate(const Ref<const VectorXd>& control_point_slew)
{
  if (!opts_.slew_control_points)
    throw std::logic_error("[MPCBase::setSlewRate] Slew rate is not enabled.");
  validateSize(control_point_slew.size(), input_dim_, "setSlewRate",
               "control_point_slew");
  if (control_point_slew.minCoeff() < 0.0)
    throw std::invalid_argument(
        "[MPCBase::setSlewRate] Slew rate must be non-negative.");
  ctrls_slew_ = control_point_slew;
  ctrls_slew_rate_set_ = true;

  u_.segment(slew_idx_, slew_dim_) =
      ctrls_slew_.replicate(num_ctrl_pts_ - 1, 1);
  l_.segment(slew_idx_, slew_dim_) = -u_.segment(slew_idx_, slew_dim_);
  return qpUpdateSlewRate();
}

bool MPCBase::setSlewRateInitial(const Ref<const VectorXd>& u0_slew)
{
  if (!opts_.slew_initial_input)
    throw std::logic_error(
        "[MPCBase::setSlewRateInitial] Initial slew rate is not enabled.");
  validateSize(u0_slew.size(), input_dim_, "setSlewRateInitial", "u0_slew");
  if (u0_slew.minCoeff() < 0.0)
    throw std::invalid_argument(
        "[MPCBase::setSlewRateInitial] Slew rate must be non-negative.");
  u0_slew_ = u0_slew;
  slew0_rate_set_ = true;

  l_.segment(slew0_idx_, input_dim_) = u_prev_ - u0_slew_;
  u_.segment(slew0_idx_, input_dim_) = u_prev_ + u0_slew_;
  return qpUpdateSlewRate();
}

bool MPCBase::setPreviousInput(const Ref<const VectorXd>& u_prev)
{
  if (!opts_.slew_initial_input)
    throw std::logic_error(
        "[MPCBase::setPreviousInput] Initial slew rate is not enabled.");
  validateSize(u_prev.size(), input_dim_, "setPreviousInput", "u_prev");
  u_prev_ = u_prev;

  l_.segment(slew0_idx_, input_dim_) = u_prev_ - u0_slew_;
  u_.segment(slew0_idx_, input_dim_) = u_prev_ + u0_slew_;
  return qpUpdateSlewRate();
}

VectorXd MPCBase::getStateWeights() const
{
  return Q_big_.diagonal().head(state_dim_);
}

VectorXd MPCBase::getTerminalStateWeights() const
{
  return Q_big_.diagonal().tail(state_dim_);
}

VectorXd MPCBase::getInputWeights() const
{
  if (!opts_.use_input_cost)
    throw std::logic_error(
        "[MPCBase::getInputWeights] Input cost is not enabled.");
  return R_big_.diagonal().head(input_dim_);
}

const VectorXd& MPCBase::getStateMin() const
{
  if (!opts_.saturate_states)
    throw std::logic_error(
        "[MPCBase::getStateMin] State saturation is not enabled.");
  return x_min_;
}

const VectorXd& MPCBase::getStateMax() const
{
  if (!opts_.saturate_states)
    throw std::logic_error(
        "[MPCBase::getStateMax] State saturation is not enabled.");
  return x_max_;
}

const VectorXd& MPCBase::getSlewRate() const
{
  if (!opts_.slew_control_points)
    throw std::logic_error("[MPCBase::getSlewRate] Slew rate is not enabled.");
  return ctrls_slew_;
}

const VectorXd& MPCBase::getSlewRateInitial() const
{
  if (!opts_.slew_initial_input)
    throw std::logic_error(
        "[MPCBase::getSlewRateInitial] Initial slew rate is not enabled.");
  return u0_slew_;
}

const VectorXd& MPCBase::getPreviousInput() const
{
  if (!opts_.slew_initial_input)
    throw std::logic_error(
        "[MPCBase::getPreviousInput] Initial slew rate is not enabled.");
  return u_prev_;
}

const VectorXd& MPCBase::getReferenceInputControlPoints() const
{
  if (!opts_.use_input_cost)
    throw std::logic_error("[MPCBase::getReferenceInputControlPoints] "
                           "Input cost is not enabled.");
  return ctrls_ref_;
}

std::ostream& print(std::ostream& os, const MPCBase& mpc, bool capitalize_bools)
{
  const Parameterization& param{mpc.getParameterization()};

  os << mpc.getClassName() << ":\n  state_dim = " << mpc.getStateDim()
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
  os << "\n  u_min = " << mpc.getInputMin().format(fmt)
     << "\n  u_max = " << mpc.getInputMax().format(fmt);
  if (mpc.getOptions().saturate_states)
    os << "\n  x_min = " << mpc.getStateMin().format(fmt)
       << "\n  x_max = " << mpc.getStateMax().format(fmt);
  if (mpc.getOptions().slew_initial_input)
    os << "\n  u0_slew = " << mpc.getSlewRateInitial().format(fmt);
  if (mpc.getOptions().slew_control_points)
    os << "\n  control_point_slew = " << mpc.getSlewRate().format(fmt);
  return os;
}

std::ostream&
printInline(std::ostream& os, const MPCBase& mpc, bool capitalize_bools)
{
  const Parameterization& param{mpc.getParameterization()};

  os << mpc.getClassName() << "(state_dim=" << mpc.getStateDim()
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
  os << ", u_min=" << mpc.getInputMin().format(fmt)
     << ", u_max=" << mpc.getInputMax().format(fmt);
  if (mpc.getOptions().saturate_states)
    os << ", x_min=" << mpc.getStateMin().format(fmt)
       << ", x_max=" << mpc.getStateMax().format(fmt);
  if (mpc.getOptions().slew_initial_input)
    os << ", u0_slew=" << mpc.getSlewRateInitial().format(fmt);
  if (mpc.getOptions().slew_control_points)
    os << ", control_point_slew=" << mpc.getSlewRate().format(fmt);
  os << ')';
  return os;
}

std::ostream& operator<<(std::ostream& os, const MPCBase& mpc)
{
  return print(os, mpc);
}

} // namespace affine_mpc
