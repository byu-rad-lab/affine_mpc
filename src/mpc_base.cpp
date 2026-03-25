#include "affine_mpc/mpc_base.hpp"

#include <cassert>   // for assert
#include <stdexcept> // for exceptions

// #include <Eigen/Core>  // revert back to this once Eigen 3.5 is required
#include "affine_mpc/parameterization.hpp"
#include "affine_mpc/solve_status.hpp"
#include "eigen_compat.hpp"          // revmove this once Eigen 3.5 is required
#include <osqp.h>                    // for OSQPSettings
#include <unsupported/Eigen/Splines> // for B-spline support

using namespace Eigen;
// revert back to this once Eigen 3.5 is required
// namespace ph = Eigen::placeholders;

namespace affine_mpc {

constexpr int validateStateDim(const int state_dim)
{
  if (state_dim <= 0)
    throw std::invalid_argument(
        "[MPCBase::validateStateDim] state_dim must be positive.");
  return state_dim;
}

constexpr int validateInputDim(const int input_dim)
{
  if (input_dim <= 0)
    throw std::invalid_argument(
        "[MPCBase::validateInputDim] input_dim must be positive.");
  return input_dim;
}

constexpr bool satInputTraj(const Parameterization& param, const Options& opts)
{
  return opts.saturate_input_trajectory && param.degree > 1;
}

MPCBase::MPCBase(const int state_dim,
                 const int input_dim,
                 const Parameterization& param,
                 const Options& opts,
                 const int num_design_vars,
                 const int num_custom_constraints) :
    state_dim_{validateStateDim(state_dim)},
    input_dim_{validateInputDim(input_dim)},
    horizon_steps_{param.horizon_steps},
    num_ctrl_pts_{param.num_control_points},
    spline_degree_{param.degree},
    x_traj_dim_{state_dim * param.horizon_steps},
    u_traj_dim_{input_dim * param.horizon_steps},
    ctrls_dim_{input_dim * param.num_control_points},
    opts_{opts},
    num_u_sat_cons_{satInputTraj(param, opts) ? param.horizon_steps
                                              : param.num_control_points},
    u_sat_dim_{input_dim_ * num_u_sat_cons_},
    slew_dim_{(ctrls_dim_ - input_dim_) * opts.slew_control_points},
    x_sat_dim_{x_traj_dim_ * opts.saturate_states},
    u_sat_idx_{num_custom_constraints},
    slew0_idx_{u_sat_idx_ + u_sat_dim_},
    slew_idx_{slew0_idx_ + input_dim * opts.slew_initial_input},
    x_sat_idx_{slew_idx_ + slew_dim_},
    model_set_{false},
    u_lims_set_{false},
    slew_rate_set_{false},
    x_lims_set_{false},
    solver_initialized_{false},
    weights_changed_{false},
    solver_{nullptr},
    spline_segment_idxs_{param.horizon_steps},
    // spline_knots_{param.num_control_points + param.degree + 1},
    spline_knots_{param.knots},
    spline_weights_{param.degree + 1, param.horizon_steps},
    Ad_{state_dim, state_dim},
    Bd_{state_dim, input_dim},
    wd_{state_dim},
    Q_big_{state_dim * param.horizon_steps},
    x_ref_{state_dim * param.horizon_steps},
    u_min_{input_dim},
    u_max_{input_dim},
    solution_map_{nullptr, 0}
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

  calcSplineParams();

  // initiallize common constraint matrix blocks
  A_.setZero();

  if (satInputTraj(param, opts)) {
    const int num_weights{spline_degree_ + 1};
    for (int k{0}, row{0}; k < horizon_steps_; ++k, row += input_dim_)
      for (int i{0}, col{input_dim_ * spline_segment_idxs_(k)}; i < num_weights;
           ++i, col += input_dim_) {
        A_.block(row, col, input_dim_, input_dim_)
            .diagonal()
            .setConstant(spline_weights_(i, k));
      }
  } else {
    // saturate control points directly (much fewer constraints)
    A_.middleRows(u_sat_idx_, u_sat_dim_).diagonal().setOnes();
  }

  // allocate memory needed based on options
  if (opts_.use_input_cost) {
    R_big_.setIdentity(ctrls_dim_);
    u_ref_.setZero(ctrls_dim_);
  }
  if (opts.slew_initial_input) {
    for (int i{0}, col{0}; i < spline_degree_ + 1; ++i, col += input_dim_) {
      A_.block(slew0_idx_, col, input_dim_, input_dim_)
          .diagonal()
          .setConstant(spline_weights_(i, 0));
    }
    u_prev_.setZero(input_dim_);
    u0_slew_.resize(input_dim);
    u0_slew_.setConstant(std::numeric_limits<double>::infinity());
  }
  if (opts.slew_control_points) {
    const int slew_cp_dim{input_dim * (param.num_control_points - 1)};
    A_.middleRows(slew_idx_, slew_cp_dim).diagonal().setConstant(-1.0);
    A_.middleRows(slew_idx_, slew_cp_dim).diagonal(input_dim).setOnes();
    u_slew_.resize(input_dim);
    u_slew_.setConstant(std::numeric_limits<double>::infinity());
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
  if (opts_.slew_control_points && !slew_rate_set_)
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
  // initial sparsity structure. Setting to ones.
  VectorXd x_full{state_dim_};
  x_full.setOnes();
  qpUpdateX0(x_full);

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
  if (!solver_initialized_)
    return SolveStatus::NotInitialized;
  assert(x0.size() == state_dim_);

  qpUpdateX0(x0);
  const SolveStatus status{solver_->solve()};

  // update u_prev after solve rather than before so user can manually
  // overwrite it between solves if desired
  if (opts_.slew_initial_input) {
    u_prev_ = solution_map_.head(input_dim_);
    setPreviousInput(u_prev_);
  }
  return status;
}

void MPCBase::getNextInput(Ref<VectorXd> u0) const noexcept
{
  assert(u0.size() == input_dim_);
  const Map<const MatrixXd> ctrls{solution_map_.data(), input_dim_,
                                  num_ctrl_pts_};
  const int order{spline_degree_ + 1};
  u0.noalias() = ctrls.leftCols(order) * spline_weights_.col(0);
}

void MPCBase::getParameterizedInputTrajectory(
    Ref<VectorXd> u_traj_ctrl_pts) const noexcept
{
  assert(u_traj_ctrl_pts.size() == ctrls_dim_);
  u_traj_ctrl_pts = solution_map_.head(ctrls_dim_);
}

void MPCBase::getInputTrajectory(Ref<VectorXd> u_traj) const noexcept
{
  assert(u_traj.size() == u_traj_dim_);

  const Map<const MatrixXd> ctrls{solution_map_.data(), input_dim_,
                                  num_ctrl_pts_};
  Map<MatrixXd> u_traj_mat{u_traj.data(), input_dim_, horizon_steps_};

  const int order{spline_degree_ + 1};
  for (int k{0}; k < horizon_steps_; ++k) {
    const int seg{spline_segment_idxs_(k)};
    u_traj_mat.col(k).noalias() =
        ctrls.middleCols(seg, order) * spline_weights_.col(k);
  }
}

void MPCBase::getPredictedStateTrajectory(Ref<VectorXd> x_traj) const noexcept
{
  assert(x_traj.size() == x_traj_dim_);
}

void MPCBase::propagateModel(const Ref<const VectorXd>& x0,
                             const Ref<const VectorXd>& u,
                             Ref<VectorXd> x_next) const
{
  if (!model_set_)
    throw std::logic_error("Model must be set before propagation");
  assert(u.size() == input_dim_);
  assert(x0.size() == state_dim_ && x_next.size() == state_dim_);
  // do not use noalias here since x_next could be an alias of x0
  x_next = Ad_ * x0 + Bd_ * u + wd_;
}

bool MPCBase::setModelDiscrete(const Ref<const MatrixXd>& Ad,
                               const Ref<const MatrixXd>& Bd,
                               const Ref<const VectorXd>& wd)
{
  assert(Ad.rows() == state_dim_ && Ad.cols() == state_dim_);
  assert(Bd.rows() == state_dim_ && Bd.cols() == input_dim_);
  assert(wd.size() == state_dim_);

  Ad_ = Ad;
  Bd_ = Bd;
  wd_ = wd;
  model_set_ = true;
  return qpUpdateModel();
}

bool MPCBase::setModelContinuous2Discrete(const Ref<const MatrixXd>& Ac,
                                          const Ref<const MatrixXd>& Bc,
                                          const Ref<const VectorXd>& wc,
                                          const double dt,
                                          const double tol)
{
  assert(Ac.rows() == state_dim_ && Ac.cols() == state_dim_);
  assert(Bc.rows() == state_dim_ && Bc.cols() == input_dim_);
  assert(wc.size() == state_dim_);

  // allocates memory first time only (since sizes are constant)
  At_.resize(state_dim_, state_dim_);
  At_.noalias() = Ac * dt;
  At_pow_.setIdentity(state_dim_, state_dim_);
  Ad_.setIdentity(state_dim_, state_dim_);
  G_.setIdentity(state_dim_, state_dim_);

  int i{1};
  double factorial{1};
  for (double t_pow{dt}; t_pow / factorial > tol; t_pow *= dt) {
    At_pow_ *= At_;
    Ad_ += At_pow_ / factorial;
    factorial *= ++i;
    G_ += At_pow_ / factorial;
  }
  G_ *= dt;

  Bd_.noalias() = G_ * Bc;
  wd_.noalias() = G_ * wc;
  model_set_ = true;
  return qpUpdateModel();
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
  if (Q_diag.minCoeff() < 0.0)
    throw std::invalid_argument(
        "[MPCBase::setStateWeights] State weights must be non-negative.");
  assert(Q_diag.size() == state_dim_);
  Q_big_.diagonal() = Q_diag.replicate(horizon_steps_, 1);
  weights_changed_ = true;
}

void MPCBase::setStateWeights(const Ref<const VectorXd>& Q_diag,
                              const Ref<const VectorXd>& Qf_diag)
{
  if (Q_diag.minCoeff() < 0.0 || Qf_diag.minCoeff() < 0.0)
    throw std::invalid_argument(
        "[MPCBase::setStateWeights] State weights must be non-negative.");
  assert(Q_diag.size() == state_dim_ && Qf_diag.size() == state_dim_);
  Q_big_.diagonal().head(state_dim_ * (horizon_steps_ - 1)) =
      Q_diag.replicate(horizon_steps_ - 1, 1);
  Q_big_.diagonal().tail(state_dim_) = Qf_diag;
  weights_changed_ = true;
}

void MPCBase::setInputWeights(const Ref<const VectorXd>& R_diag)
{
  if (!opts_.use_input_cost)
    throw std::logic_error(
        "[MPCBase::setInputWeights] Input cost is not enabled.");
  if (R_diag.minCoeff() < 0.0)
    throw std::invalid_argument(
        "[MPCBase::setInputWeights] Input weights must be non-negative.");
  assert(R_diag.size() == input_dim_);
  R_big_.diagonal() = R_diag.replicate(num_ctrl_pts_, 1);
  weights_changed_ = true;
}

bool MPCBase::setReferenceState(const Ref<const VectorXd>& x_step)
{
  assert(x_step.size() == state_dim_);
  x_ref_ = x_step.replicate(horizon_steps_, 1);
  return qpUpdateReferences();
}

bool MPCBase::setReferenceStateTrajectory(const Ref<const VectorXd>& x_traj)
{
  assert(x_traj.size() == x_traj_dim_);
  x_ref_ = x_traj;
  return qpUpdateReferences();
}

bool MPCBase::setReferenceInput(const Ref<const VectorXd>& u_step)
{
  if (!opts_.use_input_cost)
    throw std::logic_error(
        "[MPCBase::setReferenceInput] Input cost is not enabled.");
  assert(u_step.size() == input_dim_);
  u_ref_ = u_step.replicate(num_ctrl_pts_, 1);
  return qpUpdateReferences();
}

bool MPCBase::setReferenceParameterizedInputTrajectory(
    const Ref<const VectorXd>& u_traj_ctrl_pts)
{
  if (!opts_.use_input_cost)
    throw std::logic_error(
        "[MPCBase::setReferenceParameterizedInputTrajectory] "
        "Input cost is not enabled.");
  assert(u_traj_ctrl_pts.size() == ctrls_dim_);
  u_ref_ = u_traj_ctrl_pts;
  return qpUpdateReferences();
}

bool MPCBase::setInputLimits(const Ref<const VectorXd>& u_min,
                             const Ref<const VectorXd>& u_max)
{
  if ((u_max - u_min).minCoeff() < 0.0)
    throw std::invalid_argument(
        "[MPCBase::setInputLimits] u_min cannot be greater than u_max.");
  assert(u_min.size() == input_dim_ && u_max.size() == input_dim_);
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
  if ((x_max - x_min).minCoeff() < 0.0)
    throw std::invalid_argument(
        "[MPCBase::setStateLimits] x_min cannot be greater than x_max.");
  assert(x_min.size() == state_dim_ && x_max.size() == state_dim_);
  x_min_ = x_min;
  x_max_ = x_max;
  x_lims_set_ = true;

  l_.tail(x_sat_dim_) = x_min_.replicate(horizon_steps_, 1);
  u_.tail(x_sat_dim_) = x_max_.replicate(horizon_steps_, 1);
  return qpUpdateStateLimits();
}

bool MPCBase::setSlewRate(const Ref<const VectorXd>& u_slew)
{
  if (!opts_.slew_control_points)
    throw std::logic_error("[MPCBase::setSlewRate] Slew rate is not enabled.");
  if (u_slew.minCoeff() < 0.0)
    throw std::invalid_argument(
        "[MPCBase::setSlewRate] Slew rate must be non-negative.");
  assert(u_slew.size() == input_dim_);
  u_slew_ = u_slew;
  slew_rate_set_ = true;

  u_.segment(slew_idx_, slew_dim_) = u_slew_.replicate(num_ctrl_pts_ - 1, 1);
  l_.segment(slew_idx_, slew_dim_) = -u_.segment(slew_idx_, slew_dim_);
  return qpUpdateSlewRate();
}

bool MPCBase::setSlewRateInitial(const Ref<const VectorXd>& u0_slew)
{
  if (!opts_.slew_initial_input)
    throw std::logic_error(
        "[MPCBase::setSlewRateInitial] Initial slew rate is not enabled.");
  if (u0_slew.minCoeff() < 0.0)
    throw std::invalid_argument(
        "[MPCBase::setSlewRate] Slew rate must be non-negative.");
  assert(u0_slew.size() == input_dim_);
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
  assert(u_prev.size() == input_dim_);
  u_prev_ = u_prev;

  l_.segment(slew0_idx_, input_dim_) = u_prev_ - u0_slew_;
  u_.segment(slew0_idx_, input_dim_) = u_prev_ + u0_slew_;
  return qpUpdateSlewRate();
}

void MPCBase::calcSplineParams()
{
  using Spline1d = Spline<double, 1>;
  for (int k{0}; k < horizon_steps_; ++k) {
    const double t = k;
    const int span = Spline1d::Span(t, spline_degree_, spline_knots_);
    spline_segment_idxs_(k) = span - spline_degree_;

    spline_weights_.col(k) =
        Spline1d::BasisFunctions(t, spline_degree_, spline_knots_);
  }
}

} // namespace affine_mpc
