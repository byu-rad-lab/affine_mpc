#include "affine_mpc/mpc_base.hpp"

#include <Eigen/Core>
#include <osqp.h> // for OSQPSettings

namespace affine_mpc {

using Eigen::Ref;
using Eigen::Map;
using Eigen::MatrixXd;
using Eigen::VectorXd;


MPCBase::MPCBase(const int num_states, const int num_inputs,
                 const int len_horizon, const int num_control_points,
                 const bool use_input_cost, const bool use_slew_rate,
                 const bool saturate_states) :
    num_states_{num_states}, num_inputs_{num_inputs},
    len_horizon_{len_horizon}, num_ctrl_pts_{num_control_points},
    use_input_cost_{use_input_cost},
    use_slew_rate_{use_slew_rate},
    saturate_states_{saturate_states},
    model_set_{false},
    input_limits_set_{false},
    slew_rate_set_{false},
    state_limits_set_{false},
    solver_initialized_{false},
    solver_{nullptr},
    Ad_{num_states,num_states},
    Bd_{num_states,num_inputs},
    wd_{num_states},
    Q_big_{num_states*len_horizon},
    x_goal_{num_states*len_horizon},
    u_min_{num_inputs},
    u_max_{num_inputs},
    solution_map_{nullptr,0}
{
  assert(num_states > 0 && num_inputs > 0 && len_horizon > 0);
  assert(num_control_points > 0 && num_control_points <= len_horizon);
  Q_big_.setIdentity(); // default state weights

  // allocate memory needed based on options
  if (use_input_cost)
  {
    R_big_.setZero(num_inputs*num_control_points);
    u_goal_.setZero(num_inputs*num_control_points);
  }
  if (use_slew_rate)
    u_slew_.setZero(num_inputs);
  if (saturate_states)
  {
    x_min_.setZero(num_states);
    x_max_.setZero(num_states);
  }
}

MPCBase::~MPCBase()
{
  // Derived classes create solver_ but do not need to delete it.
  // This deletes the solver, derived classes can use default destructor.
  if (solver_)
    delete solver_;
}

bool MPCBase::initializeSolver(const OSQPSettings* solver_settings)
{
  assert(model_set_ && input_limits_set_);
  if (solver_initialized_)
    return true;

  if (use_slew_rate_)
    assert(slew_rate_set_);
  if (saturate_states_)
    assert(state_limits_set_);

  // Avoid zeros in initial cost/constraint matrices. OSQP is a sparse solver
  // that only tracks elements that are initially non-zero.
  VectorXd x_full{num_states_};
  x_full.setOnes(); // may need something other than ones
  convertToQP(x_full);

  solver_initialized_ = solver_->initialize(P_, A_, q_, l_, u_, solver_settings);
  // According to Eigen's documentation, use "placement new" syntax to update
  // solution_map_ with the new pointer to the solution vector. This does not
  // allocate new memory that needs to be deleted, it just updates the pointer.
  // This location in memory is managed by the solver_ object and is unchanged
  // while the solver_ is in scope. solution_map_ will function like an Eigen
  // VectorXd object that automatically updates when solve() is called. Using
  // Eigen Map allows the solution to be accessed without copying memory.
  new (&solution_map_) Map<const VectorXd>(solver_->getSolutionPtr(),
                                           num_inputs_*num_ctrl_pts_);
  return solver_initialized_;
}

bool MPCBase::solve(const Ref<const VectorXd>& x0)
{
  assert(x0.size() == num_states_);
  convertToQP(x0);
  return solver_->solve();
}

void MPCBase::getNextInput(Ref<VectorXd> u0) const
{
  assert(u0.size() == num_inputs_);
  u0 = solution_map_.head(num_inputs_);
}

void MPCBase::getParameterizedInputTrajectory(Ref<VectorXd> u_traj_ctrl_pts) const
{
  assert(u_traj_ctrl_pts.size() == num_inputs_*num_ctrl_pts_);
  u_traj_ctrl_pts = solution_map_.head(num_inputs_*num_ctrl_pts_);
}

void MPCBase::getInputTrajectory(Ref<VectorXd> u_traj) const
{
  assert(u_traj.size() == num_inputs_*len_horizon_);
}

void MPCBase::getPredictedStateTrajectory(Ref<VectorXd> x_traj) const
{
  assert(x_traj.size() == num_states_*len_horizon_);
}

void MPCBase::propagateModel(const Ref<const VectorXd>& x0, const Ref<const VectorXd>& u,
                             Ref<VectorXd> x_next) const
{
  assert(model_set_ && u.size() == num_inputs_);
  assert(x0.size() == num_states_ && x_next.size() == num_states_);
  x_next = Ad_*x0 + Bd_*u + wd_;
}

void MPCBase::setModelDiscrete(const Ref<const MatrixXd>& Ad,
                               const Ref<const MatrixXd>& Bd,
                               const Ref<const VectorXd>& wd)
{
  assert(Ad.rows() == num_states_ && Ad.cols() == num_states_);
  assert(Bd.rows() == num_states_ && Bd.cols() == num_inputs_);
  assert(wd.size() == num_states_);

  Ad_ = Ad;
  Bd_ = Bd;
  wd_ = wd;
  model_set_ = true;
}

void MPCBase::setModelContinuous2Discrete(const Ref<const MatrixXd>& Ac,
                                          const Ref<const MatrixXd>& Bc,
                                          const Ref<const VectorXd>& wc,
                                          double dt, double tol)
{
  assert(Ac.rows() == num_states_ && Ac.cols() == num_states_);
  assert(Bc.rows() == num_states_ && Bc.cols() == num_inputs_);
  assert(wc.size() == num_states_);

  static MatrixXd G{num_states_,num_states_};
  static MatrixXd At{num_states_,num_states_};
  static MatrixXd At_i{num_states_,num_states_};

  At.noalias() = Ac * dt;
  At_i.setIdentity(num_states_,num_states_);
  Ad_.setIdentity(num_states_,num_states_);
  G.setIdentity(num_states_,num_states_);

  int i{1};
  double factorial{1};
  for (double t_i{dt}; t_i/factorial > tol; t_i*=dt)
  {
    At_i *= At;
    Ad_ += At_i / factorial;
    factorial *= ++i;
    G += At_i / factorial;
  }
  G *= dt;

  Bd_.noalias() = G * Bc;
  wd_.noalias() = G * wc;
  model_set_ = true;
}

void MPCBase::setWeights(const Ref<const VectorXd>& Q_diag,
                         const Ref<const VectorXd>& R_diag)
{
  setStateWeights(Q_diag);
  setInputWeights(R_diag);
}

void MPCBase::setStateWeights(const Ref<const VectorXd>& Q_diag)
{
  assert(Q_diag.size() == num_states_);
  for (int i{0}; i < len_horizon_; ++i)
    Q_big_.diagonal().segment(num_states_*i,num_states_) = Q_diag;
}

void MPCBase::setStateWeightsTerminal(const Ref<const VectorXd>& Qf_diag)
{
  assert(Qf_diag.size() == num_states_);
  Q_big_.diagonal().segment(num_states_*(len_horizon_-1),num_states_) = Qf_diag;
}

void MPCBase::setInputWeights(const Ref<const VectorXd>& R_diag)
{
  assert(use_input_cost_);
  assert(R_diag.size() == num_inputs_);
  for (int i{0}; i < num_ctrl_pts_; ++i)
    R_big_.diagonal().segment(num_inputs_*i,num_inputs_) = R_diag;
}

void MPCBase::setReferenceState(const Ref<const VectorXd>& x_step)
{
  assert(x_step.size() == num_states_);
  for (int k{0}; k < len_horizon_; ++k)
    x_goal_.segment(num_states_*k,num_states_) = x_step;
}

void MPCBase::setReferenceInput(const Ref<const VectorXd>& u_step)
{
  assert(use_input_cost_);
  assert(u_step.size() == num_inputs_);
  for (int i{0}; i < num_ctrl_pts_; ++i)
    u_goal_.segment(num_inputs_*i,num_inputs_) = u_step;
}

void MPCBase::setReferenceStateTrajectory(const Ref<const VectorXd>& x_traj)
{
  assert(x_traj.size() == num_states_*len_horizon_);
  x_goal_ = x_traj;
}

void MPCBase::setReferenceParameterizedInputTrajectory(
    const Ref<const VectorXd>& u_traj_ctrl_pts)
{
  assert(use_input_cost_);
  assert(u_traj_ctrl_pts.size() == num_inputs_*num_ctrl_pts_);
  u_goal_ = u_traj_ctrl_pts;
}

void MPCBase::setInputLimits(const Ref<const VectorXd>& u_min,
                             const Ref<const VectorXd>& u_max)
{
  assert(u_min.size() == num_inputs_ && u_max.size() == num_inputs_);
  assert((u_max-u_min).all() >= 0.0);
  u_min_ = u_min;
  u_max_ = u_max;
  input_limits_set_ = true;
}

void MPCBase::setStateLimits(const Ref<const VectorXd>& x_min,
                             const Ref<const VectorXd>& x_max)
{
  assert(saturate_states_);
  assert(x_min.size() == num_states_ && x_max.size() == num_states_);
  assert((x_max-x_min).all() >= 0);
  x_min_ = x_min;
  x_max_ = x_max;
  state_limits_set_ = true;
}

void MPCBase::setSlewRate(const Ref<const VectorXd>& u_slew)
{
  assert(use_slew_rate_);
  assert(u_slew.size() == num_inputs_);
  u_slew_ = u_slew;
  slew_rate_set_ = true;
}

} // namespace affine_mpc
