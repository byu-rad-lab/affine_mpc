#include "affine_mpc/condensed_mpc.hpp"

// #include <Eigen/Core> // revert back to this once Eigen 3.5 is required
#include "eigen_compat.hpp" // remove this once Eigen 3.5 is required
#include <unsupported/Eigen/Splines>

#include "affine_mpc/mpc_base.hpp"
#include "affine_mpc/osqp_solver.hpp"

using namespace Eigen;
// revert back to this once Eigen 3.5 is required
// namespace ph = Eigen::placeholders;

namespace affine_mpc {


CondensedMPC::CondensedMPC(const int state_dim,
                           const int input_dim,
                           const Parameterization& param,
                           const Options& opts) :
    MPCBase(state_dim,
            input_dim,
            param,
            opts,
            input_dim * param.num_control_points, // num_design_vars
            0),                                   // num_custom_constraints
    S_{x_traj_dim_, ctrls_dim_},
    v_{x_traj_dim_},
    model_changed_{false}
{
  // Avoids setting first row block of S_ to zero every time solve() is
  // called (a minor speed optimization)
  S_.topRows(state_dim).setZero();
}

CondensedMPC::CondensedMPC(const int state_dim,
                           const int input_dim,
                           const int horizon_steps,
                           const Options& opts) :
    CondensedMPC(state_dim,
                 input_dim,
                 Parameterization::moveBlocking(horizon_steps, horizon_steps),
                 opts)
{
  // nothing to do, delegating to main constructor with no parameterization
}

void CondensedMPC::getPredictedStateTrajectory(
    Ref<VectorXd> x_traj) const noexcept
{
  MPCBase::getPredictedStateTrajectory(x_traj); // size checks
  x_traj.noalias() = S_ * solution_map_;
  x_traj += v_;
}

void CondensedMPC::qpUpdateX0(const Ref<const VectorXd>& x0)
{
  // Note: success is used to avoid build warnings. Failures will manifest in
  // either initializeSolver() or solve(), no need to check on protected method
  [[maybe_unused]] bool success;

  // check every time because avoiding this computation is more significant than
  // the overhead of a boolean check if the model does not change often
  if (model_changed_ || weights_changed_) {
    model_changed_ = weights_changed_ = false;
    P_.noalias() = S_.transpose() * Q_big_ * S_;
    if (use_input_cost_)
      P_ += R_big_;
    success = solver_->updateCostMatrix(P_);

    if (saturate_states_) {
      A_.bottomRows(S_.rows()) = S_;
      success = solver_->updateConstraintMatrix(A_);
    }
  }

  updateV(x0);
  q_.noalias() = S_.transpose() * Q_big_ * (v_ - x_goal_);
  if (use_input_cost_)
    q_.noalias() -= R_big_ * u_goal_;
  success = solver_->updateCostVector(q_);
}

bool CondensedMPC::qpUpdateModel()
{
  model_changed_ = true;
  updateS();
  return true;
}

bool CondensedMPC::qpUpdateReferences()
{
  // no need to do anything because they only affect q, which is updated every
  // solve anyway.
  return true;
}

bool CondensedMPC::qpUpdateInputLimits()
{
  if (!solver_initialized_)
    return true;
  return solver_->updateBounds(l_, u_);
}

bool CondensedMPC::qpUpdateStateLimits()
{
  u_.tail(x_traj_dim_) -= v_;
  l_.tail(x_traj_dim_) -= v_;
  if (!solver_initialized_)
    return true;
  return solver_->updateBounds(l_, u_);
}

bool CondensedMPC::qpUpdateSlewRate()
{
  if (!solver_initialized_)
    return true;
  return solver_->updateBounds(l_, u_);
}

void CondensedMPC::updateS()
{
  const int num_weights{spline_degree_ + 1};

  // add dynamics of initial time step (k=0)
  for (int i{0}, col{0}; i < num_weights; ++i, col += input_dim_)
    S_.block(0, col, state_dim_, input_dim_) = spline_weights_(i, 0) * Bd_;

  for (int k{1}, row{state_dim_}; k < horizon_steps_; ++k, row += state_dim_) {
    // add cumulative effect of all previous dynamics
    S_.middleRows(row, state_dim_).noalias() =
        Ad_ * S_.middleRows(row - state_dim_, state_dim_);

    // add effect of current input parameterized by spline
    for (int i{0}, col{input_dim_ * spline_segment_idxs_(k)}; i < num_weights;
         ++i, col += input_dim_)
      S_.block(row, col, state_dim_, input_dim_) += spline_weights_(i, k) * Bd_;
  }
}

void CondensedMPC::updateV(const Ref<const VectorXd>& x0)
{
  // add dynamics of initial time step (k=0)
  v_.head(state_dim_).noalias() = Ad_ * x0 + wd_;

  for (int k{1}, row{state_dim_}; k < horizon_steps_; ++k, row += state_dim_) {
    // add cumulative effect of all previous dynamics
    v_.segment(row, state_dim_).noalias() =
        Ad_ * v_.segment(row - state_dim_, state_dim_) + wd_;
  }
}

} // namespace affine_mpc
