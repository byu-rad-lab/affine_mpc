#include "affine_mpc/condensed_mpc.hpp"

// #include <Eigen/Core> // revert back to this once Eigen 3.5 is required
#include "eigen_compat.hpp" // revmove this once Eigen 3.5 is required
#include <exception>
#include <sys/stat.h>
#include <unsupported/Eigen/Splines>

#include "affine_mpc/mpc_base.hpp"
#include "affine_mpc/osqp_solver.hpp"

using namespace Eigen;
// revert back to this once Eigen 3.5 is required
// namespace ph = Eigen::placeholders;

namespace affine_mpc {


CondensedMPC::CondensedMPC(const int state_dim,
                           const int input_dim,
                           const int horizon_steps,
                           const int num_control_points,
                           const int spline_degree,
                           const Ref<const VectorXd>& knots,
                           const bool use_input_cost,
                           const bool use_slew_rate,
                           const bool saturate_states) :
    MPCBase(state_dim,
            input_dim,
            horizon_steps,
            num_control_points,
            spline_degree,
            knots,
            use_input_cost,
            use_slew_rate,
            saturate_states),
    // if using a slew rate constraint then state saturation rows of A are
    // shifted down by the number of slew rate constraints
    x_sat_idx_{input_dim * num_control_points
               + input_dim * (num_control_points - 1) * use_slew_rate},
    num_constraints_{x_sat_idx_ + state_dim * horizon_steps * saturate_states},
    S_{state_dim * horizon_steps, input_dim * num_control_points},
    v_{state_dim * horizon_steps},
    model_changed_{false}
{
  const int num_design_vars{input_dim * num_control_points};
  solver_ = std::make_unique<OSQPSolver>(num_design_vars, num_constraints_);
  P_.resize(num_design_vars, num_design_vars);
  A_.resize(num_constraints_, num_design_vars);
  q_.resize(num_design_vars);
  l_.resize(num_constraints_);
  u_.resize(num_constraints_);

  A_.setIdentity(); // input saturation conststraint
  if (use_slew_rate) {
    // shorten block index variables for readability
    const auto slew_rows{seqN(ctrls_dim_, ctrls_dim_ - input_dim)};
    A_(slew_rows, ph::all).diagonal().setConstant(-1.0);
    A_(slew_rows, seq(input_dim, ph::last)).diagonal().setOnes();
  }
  // Avoids setting first row block of S_ to zero every time solve() is
  // called (a minor speed optimization)
  S_.topRows(state_dim).setZero();
}

void CondensedMPC::getPredictedStateTrajectory(
    Ref<VectorXd> x_traj) const noexcept
{
  MPCBase::getPredictedStateTrajectory(x_traj); // size checks
  x_traj.noalias() = S_ * solution_map_;
  x_traj += v_;
}

void CondensedMPC::setModelDiscrete(const Ref<const MatrixXd>& Ad,
                                    const Ref<const MatrixXd>& Bd,
                                    const Ref<const VectorXd>& wd)
{
  MPCBase::setModelDiscrete(Ad, Bd, wd);

  model_changed_ = true;
  updateS();
}

void CondensedMPC::setModelContinuous2Discrete(const Ref<const MatrixXd>& Ac,
                                               const Ref<const MatrixXd>& Bc,
                                               const Ref<const VectorXd>& wc,
                                               double dt,
                                               double tol)
{
  MPCBase::setModelContinuous2Discrete(Ac, Bc, wc, dt, tol);

  model_changed_ = true;
  updateS();
}

bool CondensedMPC::setInputLimits(const Ref<const VectorXd>& u_min,
                                  const Ref<const VectorXd>& u_max)
{
  MPCBase::setInputLimits(u_min, u_max);

  for (int k{0}; k < num_ctrl_pts_; ++k) {
    l_.segment(input_dim_ * k, input_dim_) = u_min_;
    u_.segment(input_dim_ * k, input_dim_) = u_max_;
  }
  if (!solver_initialized_)
    return true;
  return solver_->updateBounds(l_, u_);
}

bool CondensedMPC::setStateLimits(const Ref<const VectorXd>& x_min,
                                  const Ref<const VectorXd>& x_max)
{
  MPCBase::setStateLimits(x_min, x_max);

  /* math explained
  x_min_stacked <= x_traj <= x_max_stacked
  x_min_stacked <= S_*z + v_ <= x_max_stacked
  x_min_stacked - v_ <= S_*z <= x_max_stacked - v_
  */
  l_.tail(x_traj_dim_) = x_min_.replicate(horizon_steps_, 1) - v_;
  u_.tail(x_traj_dim_) = x_max_.replicate(horizon_steps_, 1) - v_;
  if (!solver_initialized_)
    return true;

  return solver_->updateBounds(l_, u_);
}

bool CondensedMPC::setSlewRate(const Ref<const VectorXd>& u_slew)
{
  MPCBase::setSlewRate(u_slew);

  const int slew_dim{ctrls_dim_ - input_dim_};
  const auto neg_slew = (-u_slew_).eval();
  l_.segment(ctrls_dim_, slew_dim) = neg_slew.replicate(num_ctrl_pts_ - 1, 1);
  u_.segment(ctrls_dim_, slew_dim) = u_slew_.replicate(num_ctrl_pts_ - 1, 1);
  if (!solver_initialized_)
    return true;
  return solver_->updateBounds(l_, u_);
}

void CondensedMPC::updateQP(const Ref<const VectorXd>& x0)
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
