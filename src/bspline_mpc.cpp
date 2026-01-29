#include "affine_mpc/bspline_mpc.hpp"

// #include <Eigen/Core> // revert back to this once Eigen 3.5 is required
#include "eigen_compat.hpp" // revmove this once Eigen 3.5 is required
#include <exception>
#include <unsupported/Eigen/Splines>

#include "affine_mpc/mpc_base.hpp"
#include "affine_mpc/osqp_solver.hpp"

using namespace Eigen;
// revert back to this once Eigen 3.5 is required
// namespace ph = Eigen::placeholders;

namespace affine_mpc {


BSplineMPC::BSplineMPC(const int state_dim,
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
    x_sat_idx_{input_dim * num_control_points +
               input_dim * (num_control_points - 1) * use_slew_rate},
    num_constraints_{x_sat_idx_ + state_dim * horizon_steps * saturate_states},
    S_{state_dim * horizon_steps, input_dim * num_control_points},
    v_{state_dim * horizon_steps}
{
  int num_design_vars{input_dim * num_control_points};
  solver_ = std::make_unique<OSQPSolver>(num_design_vars, num_constraints_);
  P_.resize(num_design_vars, num_design_vars);
  A_.resize(num_constraints_, num_design_vars);
  q_.resize(num_design_vars);
  l_.resize(num_constraints_);
  u_.resize(num_constraints_);

  A_.setIdentity(); // input saturation conststraint
  if (use_slew_rate) {
    // shorten block index variables for readability
    int mp{input_dim * num_control_points};
    int mp_m{input_dim * num_control_points - input_dim};
    A_.block(mp, 0, mp_m, mp).diagonal().setConstant(-1);
    A_.block(mp, input_dim_, mp_m, mp_m).diagonal().setOnes();
  }
  // Avoids setting first row block of S_ to zero every time solve() is
  // called (a minor speed optimization)
  S_.topRows(state_dim).setZero();
}

// void BSplineMPC::getInputTrajectory(Ref<VectorXd> u_traj) const
// {
//   MPCBase::getInputTrajectory(u_traj); // size checks
//
//   int seg;
//   Map<const MatrixXd> ctrls{solution_map_.data(), input_dim_,
//   num_ctrl_pts_};
//   VectorXd weights;
//
//   for (int k{0}; k < horizon_steps_; ++k) {
//     seg = spline_segment_idxs_(k);
//     u_traj(seqN(k, input_dim_)) =
//         ctrls(ph::all, seqN(seg, spline_degree_ + 1)) *
//         spline_weights_.col(k);
//   }
// }

void BSplineMPC::getPredictedStateTrajectory(
    Ref<VectorXd> x_traj) const noexcept
{
  MPCBase::getPredictedStateTrajectory(x_traj); // size checks
  x_traj.noalias() = S_ * solution_map_;
  x_traj += v_;
}

bool BSplineMPC::setInputLimits(const Ref<const VectorXd>& u_min,
                                const Ref<const VectorXd>& u_max)
{
  MPCBase::setInputLimits(u_min, u_max); // size checks

  for (int k{0}; k < num_ctrl_pts_; ++k) {
    l_.segment(input_dim_ * k, input_dim_) = u_min_;
    u_.segment(input_dim_ * k, input_dim_) = u_max_;
  }
  if (!solver_initialized_)
    return true;
  return solver_->updateBounds(l_, u_);
}

bool BSplineMPC::setStateLimits(const Ref<const VectorXd>& x_min,
                                const Ref<const VectorXd>& x_max)
{
  MPCBase::setStateLimits(x_min, x_max); // size checks

  /* math explained
  x_min_stacked <= x_traj <= x_max_stacked
  x_min_stacked <= S_*z + v_ <= x_max_stacked
  x_min_stacked - v_ <= S_*z <= x_max_stacked - v_
  */

  // A_.block(x_sat_idx_, 0, state_dim_ * horizon_steps_,
  //          input_dim_ * num_ctrl_pts_) = S_;
  const auto x_sat_rows = seq(x_sat_idx_, ph::last);
  A_(x_sat_rows, ph::all) = S_;
  for (int k{0}; k < horizon_steps_; ++k) {
    l_.segment(x_sat_idx_ + state_dim_ * k, state_dim_) = x_min_;
    u_.segment(x_sat_idx_ + state_dim_ * k, state_dim_) = x_max_;
  }
  l_(x_sat_rows) -= v_;
  u_(x_sat_rows) -= v_;
  if (!solver_initialized_)
    return true;

  return solver_->updateBounds(l_, u_);
}

bool BSplineMPC::setSlewRate(const Ref<const VectorXd>& u_slew)
{
  MPCBase::setSlewRate(u_slew); // size checks

  int mp{input_dim_ * num_ctrl_pts_};
  int max{num_ctrl_pts_ - 1};
  for (int i{0}; i < max; ++i) {
    l_.segment(mp + input_dim_ * i, input_dim_) = -u_slew_;
    u_.segment(mp + input_dim_ * i, input_dim_) = u_slew_;
  }
  if (!solver_initialized_)
    return true;
  return solver_->updateBounds(l_, u_);
}

void BSplineMPC::convertToQP(const Ref<const VectorXd>& x0)
{
  calcSAndV(x0);
  calcPAndQ();
  bool success{true};
  success &= solver_->updateCostMatrix(P_);
  success &= solver_->updateCostVector(q_);
  if (saturate_states_) {
    A_.block(x_sat_idx_, 0, S_.rows(), S_.cols()) = S_;
    success &= solver_->updateConstraintMatrix(A_);
  }
  // Note: success is used to avoid build warnings. Failures will manifest in
  // either initializeSolver() or solve(), no need to check on protected method
}

void BSplineMPC::calcSAndV(const Ref<const VectorXd>& x0)
{
  int n{state_dim_};
  int m{input_dim_};

  // add dynamics of initial time step (k=0)
  VectorXd weights = spline_weights_.col(0);
  for (int i{0}; i < spline_weights_.rows(); ++i)
    S_(seqN(0, n), seqN(m * i, m)).noalias() = weights(i) * Bd_;
  // S_(seqN(0, n), seqN(0, m)) = Bd_; // only works if spline is clamped
  v_.head(n).noalias() = Ad_ * x0;
  v_.head(n) += wd_;

  int seg;
  for (int k{1}; k < horizon_steps_; ++k) {
    seg = spline_segment_idxs_(k);
    weights = spline_weights_.col(k);

    // add cumulative effect of all previous dynamics
    auto rows_cur = seqN(n * k, n);
    auto rows_prev = seqN(n * (k - 1), n);
    S_(rows_cur, ph::all).noalias() = Ad_ * S_(rows_prev, ph::all);
    v_(rows_cur).noalias() = Ad_ * v_(rows_prev);
    v_(rows_cur) += wd_;

    // add effect of current input parameterized by spline
    for (int i{0}; i < spline_weights_.rows(); ++i)
      S_(rows_cur, seqN(m * (seg + i), m)) += weights(i) * Bd_;
  }
}

void BSplineMPC::calcPAndQ()
{
  P_.noalias() = S_.transpose() * Q_big_ * S_;
  q_.noalias() = S_.transpose() * Q_big_ * (v_ - x_goal_);
  if (use_input_cost_) {
    P_ += R_big_;
    q_ -= R_big_ * u_goal_;
  }
}

} // namespace affine_mpc
