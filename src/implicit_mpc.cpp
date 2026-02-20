#include "affine_mpc/implicit_mpc.hpp"

#include <Eigen/Core>

#include "affine_mpc/mpc_base.hpp"
#include "affine_mpc/osqp_solver.hpp"

namespace affine_mpc {

using Eigen::Ref;
using Eigen::VectorXd;


ImplicitMPC::ImplicitMPC(const int state_dim,
                         const int input_dim,
                         const int horizon_steps,
                         const int num_control_points,
                         const bool use_input_cost,
                         const bool use_slew_rate,
                         const bool saturate_states) :
    MPCBase(state_dim,
            input_dim,
            horizon_steps,
            num_control_points,
            1,
            VectorXd(0),
            use_input_cost,
            use_slew_rate,
            saturate_states),
    // if using a slew rate constraint then state saturation rows of A are
    // shifted down by the number of slew rate constraints
    x_sat_idx_{input_dim * num_control_points
               + input_dim * (num_control_points - 1) * use_slew_rate},
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
  // Avoids setting first row block of S_ to zero every time solve() is called
  // (a minor speed optimization)
  S_.topRows(state_dim).setZero();
}

// Implementation from section IV.C of https://arxiv.org/pdf/2001.04931
void ImplicitMPC::getInputTrajectory(Ref<VectorXd> u_traj) const noexcept
{
  MPCBase::getInputTrajectory(u_traj);

  VectorXd uk{input_dim_}, ctrl_pt1{input_dim_}, ctrl_pt2{input_dim_};
  double Tp{(horizon_steps_ - 1) / double(num_ctrl_pts_ - 1)};
  int idx1, idx2;
  double c;

  for (int k{0}; k < horizon_steps_ - 1; ++k) {
    idx1 = int(k / Tp);
    idx2 = idx1 + 1;
    c = k / Tp - idx1;
    ctrl_pt1 = solution_map_.segment(idx1 * input_dim_, input_dim_);
    ctrl_pt2 = solution_map_.segment(idx2 * input_dim_, input_dim_);
    uk = (1 - c) * ctrl_pt1 + c * ctrl_pt2;
    u_traj.segment(k * input_dim_, input_dim_) = uk;
  }
  u_traj.tail(input_dim_) = solution_map_.tail(input_dim_);
}

void ImplicitMPC::getPredictedStateTrajectory(
    Ref<VectorXd> x_traj) const noexcept
{
  MPCBase::getPredictedStateTrajectory(x_traj);
  x_traj.noalias() = S_ * solution_map_;
  x_traj += v_;
}

bool ImplicitMPC::setInputLimits(const Ref<const VectorXd>& u_min,
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

bool ImplicitMPC::setStateLimits(const Ref<const VectorXd>& x_min,
                                 const Ref<const VectorXd>& x_max)
{
  MPCBase::setStateLimits(x_min, x_max);

  A_.block(x_sat_idx_, 0, state_dim_ * horizon_steps_,
           input_dim_ * num_ctrl_pts_) = S_;
  for (int k{0}; k < horizon_steps_; ++k) {
    l_.segment(x_sat_idx_ + state_dim_ * k, state_dim_) = x_min_;
    u_.segment(x_sat_idx_ + state_dim_ * k, state_dim_) = x_max_;
  }
  if (!solver_initialized_)
    return true;
  return solver_->updateBounds(l_, u_);
}

bool ImplicitMPC::setSlewRate(const Ref<const VectorXd>& u_slew)
{
  MPCBase::setSlewRate(u_slew);

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

void ImplicitMPC::updateQP(const Ref<const VectorXd>& x0)
{
  calcSAndV(x0);
  calcPandQ();
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

void ImplicitMPC::calcSAndV(const Ref<const VectorXd>& x0)
{
  double Tp{(horizon_steps_ - 1) / double(num_ctrl_pts_ - 1)};

  S_.block(0, 0, state_dim_, input_dim_) = Bd_;
  v_.segment(0, state_dim_).noalias() = Ad_ * x0;
  v_.segment(0, state_dim_) += wd_;

  int i, j;
  double c;
  int n{state_dim_};
  int m{input_dim_};
  for (int k{1}; k < horizon_steps_; ++k) {
    i = k / Tp;
    j = i + 1;
    c = k / Tp - i;

    // S_.block(state_dim_*k,0,state_dim_,input_dim_*num_ctrl_pts_) = Ad_ *
    // S_.block(state_dim_*(k-1),0,state_dim_,input_dim_*num_ctrl_pts_);
    S_.block(state_dim_ * k, 0, state_dim_, S_.cols()).noalias() =
        Ad_ * S_.block(state_dim_ * (k - 1), 0, state_dim_, S_.cols());
    // S_.block(n*k,0,n,m*num_ctrl_pts_) = Ad_ *
    // S_.block(n*(k-1),0,n,m*num_ctrl_pts_);
    S_.block(n * k, m * i, n, m) += (1 - c) * Bd_;

    if (j < num_ctrl_pts_)
      S_.block(state_dim_ * k, input_dim_ * j, state_dim_, input_dim_) +=
          c * Bd_;

    v_.segment(state_dim_ * k, state_dim_).noalias() =
        Ad_ * v_.segment(state_dim_ * (k - 1), state_dim_);
    v_.segment(state_dim_ * k, state_dim_) += wd_;
  }
}

void ImplicitMPC::calcPandQ()
{
  P_.noalias() = S_.transpose() * Q_big_ * S_;
  q_.noalias() = S_.transpose() * Q_big_ * (v_ - x_goal_);
  if (use_input_cost_) {
    P_ += R_big_;
    q_.noalias() -= R_big_ * u_goal_;
  }
}

} // namespace affine_mpc
