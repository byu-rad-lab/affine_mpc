#include "affine_mpc/implicit_mpc.hpp"

#include <Eigen/Core>

#include "affine_mpc/mpc_base.hpp"
#include "affine_mpc/osqp_solver.hpp"

namespace affine_mpc {

using Eigen::Ref;
using Eigen::VectorXd;


ImplicitMPC::ImplicitMPC(const int num_states,
                         const int num_inputs,
                         const int len_horizon,
                         const int num_control_points,
                         const bool use_input_cost,
                         const bool use_slew_rate,
                         const bool saturate_states) :
    MPCBase(num_states,
            num_inputs,
            len_horizon,
            num_control_points,
            1,
            VectorXd(0),
            use_input_cost,
            use_slew_rate,
            saturate_states),
    // if using a slew rate constraint then state saturation rows of A are
    // shifted down by the number of slew rate constraints
    x_sat_idx_{num_inputs * num_control_points +
               num_inputs * (num_control_points - 1) * use_slew_rate},
    num_constraints_{x_sat_idx_ + num_states * len_horizon * saturate_states},
    S_{num_states * len_horizon, num_inputs * num_control_points},
    v_{num_states * len_horizon}
{
  int num_design_vars{num_inputs * num_control_points};
  solver_ = new OSQPSolver{num_design_vars, num_constraints_};
  P_.resize(num_design_vars, num_design_vars);
  A_.resize(num_constraints_, num_design_vars);
  q_.resize(num_design_vars);
  l_.resize(num_constraints_);
  u_.resize(num_constraints_);

  A_.setIdentity(); // input saturation conststraint
  if (use_slew_rate) {
    // shorten block index variables for readability
    int mp{num_inputs * num_control_points};
    int mp_m{num_inputs * num_control_points - num_inputs};
    A_.block(mp, 0, mp_m, mp).diagonal().setConstant(-1);
    A_.block(mp, num_inputs_, mp_m, mp_m).diagonal().setOnes();
  }
  // Avoids setting first row block of S_ to zero every time solve() is called
  // (a minor speed optimization)
  S_.topRows(num_states).setZero();
}

// Implementation from section IV.C of https://arxiv.org/pdf/2001.04931
void ImplicitMPC::getInputTrajectory(Ref<VectorXd> u_traj) const
{
  MPCBase::getInputTrajectory(u_traj);

  VectorXd uk{num_inputs_}, ctrl_pt1{num_inputs_}, ctrl_pt2{num_inputs_};
  double Tp{(len_horizon_ - 1) / double(num_ctrl_pts_ - 1)};
  int idx1, idx2;
  double c;

  for (int k{0}; k < len_horizon_ - 1; ++k) {
    idx1 = int(k / Tp);
    idx2 = idx1 + 1;
    c = k / Tp - idx1;
    ctrl_pt1 = solution_map_.segment(idx1 * num_inputs_, num_inputs_);
    ctrl_pt2 = solution_map_.segment(idx2 * num_inputs_, num_inputs_);
    uk = (1 - c) * ctrl_pt1 + c * ctrl_pt2;
    u_traj.segment(k * num_inputs_, num_inputs_) = uk;
  }
  u_traj.tail(num_inputs_) = solution_map_.tail(num_inputs_);
}

void ImplicitMPC::getPredictedStateTrajectory(Ref<VectorXd> x_traj) const
{
  MPCBase::getPredictedStateTrajectory(x_traj);
  x_traj = S_ * solution_map_ + v_;
}

void ImplicitMPC::setInputLimits(const Ref<const VectorXd>& u_min,
                                 const Ref<const VectorXd>& u_max)
{
  MPCBase::setInputLimits(u_min, u_max);

  for (int k{0}; k < num_ctrl_pts_; ++k) {
    l_.segment(num_inputs_ * k, num_inputs_) = u_min_;
    u_.segment(num_inputs_ * k, num_inputs_) = u_max_;
  }
  if (solver_initialized_)
    solver_->updateBounds(l_, u_);
}

void ImplicitMPC::setStateLimits(const Ref<const VectorXd>& x_min,
                                 const Ref<const VectorXd>& x_max)
{
  MPCBase::setStateLimits(x_min, x_max);

  A_.block(x_sat_idx_, 0, num_states_ * len_horizon_,
           num_inputs_ * num_ctrl_pts_) = S_;
  for (int k{0}; k < len_horizon_; ++k) {
    l_.segment(x_sat_idx_ + num_states_ * k, num_states_) = x_min_;
    u_.segment(x_sat_idx_ + num_states_ * k, num_states_) = x_max_;
  }
  if (solver_initialized_)
    solver_->updateBounds(l_, u_);
}

void ImplicitMPC::setSlewRate(const Ref<const VectorXd>& u_slew)
{
  MPCBase::setSlewRate(u_slew);

  int mp{num_inputs_ * num_ctrl_pts_};
  int max{num_ctrl_pts_ - 1};
  for (int i{0}; i < max; ++i) {
    l_.segment(mp + num_inputs_ * i, num_inputs_) = -u_slew_;
    u_.segment(mp + num_inputs_ * i, num_inputs_) = u_slew_;
  }
  if (solver_initialized_)
    solver_->updateBounds(l_, u_);
}

void ImplicitMPC::convertToQP(const Ref<const VectorXd>& x0)
{
  calcSAndV(x0);
  calcPandQ();
  solver_->updateCostMatrix(P_);
  solver_->updateCostVector(q_);
  if (saturate_states_) {
    A_.block(x_sat_idx_, 0, S_.rows(), S_.cols()) = S_;
    solver_->updateConstraintMatrix(A_);
  }
}

void ImplicitMPC::calcSAndV(const Ref<const VectorXd>& x0)
{
  double Tp{(len_horizon_ - 1) / double(num_ctrl_pts_ - 1)};

  S_.block(0, 0, num_states_, num_inputs_) = Bd_;
  v_.segment(0, num_states_) = Ad_ * x0 + wd_;

  int i, j;
  double c;
  int n{num_states_};
  int m{num_inputs_};
  for (int k{1}; k < len_horizon_; ++k) {
    i = k / Tp;
    j = i + 1;
    c = k / Tp - i;

    // S_.block(num_states_*k,0,num_states_,num_inputs_*num_ctrl_pts_) = Ad_ *
    // S_.block(num_states_*(k-1),0,num_states_,num_inputs_*num_ctrl_pts_);
    S_.block(num_states_ * k, 0, num_states_, S_.cols()) =
        Ad_ * S_.block(num_states_ * (k - 1), 0, num_states_, S_.cols());
    // S_.block(n*k,0,n,m*num_ctrl_pts_) = Ad_ *
    // S_.block(n*(k-1),0,n,m*num_ctrl_pts_);
    S_.block(n * k, m * i, n, m) += (1 - c) * Bd_;

    if (j < num_ctrl_pts_)
      S_.block(num_states_ * k, num_inputs_ * j, num_states_, num_inputs_) +=
          c * Bd_;

    v_.segment(num_states_ * k, num_states_) =
        Ad_ * v_.segment(num_states_ * (k - 1), num_states_) + wd_;
  }
}

void ImplicitMPC::calcPandQ()
{
  P_ = S_.transpose() * Q_big_ * S_;
  q_ = S_.transpose() * Q_big_ * (v_ - x_goal_);
  if (use_input_cost_) {
    P_ += R_big_;
    q_ -= R_big_ * u_goal_;
  }
}

} // namespace affine_mpc
