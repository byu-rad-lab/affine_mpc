#include "affine_mpc/bspline_mpc.hpp"

#include <Eigen/Core>
#include <exception>
#include <unsupported/Eigen/Splines>

#include "affine_mpc/mpc_base.hpp"
#include "affine_mpc/osqp_solver.hpp"

using namespace Eigen;
namespace ph = Eigen::placeholders;

namespace affine_mpc {


BSplineMPC::BSplineMPC(const int num_states,
                       const int num_inputs,
                       const int num_steps,
                       const int num_controls,
                       const int spline_degree,
                       const Ref<const VectorXd>& knots,
                       const bool use_input_cost,
                       const bool use_slew_rate,
                       const bool saturate_states) :
    MPCBase(num_states,
            num_inputs,
            num_steps,
            num_controls,
            spline_degree,
            knots,
            use_input_cost,
            use_slew_rate,
            saturate_states),
    // if using a slew rate constraint then state saturation rows of A are
    // shifted down by the number of slew rate constraints
    x_sat_idx_{num_inputs * num_controls +
               num_inputs * (num_controls - 1) * use_slew_rate},
    num_constraints_{x_sat_idx_ + num_states * num_steps * saturate_states},
    S_{num_states * num_steps, num_inputs * num_controls},
    v_{num_states * num_steps}
{
  int num_design_vars{num_inputs * num_controls};
  solver_ = new OSQPSolver{num_design_vars, num_constraints_};
  P_.resize(num_design_vars, num_design_vars);
  A_.resize(num_constraints_, num_design_vars);
  q_.resize(num_design_vars);
  l_.resize(num_constraints_);
  u_.resize(num_constraints_);

  A_.setIdentity(); // input saturation conststraint
  if (use_slew_rate) {
    // shorten block index variables for readability
    int mp{num_inputs * num_controls};
    int mp_m{num_inputs * num_controls - num_inputs};
    A_.block(mp, 0, mp_m, mp).diagonal().setConstant(-1);
    A_.block(mp, num_inputs_, mp_m, mp_m).diagonal().setOnes();
  }
  // Avoids setting first row block of S_ to zero every time solve() is
  // cph::alled (a minor speed optimization)
  S_.topRows(num_states).setZero();
}

// void BSplineMPC::getInputTrajectory(Ref<VectorXd> u_traj) const
// {
//   MPCBase::getInputTrajectory(u_traj); // size checks
//
//   int seg;
//   Map<const MatrixXd> ctrls{solution_map_.data(), num_inputs_,
//   num_ctrl_pts_};
//   VectorXd weights;
//
//   for (int k{0}; k < len_horizon_; ++k) {
//     seg = spline_segment_idxs_(k);
//     u_traj(seqN(k, num_inputs_)) =
//         ctrls(ph::all, seqN(seg, degree_ + 1)) * spline_weights_.col(k);
//   }
// }

void BSplineMPC::getPredictedStateTrajectory(Ref<VectorXd> x_traj) const
{
  MPCBase::getPredictedStateTrajectory(x_traj); // size checks
  x_traj = S_ * solution_map_ + v_;
}

void BSplineMPC::setInputLimits(const Ref<const VectorXd>& u_min,
                                const Ref<const VectorXd>& u_max)
{
  MPCBase::setInputLimits(u_min, u_max); // size checks

  for (int k{0}; k < num_ctrl_pts_; ++k) {
    l_.segment(num_inputs_ * k, num_inputs_) = u_min_;
    u_.segment(num_inputs_ * k, num_inputs_) = u_max_;
  }
  if (solver_initialized_)
    solver_->updateBounds(l_, u_);
}

void BSplineMPC::setStateLimits(const Ref<const VectorXd>& x_min,
                                const Ref<const VectorXd>& x_max)
{
  MPCBase::setStateLimits(x_min, x_max); // size checks

  /* math explained
  x_min_stacked <= x_traj <= x_max_stacked
  x_min_stacked <= S_*z + v_ <= x_max_stacked
  x_min_stacked - v_ <= S_*z <= x_max_stacked - v_
  */

  // A_.block(x_sat_idx_, 0, num_states_ * len_horizon_,
  //          num_inputs_ * num_ctrl_pts_) = S_;
  const auto x_sat_rows = seq(x_sat_idx_, ph::last);
  A_(x_sat_rows, ph::all) = S_;
  for (int k{0}; k < len_horizon_; ++k) {
    l_.segment(x_sat_idx_ + num_states_ * k, num_states_) = x_min_;
    u_.segment(x_sat_idx_ + num_states_ * k, num_states_) = x_max_;
  }
  l_(x_sat_rows) -= v_;
  u_(x_sat_rows) -= v_;
  if (solver_initialized_)
    solver_->updateBounds(l_, u_);
}

void BSplineMPC::setSlewRate(const Ref<const VectorXd>& u_slew)
{
  MPCBase::setSlewRate(u_slew); // size checks

  int mp{num_inputs_ * num_ctrl_pts_};
  int max{num_ctrl_pts_ - 1};
  for (int i{0}; i < max; ++i) {
    l_.segment(mp + num_inputs_ * i, num_inputs_) = -u_slew_;
    u_.segment(mp + num_inputs_ * i, num_inputs_) = u_slew_;
  }
  if (solver_initialized_)
    solver_->updateBounds(l_, u_);
}

void BSplineMPC::convertToQP(const Ref<const VectorXd>& x0)
{
  calcSAndV(x0);
  calcPAndQ();
  solver_->updateCostMatrix(P_);
  solver_->updateCostVector(q_);
  if (saturate_states_) {
    A_.block(x_sat_idx_, 0, S_.rows(), S_.cols()) = S_;
    solver_->updateConstraintMatrix(A_);
  }
}

void BSplineMPC::calcSAndV(const Ref<const VectorXd>& x0)
{
  int n{num_states_};
  int m{num_inputs_};

  // add dynamics of initial time step (k=0)
  VectorXd weights = spline_weights_.col(0);
  for (int i{0}; i < spline_weights_.rows(); ++i)
    S_(seqN(0, n), seqN(m * i, m)) = weights(i) * Bd_;
  // S_(seqN(0, n), seqN(0, m)) = Bd_; // only works if spline is clamped
  v_.head(n) = Ad_ * x0 + wd_;

  int seg;
  for (int k{1}; k < len_horizon_; ++k) {
    seg = spline_segment_idxs_(k);
    weights = spline_weights_.col(k);

    // add cumulative effect of all previous dynamics
    auto rows_cur = seqN(n * k, n);
    auto rows_prev = seqN(n * (k - 1), n);
    S_(rows_cur, ph::all) = Ad_ * S_(rows_prev, ph::all);
    v_(rows_cur) = Ad_ * v_(rows_prev) + wd_;

    // add effect of current input parameterized by spline
    for (int i{0}; i < spline_weights_.rows(); ++i)
      S_(rows_cur, seqN(m * (seg + i), m)) += weights(i) * Bd_;
  }
}

void BSplineMPC::calcPAndQ()
{
  P_ = S_.transpose() * Q_big_ * S_;
  q_ = S_.transpose() * Q_big_ * (v_ - x_goal_);
  if (use_input_cost_) {
    P_ += R_big_;
    q_ -= R_big_ * u_goal_;
  }
}

} // namespace affine_mpc
