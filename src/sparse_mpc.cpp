#include "affine_mpc/sparse_mpc.hpp"

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


SparseMPC::SparseMPC(const int state_dim,
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
    // x_sat_idx is the starting row index of the state saturation constraints
    // constraint ordering:
    //   - model (always present in sparse MPC)
    //   - input saturation (always present)
    //   - slew rate constraint (if enabled)
    //   - state saturation constraints (if enabled)
    x_sat_idx_{x_traj_dim_  // model constraints
               + ctrls_dim_ // input bound constraints
               + (ctrls_dim_ - input_dim) * use_slew_rate},
    num_constraints_{x_sat_idx_ + x_traj_dim_ * saturate_states},
    refs_changed_{true} // ensure q_ updates on first solve
{
  const int num_design_vars{ctrls_dim_ + x_traj_dim_};
  solver_ = std::make_unique<OSQPSolver>(num_design_vars, num_constraints_);
  P_.resize(num_design_vars, num_design_vars);
  A_.resize(num_constraints_, num_design_vars);
  q_.resize(num_design_vars);
  l_.resize(num_constraints_);
  u_.resize(num_constraints_);

  if (use_input_cost_) {
    P_.setIdentity();
    q_.setConstant(-1.0);
  } else {
    P_.setZero();
    P_.diagonal().tail(x_traj_dim_).setOnes();
    q_.head(ctrls_dim_).setZero();
    q_.tail(x_traj_dim_).setConstant(-1.0);
  }

  const auto ctrls_seq{seqN(0, ctrls_dim_)};
  const auto x_traj_seq{ph::lastN(x_traj_dim_)};

  A_.setZero();

  // Model constraints: -I on x_traj block diagonal
  A_(seqN(0, x_traj_dim_), x_traj_seq).diagonal().setConstant(-1.0);

  // Control point saturation constraint: I on controls block
  A_(seqN(x_traj_dim_, ctrls_dim_), ctrls_seq).setIdentity();

  if (use_slew_rate) {
    const int slew_idx{x_traj_dim_ + ctrls_dim_};
    const auto slew_rows{seqN(slew_idx, ctrls_dim_ - input_dim)};
    A_(slew_rows, ph::all).diagonal().setConstant(-1.0);
    A_(slew_rows, seq(input_dim, ph::last)).diagonal().setOnes();
  }
  if (saturate_states_)
    A_(x_traj_seq, x_traj_seq).setIdentity();
}

void SparseMPC::getPredictedStateTrajectory(Ref<VectorXd> x_traj) const noexcept
{
  MPCBase::getPredictedStateTrajectory(x_traj); // size checks
  x_traj = solution_map_.tail(x_traj_dim_);
}

bool SparseMPC::setModelDiscrete(const Ref<const MatrixXd>& Ad,
                                 const Ref<const MatrixXd>& Bd,
                                 const Ref<const VectorXd>& wd)
{
  MPCBase::setModelDiscrete(Ad, Bd, wd);
  return qpUpdateModel();
}

bool SparseMPC::setModelContinuous2Discrete(const Ref<const MatrixXd>& Ac,
                                            const Ref<const MatrixXd>& Bc,
                                            const Ref<const VectorXd>& wc,
                                            double dt,
                                            double tol)
{
  MPCBase::setModelContinuous2Discrete(Ac, Bc, wc, dt, tol);
  return qpUpdateModel();
}

void SparseMPC::setReferenceState(const Ref<const VectorXd>& x_step)
{
  MPCBase::setReferenceState(x_step);
  refs_changed_ = true;
}

void SparseMPC::setReferenceInput(const Ref<const VectorXd>& u_step)
{
  MPCBase::setReferenceInput(u_step);
  refs_changed_ = true;
}

void SparseMPC::setReferenceStateTrajectory(const Ref<const VectorXd>& x_traj)
{
  MPCBase::setReferenceStateTrajectory(x_traj);
  refs_changed_ = true;
}

void SparseMPC::setReferenceParameterizedInputTrajectory(
    const Ref<const VectorXd>& u_traj_ctrl_pts)
{
  MPCBase::setReferenceParameterizedInputTrajectory(u_traj_ctrl_pts);
  refs_changed_ = true;
}

void SparseMPC::setInputLimits(const Ref<const VectorXd>& u_min,
                               const Ref<const VectorXd>& u_max)
{
  MPCBase::setInputLimits(u_min, u_max);

  for (Index k{0}, offset{x_traj_dim_}; k < num_ctrl_pts_;
       ++k, offset += input_dim_) {
    l_.segment(offset, input_dim_) = u_min_;
    u_.segment(offset, input_dim_) = u_max_;
  }
}

void SparseMPC::setStateLimits(const Ref<const VectorXd>& x_min,
                               const Ref<const VectorXd>& x_max)
{
  MPCBase::setStateLimits(x_min, x_max); // size checks

  for (Index k{0}, offset{x_sat_idx_}; k < horizon_steps_;
       ++k, offset += state_dim_) {
    l_.segment(offset, state_dim_) = x_min_;
    u_.segment(offset, state_dim_) = x_max_;
  }
}

void SparseMPC::setSlewRate(const Ref<const VectorXd>& u_slew)
{
  MPCBase::setSlewRate(u_slew); // size checks

  const int slew_idx{x_traj_dim_ + ctrls_dim_};
  const auto neg_slew = (-u_slew_).eval();
  for (int i = 0, offset = slew_idx; i < num_ctrl_pts_ - 1;
       ++i, offset += input_dim_) {
    l_.segment(offset, input_dim_) = neg_slew;
    u_.segment(offset, input_dim_) = u_slew_;
  }
}

void SparseMPC::updateQP(const Ref<const VectorXd>& x0)
{
  bool success{true};
  l_.head(state_dim_).noalias() = -(Ad_ * x0 + wd_);
  u_.head(state_dim_) = l_.head(state_dim_);
  success &= solver_->updateBounds(l_, u_);

  if (weights_changed_) {
    refs_changed_ = false;
    weights_changed_ = false;
    calcBothCostTerms();
    success &= solver_->updateCostMatrix(P_);
    success &= solver_->updateCostVector(q_);
  } else if (refs_changed_) {
    refs_changed_ = false;
    calcCostVector();
    success &= solver_->updateCostVector(q_);
  }

  // Note: success is used to avoid build warnings. Failures will manifest in
  // either initializeSolver() or solve(), no need to check on protected
  // method
}

void SparseMPC::calcBothCostTerms()
{
  const int x_rows{x_traj_dim_};
  P_.diagonal().tail(x_rows) = Q_big_.diagonal();
  q_.tail(x_rows).noalias() = -(Q_big_ * x_goal_);

  if (use_input_cost_) {
    const int u_rows{ctrls_dim_};
    P_.diagonal().head(u_rows) = R_big_.diagonal();
    q_.head(u_rows).noalias() = -(R_big_ * u_goal_);
  }
}

void SparseMPC::calcCostVector()
{
  q_.tail(x_traj_dim_).noalias() = -(Q_big_ * x_goal_);
  if (use_input_cost_) {
    q_.head(ctrls_dim_).noalias() = -(R_big_ * u_goal_);
  }
}

bool SparseMPC::qpUpdateModel()
{
  // Don't need to update 1st state block since it must be updated every solve
  // with x0 anyway. Similarly, don't need to update l/u with solver
  const auto neg_wd = (-wd_).eval();
  for (Index k{1}, offset{state_dim_}; k < horizon_steps_;
       ++k, offset += state_dim_) {
    l_.segment(offset, state_dim_) = neg_wd;
    u_.segment(offset, state_dim_) = neg_wd;
  }

  // set model constraints that multiply controls
  const int num_weights{spline_degree_ + 1};
  for (int k{0}, row{0}; k < horizon_steps_; ++k, row += state_dim_) {
    for (int i{0}, col{input_dim_ * spline_segment_idxs_(k)}; i < num_weights;
         ++i, col += input_dim_) {
      A_.block(row, col, state_dim_, input_dim_) = spline_weights_(i, k) * Bd_;
    }
  }

  // set model constraints that multiply states
  for (int k{1}, row{state_dim_}, col{ctrls_dim_}; k < horizon_steps_;
       ++k, row += state_dim_, col += state_dim_) {
    A_.block(row, col, state_dim_, state_dim_) = Ad_;
  }

  if (!solver_initialized_)
    return true;
  return solver_->updateConstraintMatrix(A_);
}

} // namespace affine_mpc
