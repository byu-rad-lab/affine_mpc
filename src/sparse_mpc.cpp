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


SparseMPC::SparseMPC(int state_dim,
                     int input_dim,
                     const Parameterization& param,
                     const Options& opts) :
    MPCBase(state_dim,
            input_dim,
            param,
            opts,
            // num_design_vars
            input_dim * param.num_control_points
                + state_dim * param.horizon_steps,
            // num_custom_constraints (model)
            state_dim * param.horizon_steps),
    refs_changed_{true} // ensure q_ updates on first solve
{
  if (opts_.use_input_cost) {
    P_.setIdentity();
    q_.setConstant(-1.0);
  } else {
    P_.setZero();
    P_.diagonal().tail(x_traj_dim_).setOnes();
    q_.head(ctrls_dim_).setZero();
    q_.tail(x_traj_dim_).setConstant(-1.0);
  }

  const auto x_traj_seq{ph::lastN(x_traj_dim_)};

  // Model constraints: -I on x_traj block diagonal
  A_(seqN(0, x_traj_dim_), x_traj_seq).diagonal().setConstant(-1.0);
  if (opts_.saturate_states)
    A_(x_traj_seq, x_traj_seq).setIdentity();
}

SparseMPC::SparseMPC(int state_dim,
                     int input_dim,
                     int horizon_steps,
                     const Options& opts) :
    SparseMPC(state_dim,
              input_dim,
              Parameterization::moveBlocking(horizon_steps, horizon_steps),
              opts)
{
  // nothing to do, delegating to main constructor with no parameterization
}

void SparseMPC::getPredictedStateTrajectory(Ref<VectorXd> x_traj) const noexcept
{
  MPCBase::getPredictedStateTrajectory(x_traj); // size checks
  x_traj = solution_map_.tail(x_traj_dim_);
}

void SparseMPC::qpUpdateX0(const Ref<const VectorXd>& x0)
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

bool SparseMPC::qpUpdateReferences()
{
  // refs only affect q, but weights also affect q, so this just tells the
  // updateQP() method to update q without updating P.
  refs_changed_ = true;
  return true;
}

bool SparseMPC::qpUpdateInputLimits()
{
  // Don't need to update solver bounds because they are update every solve
  // already
  return true;
}

bool SparseMPC::qpUpdateStateLimits()
{
  // Don't need to update solver bounds because they are update every solve
  // already
  return true;
}

bool SparseMPC::qpUpdateSlewRate()
{
  // Don't need to update solver bounds because they are update every solve
  // already
  return true;
}

void SparseMPC::calcBothCostTerms()
{
  const int x_rows{x_traj_dim_};
  P_.diagonal().tail(x_rows) = Q_big_.diagonal();
  q_.tail(x_rows).noalias() = -(Q_big_ * x_ref_);

  if (opts_.use_input_cost) {
    const int u_rows{ctrls_dim_};
    P_.diagonal().head(u_rows) = R_big_.diagonal();
    q_.head(u_rows).noalias() = -(R_big_ * ctrls_ref_);
  }
}

void SparseMPC::calcCostVector()
{
  q_.tail(x_traj_dim_).noalias() = -(Q_big_ * x_ref_);
  if (opts_.use_input_cost) {
    q_.head(ctrls_dim_).noalias() = -(R_big_ * ctrls_ref_);
  }
}

} // namespace affine_mpc
