#include "affine_mpc/condensed_mpc.hpp"

#include <Eigen/Core>

#include "affine_mpc/mpc_base.hpp"
#include "affine_mpc/osqp_solver.hpp"
#include "eigen_compat.hpp" // remove this once Eigen 3.5 is required

using namespace Eigen;
// revert back to this once Eigen 3.5 is required
// namespace ph = Eigen::placeholders;

namespace affine_mpc {


CondensedMPC::CondensedMPC(int state_dim,
                           int input_dim,
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
    weighted_state_err_{x_traj_dim_},
    weighted_S_{x_traj_dim_, ctrls_dim_}
{
  // Avoids setting first row block of S_ to zero every time solve() is
  // called (a minor speed optimization)
  S_.topRows(state_dim).setZero();
}

CondensedMPC::CondensedMPC(int state_dim,
                           int input_dim,
                           int horizon_steps,
                           const Options& opts) :
    CondensedMPC(state_dim,
                 input_dim,
                 Parameterization::moveBlocking(horizon_steps, horizon_steps),
                 opts)
{
  // nothing to do, delegating to main constructor with no parameterization
}

void CondensedMPC::getPredictedStateTrajectory(Ref<VectorXd> x_traj) const
{
  MPCBase::getPredictedStateTrajectory(x_traj); // size checks
  x_traj.noalias() = S_ * solution_map_;
  x_traj += v_;
}

DataChangeSet CondensedMPC::qpApplyChanges(const DataChangeSet changes)
{
  const MPCData& d{data()};
  DataChangeSet pending{};

  if (changes.has(DataChange::Model))
    updateS();

  if (changes.has(DataChange::Model | DataChange::Weights)) {
    // Eigen's matrix product still heap-allocates its blocking buffers when
    // they exceed EIGEN_STACK_ALLOCATION_LIMIT (128 KB), i.e. for large
    // problems after a model or weight change.
    // Products with a DiagonalMatrix copy it (a heap allocation) in Eigen 3.4,
    // so the weights are applied through diagonal() views instead.
    weighted_S_.noalias() = d.Q_big.diagonal().asDiagonal() * S_;
    P_.noalias() = S_.transpose() * weighted_S_;
    if (opts_.use_input_cost)
      P_ += d.R_big;
    if (opts_.saturate_states)
      A_.bottomRows(S_.rows()) = S_;

    // Before initializeSolver() the QP is only assembled. When A also changes,
    // one combined update rescales and refactors once instead of twice.
    if (isSolverInitialized()) {
      const bool updated{opts_.saturate_states ? solver_->updateMatrices(P_, A_)
                                               : solver_->updateCostMatrix(P_)};
      if (!updated)
        pending |= changes & (DataChange::Model | DataChange::Weights);
    }
  }

  // MPCBase wrote the limit and slew rows. With state saturation, qpApplyX0()
  // sends the bounds at every solve anyway.
  if (!opts_.saturate_states && isSolverInitialized()
      && changes.has(DataChange::Limits | DataChange::PreviousInput)) {
    if (!solver_->updateBounds(l_, u_))
      pending |= changes & (DataChange::Limits | DataChange::PreviousInput);
  }

  // The model offset and references only enter v_ and q_, which qpApplyX0()
  // rebuilds at every solve
  return pending;
}

bool CondensedMPC::qpApplyX0(const Ref<const VectorXd>& x0)
{
  const MPCData& d{data()};
  updateV(x0);

  bool success{true};
  if (opts_.saturate_states) {
    l_.tail(x_traj_dim_) = d.x_min.replicate(horizon_steps_, 1) - v_;
    u_.tail(x_traj_dim_) = d.x_max.replicate(horizon_steps_, 1) - v_;
  }

  weighted_state_err_.noalias() = d.Q_big.diagonal().cwiseProduct(v_ - d.x_ref);
  q_.noalias() = S_.transpose() * weighted_state_err_;
  if (opts_.use_input_cost)
    q_.noalias() -= d.R_big.diagonal().cwiseProduct(d.ctrls_ref);
  success &= solver_->updateCostVector(q_);

  if (opts_.saturate_states)
    success &= solver_->updateBounds(l_, u_);
  return success;
}

void CondensedMPC::updateS()
{
  const Parameterization& param{input_param_.parameterization()};
  const MatrixXd& Ad{data().Ad};
  const MatrixXd& Bd{data().Bd};

  // add dynamics of initial time step (k=0)
  for (int i{0}; i < param.band_width; ++i) {
    const int col{input_param_.coefficientIndex(0, param.band_start(0) + i)};
    S_.block(0, col, state_dim_, input_dim_) = param.band_weights(i, 0) * Bd;
  }

  for (int k{1}, row{state_dim_}; k < horizon_steps_; ++k, row += state_dim_) {
    // add cumulative effect of all previous dynamics
    S_.middleRows(row, state_dim_).noalias() =
        Ad * S_.middleRows(row - state_dim_, state_dim_);

    // add effect of current input parameterized by the basis
    for (int i{0}; i < param.band_width; ++i) {
      const int col{input_param_.coefficientIndex(0, param.band_start(k) + i)};
      S_.block(row, col, state_dim_, input_dim_) +=
          param.band_weights(i, k) * Bd;
    }
  }
}

void CondensedMPC::updateV(const Ref<const VectorXd>& x0)
{
  const MatrixXd& Ad{data().Ad};
  const VectorXd& wd{data().wd};
  // add dynamics of initial time step (k=0). Adding wd separately avoids a
  // temporary for the product inside a sum.
  v_.head(state_dim_).noalias() = Ad * x0;
  v_.head(state_dim_) += wd;

  for (int k{1}, row{state_dim_}; k < horizon_steps_; ++k, row += state_dim_) {
    // add cumulative effect of all previous dynamics
    v_.segment(row, state_dim_).noalias() =
        Ad * v_.segment(row - state_dim_, state_dim_);
    v_.segment(row, state_dim_) += wd;
  }
}

} // namespace affine_mpc
