#include "affine_mpc/sparse_mpc.hpp"

#include <Eigen/Core>

#include "affine_mpc/mpc_base.hpp"
#include "affine_mpc/osqp_solver.hpp"
#include "eigen_compat.hpp" // remove this once Eigen 3.5 is required

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
            state_dim * param.horizon_steps)
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

void SparseMPC::getPredictedStateTrajectory(Ref<VectorXd> x_traj) const
{
  MPCBase::getPredictedStateTrajectory(x_traj); // size checks
  x_traj = solution_map_.tail(x_traj_dim_);
}

SparseMPC::DataChangeSet SparseMPC::qpApplyChanges(const DataChangeSet changes)
{
  DataChangeSet pending{};

  // dynamics-row bounds of steps k >= 1 (sent with the x0 rows at every solve)
  if (changes.has(DataChange::ModelOffset))
    setDynamicsOffsetBounds();

  if (changes.has(DataChange::Model)) {
    setDynamicsMatrixBlocks();
    if (isSolverInitialized() && !solver_->updateConstraintMatrix(A_))
      pending |= DataChange::Model;
  }

  if (changes.has(DataChange::Weights)) {
    calcBothCostTerms();
    if (!isSolverInitialized()) {
      // Assembling for initializeSolver(): store every weight position on P's
      // diagonal so a weight that is zero now can become nonzero later. This
      // costs nothing in solves since OSQP's KKT system always includes P's
      // diagonal. Weights stay pending, so the first update after
      // initialization applies the actual weights.
      P_.diagonal().tail(x_traj_dim_).setOnes();
      if (opts_.use_input_cost)
        P_.diagonal().head(ctrls_dim_).setOnes();
      pending |= DataChange::Weights;
    } else {
      bool updated{solver_->updateCostMatrix(P_)};
      updated &= solver_->updateCostVector(q_);
      if (!updated)
        pending |= changes & (DataChange::Weights | DataChange::References);
    }
  } else if (changes.has(DataChange::References)) {
    calcCostVector();
    if (isSolverInitialized() && !solver_->updateCostVector(q_))
      pending |= DataChange::References;
  }

  // MPCBase wrote the limit, slew, and state rows, which qpApplyX0() sends
  return pending;
}

bool SparseMPC::qpApplyX0(const Ref<const VectorXd>& x0)
{
  const MPCData& d{data()};
  // separate statements avoid a temporary for the product inside a sum
  l_.head(state_dim_) = -d.wd;
  l_.head(state_dim_).noalias() -= d.Ad * x0;
  u_.head(state_dim_) = l_.head(state_dim_);
  return solver_->updateBounds(l_, u_);
}

void SparseMPC::setDynamicsMatrixBlocks()
{
  const MPCData& d{data()};

  // set model constraints that multiply controls
  const Parameterization& param{input_param_.parameterization()};
  for (int k{0}, row{0}; k < horizon_steps_; ++k, row += state_dim_) {
    for (int i{0}; i < param.band_width; ++i) {
      const int col{input_param_.coefficientIndex(0, param.band_start(k) + i)};
      A_.block(row, col, state_dim_, input_dim_) =
          param.band_weights(i, k) * d.Bd;
    }
  }

  // set model constraints that multiply states
  for (int k{1}, row{state_dim_}, col{ctrls_dim_}; k < horizon_steps_;
       ++k, row += state_dim_, col += state_dim_) {
    A_.block(row, col, state_dim_, state_dim_) = d.Ad;
  }
}

void SparseMPC::setDynamicsOffsetBounds()
{
  // Don't need to update 1st state block since it must be updated every solve
  // with x0 anyway. Similarly, don't need to update l/u with solver
  const VectorXd& wd{data().wd};
  for (Index k{1}, offset{state_dim_}; k < horizon_steps_;
       ++k, offset += state_dim_) {
    l_.segment(offset, state_dim_) = -wd;
    u_.segment(offset, state_dim_) = -wd;
  }
}

void SparseMPC::calcBothCostTerms()
{
  const MPCData& d{data()};
  P_.diagonal().tail(x_traj_dim_) = d.Q_big.diagonal();
  if (opts_.use_input_cost)
    P_.diagonal().head(ctrls_dim_) = d.R_big.diagonal();
  calcCostVector();
}

void SparseMPC::calcCostVector()
{
  const MPCData& d{data()};
  // Products with a DiagonalMatrix copy it (a heap allocation) in Eigen 3.4
  q_.tail(x_traj_dim_).noalias() = -d.Q_big.diagonal().cwiseProduct(d.x_ref);
  if (opts_.use_input_cost)
    q_.head(ctrls_dim_).noalias() =
        -d.R_big.diagonal().cwiseProduct(d.ctrls_ref);
}

} // namespace affine_mpc
