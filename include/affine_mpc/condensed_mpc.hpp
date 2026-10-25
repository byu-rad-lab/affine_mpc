#ifndef AFFINE_MPC_CONDENSED_MPC_HPP
#define AFFINE_MPC_CONDENSED_MPC_HPP

/**
 * @file condensed_mpc.hpp
 * @brief Defines the CondensedMPC class (condensed MPC formulation).
 */

#include <Eigen/Core>

#include "affine_mpc/mpc_base.hpp"

namespace affine_mpc {

/**
 * @class CondensedMPC
 * @brief MPC formulation where only parameterization control points are
 * optimization design variables (condensed QP).
 *
 * Eliminates state variables analytically by implicitly wrapping the model into
 * the cost function rather than as a constraint, resulting in a smaller dense
 * QP. Preferred for shorter horizons and lower-dimensional problems.
 * Converts the MPC problem to QP form for OSQP, using input parameterization.
 */
class CondensedMPC : public MPCBase
{
public:
  /**
   * @brief Construct CondensedMPC with specified input parameterization and
   *   MPC configuration options.
   * @param state_dim State vector dimension.
   * @param input_dim Input vector dimension.
   * @param param Input trajectory parameterization.
   * @param opts Optional MPC configuration features to enable.
   */
  CondensedMPC(int state_dim,
               int input_dim,
               const Parameterization& param,
               const Options& opts = Options{});

  /**
   * @brief Construct CondensedMPC with no parameterization (full input
   *   trajectory will be optimized) and MPC configuration options.
   * @param state_dim State vector dimension.
   * @param input_dim Input vector dimension.
   * @param horizon_steps Number of discrete time steps in the horizon.
   * @param opts Optional MPC configuration features to enable.
   */
  CondensedMPC(int state_dim,
               int input_dim,
               int horizon_steps,
               const Options& opts = Options{});

  virtual ~CondensedMPC() = default;

  /**
   * @brief Get the predicted state trajectory over the horizon.
   * @param x_traj Output vector for state trajectory.
   */
  void getPredictedStateTrajectory(
      Eigen::Ref<Eigen::VectorXd> x_traj) const override final;

  const char* getClassName() const noexcept override { return "CondensedMPC"; }

protected: // for testing
  void updateS();
  void updateV(const Eigen::Ref<const Eigen::VectorXd>& x0);

  Eigen::MatrixXd S_;
  Eigen::VectorXd v_;

private:
  DataChangeSet qpApplyChanges(DataChangeSet changes) override final;
  bool qpApplyX0(const Eigen::Ref<const Eigen::VectorXd>& x0) override final;

  Eigen::VectorXd weighted_state_err_; // Q_big * (v_ - x_ref), avoids a temp
  Eigen::MatrixXd weighted_S_;         // Q_big * S_, avoids a temp for P_
};

} // namespace affine_mpc

#endif // AFFINE_MPC_CONDENSED_MPC_HPP
