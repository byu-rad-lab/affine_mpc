#ifndef AFFINE_MPC_CONDENSED_MPC_HPP
#define AFFINE_MPC_CONDENSED_MPC_HPP

#include <Eigen/Core>

#include "affine_mpc/mpc_base.hpp"

/**
 * @file condensed_mpc.hpp
 * @brief Defines the CondensedMPC class (condensed MPC formulation).
 */

namespace affine_mpc {

/**
 * @class CondensedMPC
 * @brief MPC formulation where only parameterization control points are
 * optimization design variables (condensed QP).
 *
 * Eliminates state variables analytically by implicitly wrapping the model into
 * the const function rather than as a constraint , resulting in a smaller dense
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
   * @param param Input parameterization (B-spline, etc.).
   * @param opts Optional features to enable.
   */
  CondensedMPC(const int state_dim,
               const int input_dim,
               const Parameterization& param,
               const Options& opts = Options{});

  /**
   * @brief Construct CondensedMPC with no parameterization (inputs optimized
   *   at every step).
   * @param state_dim State vector dimension.
   * @param input_dim Input vector dimension.
   * @param horizon_steps Number of discrete time steps in the horizon.
   * @param opts Optional features to enable.
   */
  CondensedMPC(const int state_dim,
               const int input_dim,
               const int horizon_steps,
               const Options& opts = Options{});

  virtual ~CondensedMPC() = default;

  /**
   * @brief Get the predicted state trajectory over the horizon.
   * @param x_traj Output vector for state trajectory.
   */
  void getPredictedStateTrajectory(
      Eigen::Ref<Eigen::VectorXd> x_traj) const noexcept override final;

protected: // for testing
  void qpUpdateX0(const Eigen::Ref<const Eigen::VectorXd>& x0) override final;

  void updateS();
  void updateV(const Eigen::Ref<const Eigen::VectorXd>& x0);

  Eigen::MatrixXd S_;
  Eigen::VectorXd v_;

private:
  bool qpUpdateModel() override final;
  bool qpUpdateReferences() override final;
  bool qpUpdateInputLimits() override final;
  bool qpUpdateStateLimits() override final;
  bool qpUpdateSlewRate() override final;

  bool model_changed_;
  bool bounds_changed_;
};

} // namespace affine_mpc

#endif // AFFINE_MPC_CONDENSED_MPC_HPP
