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

  ~CondensedMPC() override = default;

  const char* getClassName() const noexcept override { return "CondensedMPC"; }
};

} // namespace affine_mpc

#endif // AFFINE_MPC_CONDENSED_MPC_HPP
