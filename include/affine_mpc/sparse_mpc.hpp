#ifndef AFFINE_MPC_SPARSE_MPC_HPP
#define AFFINE_MPC_SPARSE_MPC_HPP

/**
 * @file sparse_mpc.hpp
 * @brief Defines the SparseMPC class (sparse MPC formulation).
 */

#include <Eigen/Core>

#include "affine_mpc/mpc_base.hpp"
#include "affine_mpc/options.hpp"
#include "affine_mpc/parameterization.hpp"

namespace affine_mpc {

/**
 * @class SparseMPC
 * @brief MPC formulation where the predicted state trajectory and
 * parameterization control points are both optimization design variables
 * (sparse QP).
 *
 * Retains state and input variables with model as an optimization constraint,
 * exploiting sparsity for scalability. Preferred for longer horizons or larger
 * state dimensions. Converts the MPC problem to QP form for OSQP, using input
 * parameterization.
 */
class SparseMPC : public MPCBase
{
public:
  /**
   * @brief Construct SparseMPC with specified input parameterization and MPC
   *   configuration options.
   * @param state_dim State vector dimension.
   * @param input_dim Input vector dimension.
   * @param param Input trajectory parameterization.
   * @param opts Optional MPC configuration features to enable.
   */
  SparseMPC(int state_dim,
            int input_dim,
            const Parameterization& param,
            const Options& opts = Options{});

  /**
   * @brief Construct SparseMPC with no parameterization (full input trajectory
   *   will be optimized) and MPC configuration options.
   * @param state_dim State vector dimension.
   * @param input_dim Input vector dimension.
   * @param horizon_steps Number of discrete time steps in the horizon.
   * @param opts Optional MPC configuration features to enable.
   */
  SparseMPC(int state_dim,
            int input_dim,
            int horizon_steps,
            const Options& opts = Options{});

  ~SparseMPC() override = default;

  const char* getClassName() const noexcept override { return "SparseMPC"; }
};

} // namespace affine_mpc

#endif // AFFINE_MPC_SPARSE_MPC_HPP
