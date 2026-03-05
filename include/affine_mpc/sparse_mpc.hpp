#ifndef AFFINE_MPC_SPARSE_MPC_HPP
#define AFFINE_MPC_SPARSE_MPC_HPP

#include <Eigen/Core>

#include "affine_mpc/mpc_base.hpp"
#include "affine_mpc/options.hpp"
#include "affine_mpc/parameterization.hpp"

/**
 * @file sparse_mpc.hpp
 * @brief Defines the SparseMPC class (sparse MPC formulation).
 */

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
   * @brief Construct SparseMPC with custom parameterization.
   * @param state_dim State vector dimension.
   * @param input_dim Input vector dimension.
   * @param param Input parameterization (B-spline, etc.).
   * @param opts Optional features to enable.
   */
  SparseMPC(const int state_dim,
            const int input_dim,
            const Parameterization& param,
            const Options& opts = Options{});

  /**
   * @brief Construct SparseMPC with default move-blocking parameterization.
   * @param state_dim State vector dimension.
   * @param input_dim Input vector dimension.
   * @param horizon_steps Number of discrete time steps in the horizon.
   * @param opts Optional features to enable.
   */
  SparseMPC(const int state_dim,
            const int input_dim,
            const int horizon_steps,
            const Options& opts = Options{});

  virtual ~SparseMPC() = default;

  /**
   * @brief Get the predicted state trajectory over the horizon.
   * @param x_traj Output vector for state trajectory.
   */
  void getPredictedStateTrajectory(
      Eigen::Ref<Eigen::VectorXd> x_traj) const noexcept override final;

protected: // for testing
  void qpUpdateX0(const Eigen::Ref<const Eigen::VectorXd>& x0) override final;

private:
  bool qpUpdateModel() override final;
  bool qpUpdateReferences() override final;
  bool qpUpdateInputLimits() override final;
  bool qpUpdateStateLimits() override final;
  bool qpUpdateSlewRate() override final;

  void calcBothCostTerms();
  void calcCostVector();

  bool refs_changed_;
};

} // namespace affine_mpc

#endif // AFFINE_MPC_SPARSE_MPC_HPP
