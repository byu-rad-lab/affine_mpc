#ifndef AFFINE_MPC_SPARSE_MPC_HPP
#define AFFINE_MPC_SPARSE_MPC_HPP

/**
 * @file sparse_mpc.hpp
 * @brief Defines SparseMPC, the former name of MPC with Formulation::Sparse.
 */

#include "affine_mpc/mpc.hpp"
#include "affine_mpc/mpc_base.hpp"
#include "affine_mpc/options.hpp"
#include "affine_mpc/parameterization.hpp"

namespace affine_mpc {

/**
 * @class SparseMPC
 * @brief MPC with Formulation::Sparse; use MPC instead.
 */
class SparseMPC : public MPC
{
public:
  /// @brief Same as MPC(state_dim, input_dim, param, opts,
  ///   Formulation::Sparse).
  SparseMPC(int state_dim,
            int input_dim,
            const Parameterization& param,
            const Options& opts = Options{}) :
      MPC(state_dim, input_dim, param, opts, Formulation::Sparse)
  {}

  /// @brief Same as MPC(state_dim, input_dim, horizon_steps, opts,
  ///   Formulation::Sparse).
  SparseMPC(int state_dim,
            int input_dim,
            int horizon_steps,
            const Options& opts = Options{}) :
      MPC(state_dim, input_dim, horizon_steps, opts, Formulation::Sparse)
  {}

  const char* getClassName() const noexcept override { return "SparseMPC"; }
};

} // namespace affine_mpc

#endif // AFFINE_MPC_SPARSE_MPC_HPP
