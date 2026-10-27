#ifndef AFFINE_MPC_CONDENSED_MPC_HPP
#define AFFINE_MPC_CONDENSED_MPC_HPP

/**
 * @file condensed_mpc.hpp
 * @brief Defines the deprecated CondensedMPC, the former name of MPC with
 *   Formulation::Condensed.
 */

#include "affine_mpc/mpc.hpp"
#include "affine_mpc/mpc_base.hpp"
#include "affine_mpc/options.hpp"
#include "affine_mpc/parameterization.hpp"

namespace affine_mpc {

/**
 * @class CondensedMPC
 * @brief MPC with Formulation::Condensed.
 * @deprecated Use MPC, whose formulation defaults to Formulation::Condensed.
 *   CondensedMPC will be removed in 0.5.0.
 */
class [[deprecated("use MPC with Formulation::Condensed")]] CondensedMPC final
    : public MPC
{
public:
  /// @brief Same as MPC(state_dim, input_dim, param, opts,
  ///   Formulation::Condensed).
  CondensedMPC(int state_dim,
               int input_dim,
               const Parameterization& param,
               const Options& opts = Options{}) :
      MPC(state_dim, input_dim, param, opts, Formulation::Condensed)
  {}

  /// @brief Same as MPC(state_dim, input_dim, horizon_steps, opts,
  ///   Formulation::Condensed).
  CondensedMPC(int state_dim,
               int input_dim,
               int horizon_steps,
               const Options& opts = Options{}) :
      MPC(state_dim, input_dim, horizon_steps, opts, Formulation::Condensed)
  {}

  const char* getClassName() const noexcept override { return "CondensedMPC"; }
};

} // namespace affine_mpc

#endif // AFFINE_MPC_CONDENSED_MPC_HPP
