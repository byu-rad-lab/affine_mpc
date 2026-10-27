#ifndef AFFINE_MPC_MPC_BASE_HPP
#define AFFINE_MPC_MPC_BASE_HPP

/**
 * @file mpc_base.hpp
 * @brief Defines the deprecated MPCBase, the former name of MPC (include
 *   affine_mpc/mpc.hpp instead).
 */

#include "affine_mpc/mpc.hpp"

namespace affine_mpc {

/**
 * @brief Former name of MPC.
 * @deprecated Use MPC. MPCBase will be removed in 0.5.0.
 */
using MPCBase [[deprecated("use MPC")]] = MPC;

} // namespace affine_mpc

#endif // AFFINE_MPC_MPC_BASE_HPP
