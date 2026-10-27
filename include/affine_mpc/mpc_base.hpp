#ifndef AFFINE_MPC_MPC_BASE_HPP
#define AFFINE_MPC_MPC_BASE_HPP

/**
 * @file mpc_base.hpp
 * @brief Compatibility header: MPCBase is an alias of MPC (include
 *   affine_mpc/mpc.hpp instead).
 */

#include "affine_mpc/mpc.hpp"

namespace affine_mpc {

/// @brief Former name of MPC.
using MPCBase = MPC;

} // namespace affine_mpc

#endif // AFFINE_MPC_MPC_BASE_HPP
