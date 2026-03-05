#ifndef AFFINE_MPC_HPP
#define AFFINE_MPC_HPP

/**
 * @file affine_mpc.hpp
 * @brief Top-level include for the affine_mpc library.
 *
 * Includes all public headers required for Model Predictive Control
 * using affine time-invariant models and OSQP solver integration.
 *
 * This header provides access to:
 *  - Options configuration
 *  - Input parameterization (B-splines, move-blocking, etc.)
 *  - Solve status codes
 *  - Core MPC classes (CondensedMPC, SparseMPC)
 *  - Logging utilities
 */

#include "affine_mpc/options.hpp"
#include "affine_mpc/parameterization.hpp"
#include "affine_mpc/solve_status.hpp"

#include "affine_mpc/mpc_base.hpp"
#include "affine_mpc/mpc_logger.hpp"

#include "affine_mpc/condensed_mpc.hpp"
#include "affine_mpc/sparse_mpc.hpp"

#endif // AFFINE_MPC_HPP
