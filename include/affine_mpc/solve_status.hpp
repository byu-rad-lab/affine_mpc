#ifndef AFFINE_MPC_SOLVE_STATUS_HPP
#define AFFINE_MPC_SOLVE_STATUS_HPP

/**
 * @file solve_status.hpp
 * @brief Defines SolveStatus enum and related helpers for OSQP solve results.
 */

#include <iosfwd>
#include <osqp_api_constants.h>

namespace affine_mpc {

/**
 * @enum SolveStatus
 * @brief High-level status codes for OSQP solver results.
 *
 * Maps OSQP's integer status codes to a simplified set of categories for MPC
 * usage.
 */
enum class SolveStatus
{
  Success,          ///< QP solved successfully
  NotInitialized,   ///< Solver not initialized
  SolvedInaccurate, ///< QP solved but solution may be inaccurate
  PrimalInfeasible, ///< Problem is primal infeasible
  DualInfeasible,   ///< Problem is dual infeasible
  MaxIterReached,   ///< OSQP hit max iterations
  TimeLimitReached, ///< OSQP hit time limit
  OtherFailure      ///< Any other OSQP error
};

/**
 * @brief Converts an OSQP status code to a SolveStatus value.
 * @param osqp_status Integer status code from OSQP.
 * @return Corresponding SolveStatus value.
 */
inline SolveStatus osqpStatusToSolveStatus(int osqp_status)
{
  switch (osqp_status) {
  case osqp_status_type::OSQP_SOLVED:
    return SolveStatus::Success;
  case osqp_status_type::OSQP_SOLVED_INACCURATE:
    return SolveStatus::SolvedInaccurate;
  case osqp_status_type::OSQP_PRIMAL_INFEASIBLE:
  case osqp_status_type::OSQP_PRIMAL_INFEASIBLE_INACCURATE:
    return SolveStatus::PrimalInfeasible;
  case osqp_status_type::OSQP_DUAL_INFEASIBLE:
  case osqp_status_type::OSQP_DUAL_INFEASIBLE_INACCURATE:
    return SolveStatus::DualInfeasible;
  case osqp_status_type::OSQP_MAX_ITER_REACHED:
    return SolveStatus::MaxIterReached;
  case osqp_status_type::OSQP_TIME_LIMIT_REACHED:
    return SolveStatus::TimeLimitReached;
  case osqp_status_type::OSQP_NON_CVX:
  case osqp_status_type::OSQP_SIGINT:
  case osqp_status_type::OSQP_UNSOLVED:
  default:
    return SolveStatus::OtherFailure;
  }
}

/**
 * @brief Stream output operator for SolveStatus.
 * @param os Output stream.
 * @param status SolveStatus value.
 * @return Reference to the output stream.
 */
std::ostream& operator<<(std::ostream& os, const SolveStatus status);

} // namespace affine_mpc

#endif // AFFINE_MPC_SOLVE_STATUS_HPP
