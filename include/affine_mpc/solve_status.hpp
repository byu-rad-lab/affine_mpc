#ifndef AFFINE_MPC_SOLVE_STATUS_HPP
#define AFFINE_MPC_SOLVE_STATUS_HPP

#include <osqp_api_constants.h>
#include <ostream>

/**
 * @file solve_status.hpp
 * @brief Defines SolveStatus enum and related helpers for OSQP solve results.
 */

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
inline SolveStatus osqpStatusToSolveStatus(const int osqp_status)
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
 * @brief Converts a SolveStatus value to a human-readable string.
 * @param status SolveStatus value.
 * @return String representation of the status.
 */
inline const char* toString(const SolveStatus status)
{
  switch (status) {
  case SolveStatus::Success:
    return "Success";
  case SolveStatus::NotInitialized:
    return "NotInitialized";
  case SolveStatus::SolvedInaccurate:
    return "SolvedInaccurate";
  case SolveStatus::PrimalInfeasible:
    return "PrimalInfeasible";
  case SolveStatus::DualInfeasible:
    return "DualInfeasible";
  case SolveStatus::MaxIterReached:
    return "MaxIterReached";
  case SolveStatus::TimeLimitReached:
    return "TimeLimitReached";
  case SolveStatus::OtherFailure:
    return "OtherFailure";
  default:
    return "Unknown";
  }
}

/**
 * @brief Stream output operator for SolveStatus.
 * @param os Output stream.
 * @param status SolveStatus value.
 * @return Reference to the output stream.
 */
inline std::ostream& operator<<(std::ostream& os, const SolveStatus status)
{
  os << toString(status);
  return os;
}

} // namespace affine_mpc

#endif // AFFINE_MPC_SOLVE_STATUS_HPP
