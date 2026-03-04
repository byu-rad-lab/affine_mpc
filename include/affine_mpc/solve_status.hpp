#ifndef AFFINE_MPC_SOLVE_STATUS_HPP
#define AFFINE_MPC_SOLVE_STATUS_HPP

#include <osqp_api_constants.h>
#include <ostream>

namespace affine_mpc {

enum class SolveStatus
{
  Success,          // QP solved successfully
  NotInitialized,   // Solver not initialized
  SolvedInaccurate, // QP solved but solution may be inaccurate
  PrimalInfeasible, // Problem is primal infeasible
  DualInfeasible,   // Problem is dual infeasible
  MaxIterReached,   // OSQP hit max iterations
  TimeLimitReached, // OSQP hit time limit
  OtherFailure      // Any other OSQP error
};

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

// Optional: helper function to convert status to string
inline const char* toString(SolveStatus status)
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

inline std::ostream& operator<<(std::ostream& os, SolveStatus status)
{
  os << toString(status);
  return os;
}

} // namespace affine_mpc

#endif // AFFINE_MPC_SOLVE_STATUS_HPP
