/**
 * @file solve_status.cpp
 * @brief This file was specifically created to implement the operator<<
 *   overload for SolveStatus without including the full <ostream> header in
 *   solve_status.hpp.
 */

#include "affine_mpc/solve_status.hpp"

#include <ostream>

namespace affine_mpc {

namespace {

const char* toString(const SolveStatus status)
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
  default:
    return "OtherFailure";
  }
}

} // namespace

std::ostream& operator<<(std::ostream& os, const SolveStatus status)
{
  os << toString(status);
  return os;
}

} // namespace affine_mpc
