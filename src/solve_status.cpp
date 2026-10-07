/**
 * @file solve_status.cpp
 * @brief This file was specifically created to implement the operator<<
 *   overloads for SolveStatus and SolveInfo without including the full
 * <ostream> header in solve_status.hpp.
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
  case SolveStatus::UpdateFailed:
    return "UpdateFailed";
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

std::ostream& operator<<(std::ostream& os, const SolveInfo& info)
{
  os << "SolveInfo(status=" << info.status << ", iterations=" << info.iterations
     << ", objective=" << info.objective << ", solve_time=" << info.solve_time
     << ", run_time=" << info.run_time << ')';
  return os;
}

} // namespace affine_mpc
