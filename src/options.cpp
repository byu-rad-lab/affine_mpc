/**
 * @file options.cpp
 * @brief This file was specifically created to implement the operator<<
 *   overload for Options without including the full <ostream> header in
 *   options.hpp.
 */

#include "affine_mpc/options.hpp"

#include <ostream>

namespace affine_mpc {

std::ostream&
print(std::ostream& os, const Options& opts, bool capitalize_bools)
{
  auto boolStr = [&capitalize_bools](const bool value) {
    if (capitalize_bools)
      return value ? "True" : "False";
    return value ? "true" : "false";
  };
  os << "Options(use_input_cost=" << boolStr(opts.use_input_cost)
     << ", use_input_rate_cost=" << boolStr(opts.use_input_rate_cost)
     << ", input_bounds=" << opts.input_bounds
     << ", slew_initial_input=" << boolStr(opts.slew_initial_input)
     << ", slew_control_points=" << boolStr(opts.slew_control_points)
     << ", state_bounds=" << opts.state_bounds << ", bounded_states=[";
  for (size_t i{0}; i < opts.bounded_states.size(); ++i)
    os << (i ? ", " : "") << opts.bounded_states[i];
  os << "], num_general_constraints=" << opts.num_general_constraints
     << ", num_terminal_constraints=" << opts.num_terminal_constraints << ')';
  return os;
}

std::ostream& operator<<(std::ostream& os, const Options& opts)
{
  const bool capitalize_bools{false};
  return print(os, opts, capitalize_bools);
}

std::ostream& operator<<(std::ostream& os, const StateBoundMode mode)
{
  switch (mode) {
  case StateBoundMode::Off:
    return os << "Off";
  case StateBoundMode::Hard:
    return os << "Hard";
  case StateBoundMode::Soft:
    return os << "Soft";
  }
  return os << "StateBoundMode(" << static_cast<int>(mode) << ')';
}

std::ostream& operator<<(std::ostream& os, const InputBoundMode mode)
{
  switch (mode) {
  case InputBoundMode::ControlPoints:
    return os << "ControlPoints";
  case InputBoundMode::Exact:
    return os << "Exact";
  }
  return os << "InputBoundMode(" << static_cast<int>(mode) << ')';
}

} // namespace affine_mpc
