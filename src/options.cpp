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
     << ", slew_initial_input=" << boolStr(opts.slew_initial_input)
     << ", slew_control_points=" << boolStr(opts.slew_control_points)
     << ", saturate_states=" << boolStr(opts.saturate_states)
     << ", saturate_input_trajectory="
     << boolStr(opts.saturate_input_trajectory) << ')';
  return os;
}

std::ostream& operator<<(std::ostream& os, const Options& opts)
{
  const bool capitalize_bools{false};
  return print(os, opts, capitalize_bools);
}

} // namespace affine_mpc
