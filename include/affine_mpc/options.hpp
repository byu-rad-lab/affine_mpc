#ifndef AFFINE_MPC_MPC_OPTIONS_HPP
#define AFFINE_MPC_MPC_OPTIONS_HPP

/**
 * @file options.hpp
 * @brief Defines the Options struct for configuring the MPC problem.
 */

namespace affine_mpc {

/**
 * @struct Options
 * @brief Controls which optional features are enabled at MPC construction time.
 *
 * All fields default to false. These options are immutable after constructing
 * an MPC instance.
 *
 * - use_input_cost: Enables input regularization term in the cost function
 *     ((uref_k - u_k)^T R (uref_k - u_k)).
 * - slew_initial_input: Slew-rate constraint on initial input
 *     (|u0 - u_prev| <= u0_slew).
 * - slew_control_points: Enables slew-rate constraints on parameterization
 *     control points (|v_{i+1} - v_i| <= control_point_slew).
 * - saturate_states: Enables state saturation constraints.
 * - saturate_input_trajectory: Enables saturation of each input in the
 *     trajectory rather than just the control points. Only applicable for
 *     parameterizations with degree > 1. This adds constraints to the
 *     optimiztion, but can allow control points to be outside of input limits
 *     while keeping inputs within limits.
 */
struct Options
{
  /// Enables input cost term in the MPC objective
  bool use_input_cost = false;

  /// Enables slew rate constraint on initial input (i.e. between u0 and u_prev)
  bool slew_initial_input = false;

  /// Enables slew rate constraints on parameterization control points (i.e.
  /// between v_i and v_{i+1})
  bool slew_control_points = false;

  /// Enables state saturation constraints
  bool saturate_states = false;

  /// Enables saturation of each input in the trajectory rather than just the
  /// control points (unused if degree <= 1)
  bool saturate_input_trajectory = false;
};

} // namespace affine_mpc

#endif // AFFINE_MPC_MPC_OPTIONS_HPP
