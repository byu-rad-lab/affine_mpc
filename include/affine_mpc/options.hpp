#ifndef AFFINE_MPC_MPC_OPTIONS_HPP
#define AFFINE_MPC_MPC_OPTIONS_HPP

/**
 * @file options.hpp
 * @brief Defines the Options struct for configuring the MPC problem and the
 *   ModelSparsity choice of initializeSolver().
 */

#include <iosfwd>
#include <vector>

namespace affine_mpc {

/**
 * @struct Options
 * @brief Controls which optional features are enabled at MPC construction time.
 *
 * All flags default to false (bounded_states to empty, every state, and the
 * constraint counts to zero). These options are immutable after constructing
 * an MPC instance.
 *
 * - use_input_cost: Enables input regularization term in the cost function
 *     ((uref_k - u_k)^T R (uref_k - u_k)).
 * - slew_initial_input: Slew-rate constraint on initial input
 *     (|u0 - u_prev| <= u0_slew).
 * - slew_control_points: Enables slew-rate constraints on parameterization
 *     control points (|v_{i+1} - v_i| <= control_point_slew). Requires a
 *     B-spline parameterization.
 * - saturate_states: Enables state saturation constraints.
 * - saturate_input_trajectory: Enables saturation of the sampled input
 *     trajectory rather than just the control points. Ignored for degree 0,
 *     where the control points are the sampled inputs. For degree 1 only the
 *     samples adjacent to each knot are constrained (at most
 *     2*num_control_points-2 rows), which is exact for non-integer knots. For
 *     degree > 1 every sample is constrained. Allows control points to be
 *     outside of input limits while keeping inputs within limits. A custom
 *     basis (Parameterization::fromBasis()) always constrains every sample.
 * - use_input_rate_cost: Enables a cost on the change of the sampled inputs
 *     ((u_k - u_{k-1})^T R_rate (u_k - u_{k-1}) with u_{-1} = u_prev).
 * - soft_state_bounds: Makes the state limits soft (requires
 *     saturate_states): x_min - s <= x_k <= x_max + s with one slack s >= 0
 *     per bounded state, penalized by l1^T s + s^T diag(l2) s.
 * - bounded_states: Indices of the states with limits (requires
 *     saturate_states). Empty (the default) bounds every state.
 * - num_general_constraints: Number p of general linear constraints
 *     h_min <= Hx x_k + Hu u_k <= h_max at every step k = 0, ..., T-1 (x_0 is
 *     the measured state). Zero (the default) disables them.
 * - num_terminal_constraints: Number of terminal constraints
 *     hf_min <= Hf x_T <= hf_max. Zero (the default) disables them.
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

  /// Enables saturation of the sampled input trajectory rather than just the
  /// control points (ignored for degree 0; knot-adjacent samples for degree 1)
  bool saturate_input_trajectory = false;

  /// Enables a cost on the change of the sampled inputs (u_k - u_{k-1}),
  /// starting from the previous input (i.e. u_0 - u_prev)
  bool use_input_rate_cost = false;

  /// Makes the state limits soft (requires saturate_states): one slack per
  /// bounded state, shared over the horizon, may widen both of its limits at a
  /// cost set with setStateBoundPenalties()
  bool soft_state_bounds = false;

  /// Indices of the states that have limits (requires saturate_states;
  /// strictly increasing, in [0, state_dim)). Empty bounds every state. Each
  /// bounded state adds a row per horizon step, so bounding only the states
  /// that need it keeps the QP small.
  std::vector<int> bounded_states{};

  /// Number of general linear constraints h_min <= Hx x_k + Hu u_k <= h_max per
  /// step (k = 0, ..., T-1, set with setGeneralConstraints()). Zero disables
  /// them; each one adds a row per horizon step.
  int num_general_constraints = 0;

  /// Number of terminal constraints hf_min <= Hf x_T <= hf_max (set with
  /// setTerminalConstraints()). Zero disables them.
  int num_terminal_constraints = 0;
};

/**
 * @enum ModelSparsity
 * @brief Which entries of the discrete model Ad and Bd the solver tracks,
 *   chosen with MPCBase::initializeSolver().
 *
 * OSQP fixes the sparsity pattern of the QP at initialization, and later
 * updates only change the values of the tracked entries: a model entry outside
 * them must stay zero (a nonzero one is silently dropped). Tracking fewer
 * entries makes each solve cheaper when the model has structural zeros.
 */
enum class ModelSparsity
{
  /// Track the entries of Ad and Bd that are nonzero when initializeSolver()
  /// is called (the default). Entries that may become nonzero later must be
  /// nonzero at initializeSolver(), e.g. by setting a representative model
  /// before it and the actual model after it.
  InitialNonzeros,
  /// Track every entry of Ad and Bd (full blocks), so any entry may change
  /// between zero and nonzero later. Costs more per solve when the model has
  /// structural zeros. Supported by SparseMPC only for now.
  Full,
};

std::ostream&
print(std::ostream& os, const Options& opts, bool capitalize_bools);

/**
 * @brief Stream a human-readable one-line summary of the enabled options.
 * @param os Output stream.
 * @param opts Options to print.
 * @return Reference to the output stream.
 */
std::ostream& operator<<(std::ostream& os, const Options& opts);

} // namespace affine_mpc

#endif // AFFINE_MPC_MPC_OPTIONS_HPP
