#ifndef AFFINE_MPC_MPC_OPTIONS_HPP
#define AFFINE_MPC_MPC_OPTIONS_HPP

/**
 * @file options.hpp
 * @brief Defines the Options struct for configuring the MPC problem, its
 *   bound modes, and the ModelSparsity choice of initializeSolver().
 */

#include <iosfwd>
#include <vector>

namespace affine_mpc {

/**
 * @enum StateBoundMode
 * @brief Whether and how the MPC bounds the predicted states, chosen with
 *   Options::state_bounds.
 */
enum class StateBoundMode
{
  /// No state bounds (the default).
  Off,
  /// Hard bounds x_min <= x_k <= x_max on the bounded states at every step
  /// k = 1, ..., T (set with MPC::setStateLimits()).
  Hard,
  /// Soft bounds: one slack s >= 0 per bounded state, shared over the horizon,
  /// may widen both of its limits, x_min - s <= x_k <= x_max + s, at the cost
  /// l1^T s + s^T diag(l2) s (set with MPC::setStateLimits() and
  /// MPC::setStateBoundPenalties()).
  Soft,
};

/**
 * @enum InputBoundMode
 * @brief How the MPC enforces the input limits u_min <= u_k <= u_max, chosen
 *   with Options::input_bounds.
 *
 * Bounding the control points of a B-spline bounds every sampled input (the
 * convex hull property), but for degree > 1, or degree 1 with non-integer
 * knots, it can keep the samples away from the limits, so the inputs do not
 * use the full range. A custom basis (Parameterization::fromBasis()) has no
 * such property, so its sampled inputs are always bounded, whatever the mode.
 * Refined bounds lie between the two in rows and accuracy.
 */
enum class InputBoundMode
{
  /// Bound the control points (the default): one row per control point, and
  /// the sampled inputs stay inside the limits.
  ControlPoints,
  /// Bound the sampled inputs exactly, which lets the control points leave the
  /// limits. Degree 0: the same as ControlPoints (the control points are the
  /// sampled inputs). Degree 1: only the samples adjacent to each knot (at
  /// most 2*num_control_points-2 rows), which is exact. Degree > 1: every
  /// sample (T rows per input), which can make OSQP slower on long
  /// horizons.
  Exact,
  /// Bound the control points of the same spline after knot insertion: every
  /// knot interval of length l is split into max(2, ceil(l / L)) equal parts,
  /// with L = Options::refined_bound_interval samples, and the refined control
  /// points that some sample depends on are bounded. By the convex hull
  /// property of the refined control points, every sampled input stays inside
  /// the limits, with less of the range lost than with ControlPoints and far
  /// fewer rows than Exact on long horizons (about
  /// max(T / L, 2 (num_control_points - degree)) + degree rows per input).
  /// If that is not fewer than the rows of Exact (T per input), the rows of
  /// Exact are used instead. Degree 0: the same as ControlPoints; degree 1:
  /// the same as Exact (both are already exact and cheap).
  Refined,
};

/**
 * @struct Options
 * @brief The optional costs and constraints of the MPC problem, chosen at
 *   construction.
 *
 * Every optional cost and constraint is off by default. The options fix the
 * layout of the QP, so they can not be changed after constructing an MPC
 * instance. The fields are grouped as follows.
 *
 * Costs:
 * - use_input_cost: Enables the input cost on the control points
 *     ((uref_k - u_k)^T R (uref_k - u_k)).
 * - use_input_rate_cost: Enables a cost on the change of the sampled inputs
 *     ((u_k - u_{k-1})^T R_rate (u_k - u_{k-1}) with u_{-1} = u_prev).
 *
 * Input constraints (the input limits are always enforced):
 * - input_bounds: Bound the control points (ControlPoints, the default), the
 *     sampled inputs (Exact), or the control points after knot insertion
 *     (Refined) (see InputBoundMode).
 * - refined_bound_interval: Longest knot interval, in samples, after the knot
 *     insertion of InputBoundMode::Refined, which splits every interval at
 *     least once (positive).
 * - slew_initial_input: Slew-rate constraint on the initial input
 *     (|u0 - u_prev| <= u0_slew).
 * - slew_control_points: Slew-rate constraints on the parameterization
 *     control points (|v_{i+1} - v_i| <= control_point_slew). Requires a
 *     B-spline parameterization.
 *
 * State constraints:
 * - state_bounds: Off (the default), Hard, or Soft state bounds (see
 *     StateBoundMode).
 * - bounded_states: Indices of the states with limits (requires state_bounds
 *     other than Off). Empty (the default) bounds every state.
 *
 * General constraints:
 * - num_general_constraints: Number p of general linear constraints
 *     h_min <= Hx x_k + Hu u_k <= h_max at every step k = 0, ..., T-1 (x_0 is
 *     the measured state). Zero (the default) disables them.
 * - num_terminal_constraints: Number of terminal constraints
 *     hf_min <= Hf x_T <= hf_max. Zero (the default) disables them.
 */
struct Options
{
  /// @name Costs
  /// @{

  /// Enables the input cost term in the MPC objective
  bool use_input_cost = false;

  /// Enables a cost on the change of the sampled inputs (u_k - u_{k-1}),
  /// starting from the previous input (i.e. u_0 - u_prev)
  bool use_input_rate_cost = false;

  /// @}
  /// @name Input constraints
  /// @{

  /// Whether the input limits bound the control points or the sampled inputs
  /// (see InputBoundMode)
  InputBoundMode input_bounds{InputBoundMode::ControlPoints};

  /// Longest knot interval, in samples, after the knot insertion of
  /// InputBoundMode::Refined, which also splits every interval at least once
  /// (positive; unused by the other modes). Smaller values bound the sampled
  /// inputs more tightly with more rows.
  int refined_bound_interval = 16;

  /// Enables slew rate constraint on initial input (i.e. between u0 and u_prev)
  bool slew_initial_input = false;

  /// Enables slew rate constraints on parameterization control points (i.e.
  /// between v_i and v_{i+1})
  bool slew_control_points = false;

  /// @}
  /// @name State constraints
  /// @{

  /// Whether and how the states are bounded (see StateBoundMode)
  StateBoundMode state_bounds{StateBoundMode::Off};

  /// Indices of the states that have limits (requires state_bounds other than
  /// Off; strictly increasing, in [0, state_dim)). Empty bounds every state.
  /// Each bounded state adds a row per horizon step, so bounding only the
  /// states that need it keeps the QP small.
  std::vector<int> bounded_states{};

  /// @}
  /// @name General constraints
  /// @{

  /// Number of general linear constraints h_min <= Hx x_k + Hu u_k <= h_max per
  /// step (k = 0, ..., T-1, set with setGeneralConstraints()). Zero disables
  /// them; each one adds a row per horizon step.
  int num_general_constraints = 0;

  /// Number of terminal constraints hf_min <= Hf x_T <= hf_max (set with
  /// setTerminalConstraints()). Zero disables them.
  int num_terminal_constraints = 0;

  /// @}
};

/**
 * @enum ModelSparsity
 * @brief Which entries of the discrete model Ad and Bd the solver tracks,
 *   chosen with MPC::initializeSolver().
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
  /// structural zeros. Supported by the sparse formulation only for now.
  Full,
};

/**
 * @brief Stream a one-line summary of every option.
 * @param os Output stream.
 * @param opts Options to print.
 * @param capitalize_bools Print `True`/`False` (Python style) instead of
 *   `true`/`false`.
 * @return Reference to the output stream.
 */
std::ostream&
print(std::ostream& os, const Options& opts, bool capitalize_bools);

/**
 * @brief Stream a human-readable one-line summary of the enabled options.
 * @param os Output stream.
 * @param opts Options to print.
 * @return Reference to the output stream.
 */
std::ostream& operator<<(std::ostream& os, const Options& opts);

/// @brief Stream the name of a state bound mode (e.g. `Hard`).
std::ostream& operator<<(std::ostream& os, StateBoundMode mode);

/// @brief Stream the name of an input bound mode (e.g. `Exact`).
std::ostream& operator<<(std::ostream& os, InputBoundMode mode);

} // namespace affine_mpc

#endif // AFFINE_MPC_MPC_OPTIONS_HPP
