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
 * @enum InputCostMode
 * @brief Whether and where the MPC evaluates the input cost with weight R,
 *   chosen with Options::input_cost.
 *
 * The two costs differ whenever control points influence different numbers
 * of samples (e.g. move-blocking with unequal blocks, geometric knots, or a
 * custom basis, whose coefficients may have no physical meaning).
 */
enum class InputCostMode
{
  /// No input cost (the default).
  Off,
  /// The cost sum_i (c_ref,i - c_i)^T R (c_ref,i - c_i) on the control points
  /// c_i, which weights every control point equally (references set with
  /// MPC::setReferenceInput() or MPC::setReferenceInputControlPoints()).
  ControlPoints,
  /// The cost sum_k (u_ref,k - u_k)^T R (u_ref,k - u_k) on the sampled inputs
  /// u_k, k = 0, ..., T-1, which weights the input trajectory itself
  /// (references set with MPC::setReferenceInput() or
  /// MPC::setReferenceInputTrajectory()). Adds a banded block to the cost
  /// matrix (bandwidth 2 degree + 1 for B-splines, dense for custom bases).
  Trajectory,
};

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
  /// Bound the sampled inputs u_k (the input trajectory) exactly, which lets
  /// the control points leave the limits. Degree 0: the same as ControlPoints
  /// (the control points are the sampled inputs). Degree 1: only the samples
  /// adjacent to each knot (at most 2*num_control_points-2 rows), which is
  /// exact. Degree > 1: every sample (T rows per input), which can make OSQP
  /// slower on long horizons.
  Trajectory,
  /// Bound the control points of a refined representation of the same input
  /// curve: knot insertion splits every knot interval of length l into
  /// max(2, ceil(K l / (T - 1))) equal parts, with
  /// K = Options::min_refined_intervals, and the refined control points that
  /// some sample depends on are bounded. By the convex
  /// hull property of the refined control points, every sampled input stays
  /// inside the limits, with less of the range lost than with ControlPoints
  /// and far fewer rows than Trajectory on long horizons (about
  /// max(K, 2 (num_control_points - degree)) + degree rows per input,
  /// independent of T).
  /// If that is not fewer than the rows of Trajectory (T per input), the rows
  /// of Trajectory are used instead. Degree 0: the same as ControlPoints;
  /// degree 1: the same as Trajectory (both are already exact and cheap).
  Refined,
};

/**
 * @enum SlewBoundMode
 * @brief Whether and how the MPC enforces the slew-rate limits
 *   |u_k - u_{k-1}| <= du_max for k = 1, ..., T-1 (set with
 *   MPC::setSlewRate(), per sample step), chosen with Options::slew_bounds.
 *
 * The limit is the same in every mode; the mode only decides how it is
 * enforced. The change from the previous input, u_0 - u_prev, has its own
 * limit (Options::slew_initial_input).
 */
enum class SlewBoundMode
{
  /// No slew-rate bounds (the default).
  Off,
  /// Bound the differences of consecutive control points, c_i - c_{i-1}, one
  /// row per difference, with the limit scaled by the knots: a degree-p
  /// B-spline changes by at most du_max per sample if
  /// |c_i - c_{i-1}| <= (t_{i+p} - t_i) du_max / p (knots in samples), or
  /// |c_i - c_{i-1}| <= du_max for degree 0. Exact for degree 0, and for
  /// degree 1 when every knot interval contains a whole sample step (e.g.
  /// integer knots at least one sample apart); otherwise sufficient (the
  /// sampled changes stay inside the limits, but may not reach them).
  /// Requires a B-spline parameterization.
  ControlPoints,
  /// Bound the changes u_k - u_{k-1} of the input trajectory exactly. Degree 0,
  /// and degree 1 when ControlPoints is exact: the rows of ControlPoints.
  /// Otherwise one row per sampled change (T-1 rows per input; banded for
  /// B-splines, dense for a custom basis), omitting changes whose basis row has
  /// a norm of at most 1e-9 times the largest. The only mode for a custom
  /// basis.
  Trajectory,
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
 * - input_cost: Off (the default), the input cost on the control points
 *     (ControlPoints), or on the sampled inputs (Trajectory) (see
 *     InputCostMode).
 * - use_input_rate_cost: Enables a cost on the change of the sampled inputs
 *     ((u_k - u_{k-1})^T R_rate (u_k - u_{k-1}) with u_{-1} = u_prev).
 *
 * Input constraints (the input limits are always enforced):
 * - input_bounds: Bound the control points (ControlPoints, the default), the
 *     sampled inputs (Trajectory), or the control points after knot insertion
 *     (Refined) (see InputBoundMode).
 * - min_refined_intervals: Minimum number of knot intervals across the horizon
 *     in the refined representation of InputBoundMode::Refined, which splits
 *     every interval at least once (non-negative; 0 splits every interval
 *     once; if this gives at least T constraints per input, Trajectory's
 *     exact constraints are used instead).
 * - slew_initial_input: Slew-rate constraint on the initial input
 *     (|u0 - u_prev| <= u0_slew).
 * - slew_bounds: Off (the default), or slew-rate bounds
 *     |u_k - u_{k-1}| <= du_max for k = 1, ..., T-1 enforced on the control
 *     points (ControlPoints) or the sampled inputs (Trajectory) (see
 *     SlewBoundMode).
 *
 * State constraints:
 * - state_bounds: Off (the default), Hard, or Soft state bounds (see
 *     StateBoundMode).
 * - bounded_states: Indices of the states with limits (requires state_bounds
 *     other than Off). Empty (the default) bounds every state.
 *
 * General constraints:
 * - num_general_constraints: Number of general linear constraints
 *     h_min <= Hx x_k + Hu u_k <= h_max at every step k = 0, ..., T-1 (x_0 is
 *     the measured state). Zero (the default) disables them.
 * - num_terminal_constraints: Number of terminal constraints
 *     hf_min <= Hf x_T <= hf_max. Zero (the default) disables them.
 */
struct Options
{
  /// @name Costs
  /// @{

  /// Whether and where the input cost is evaluated (see InputCostMode)
  InputCostMode input_cost{InputCostMode::Off};

  /// Enables a cost on the change of the sampled inputs (u_k - u_{k-1}),
  /// starting from the previous input (i.e. u_0 - u_prev)
  bool use_input_rate_cost = false;

  /// @}
  /// @name Input constraints
  /// @{

  /// Whether the input limits bound the control points or the sampled inputs
  /// (see InputBoundMode)
  InputBoundMode input_bounds{InputBoundMode::ControlPoints};

  /// Minimum number of knot intervals across the horizon in the refined
  /// representation of InputBoundMode::Refined, which also splits every
  /// interval at least once (non-negative; 0 splits every interval once;
  /// unused by the other modes). Doubling it shrinks the gap between the
  /// refined control points and the input about 4x and about doubles its
  /// constraints. If this gives at least T constraints per input, the T exact
  /// constraints of Trajectory are used instead.
  int min_refined_intervals = 16;

  /// Enables slew rate constraint on initial input (i.e. between u0 and u_prev)
  bool slew_initial_input = false;

  /// Whether and how the slew-rate limits |u_k - u_{k-1}| <= du_max for
  /// k = 1, ..., T-1 are enforced (see SlewBoundMode)
  SlewBoundMode slew_bounds{SlewBoundMode::Off};

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

/// @brief Stream the name of an input cost mode (e.g. `Trajectory`).
std::ostream& operator<<(std::ostream& os, InputCostMode mode);

/// @brief Stream the name of a state bound mode (e.g. `Hard`).
std::ostream& operator<<(std::ostream& os, StateBoundMode mode);

/// @brief Stream the name of an input bound mode (e.g. `Trajectory`).
std::ostream& operator<<(std::ostream& os, InputBoundMode mode);

/// @brief Stream the name of a slew bound mode (e.g. `ControlPoints`).
std::ostream& operator<<(std::ostream& os, SlewBoundMode mode);

} // namespace affine_mpc

#endif // AFFINE_MPC_MPC_OPTIONS_HPP
