#ifndef AFFINE_MPC_DETAIL_MPC_DATA_HPP
#define AFFINE_MPC_DETAIL_MPC_DATA_HPP

/**
 * @file detail/mpc_data.hpp
 * @brief Defines MPCData and the change categories: the internal MPC data
 *   (as opposed to the QP data built from it) shared by MPC and the
 *   formulations.
 *
 * Internal: installed only because public headers include it (MPC holds
 * these types by value); not part of the public API.
 */

#include <Eigen/Core>
#include <cstdint>
#include <limits>
#include <numeric>
#include <type_traits>
#include <vector>

#include "affine_mpc/options.hpp"

namespace affine_mpc::detail {

/**
 * @enum DataChange
 * @brief Internal: categories of MPC data changes. Each category updates
 *   a different part of the QP, so a setter only marks its category and the
 *   QP is updated once per category in MPC::prepare() or MPC::solve().
 *
 * The initial state x0 is never pending: solve() applies it every call. It is
 * still a category (X0, the highest bit and not part of DataChangeSet::all())
 * so the QP terms can declare which of their parts depend on it. A new
 * category takes the bit of X0, and X0 moves to the next one.
 */
enum class DataChange : std::uint16_t
{
  ModelOffset = 1 << 0, ///< model offset `wd` (QP vectors only)
  Model = 1 << 1,       ///< model matrices `Ad`, `Bd` (QP matrices)
  Weights = 1 << 2,     ///< cost weights `Q`, `Q_f`, `R` (cost matrix)
  References = 1 << 3,  ///< state and input references (cost vector)
  /// input and state limits, slew rates, general constraint bounds (bounds)
  Limits = 1 << 4,
  PreviousInput = 1 << 5, ///< `u_prev` of the initial slew rate (bounds)
  /// general and terminal constraint matrices `Hx`, `Hu`, `Hf` (QP matrices)
  ConstraintMatrices = 1 << 6,
  X0 = 1 << 7, ///< initial state (applied by every solve())
};

/**
 * @class DataChangeSet
 * @brief Internal: a set of DataChange categories.
 */
class DataChangeSet
{
  using Bits = std::underlying_type_t<DataChange>;

public:
  constexpr DataChangeSet() noexcept = default;
  constexpr DataChangeSet(DataChange change) noexcept :
      bits_{static_cast<Bits>(change)}
  {}

  /// @brief Every category that can be pending (all but X0).
  static constexpr DataChangeSet all() noexcept
  {
    DataChangeSet set;
    set.bits_ = static_cast<Bits>(static_cast<Bits>(DataChange::X0) - 1);
    return set;
  }

  /// @brief Whether no category is in the set.
  constexpr bool empty() const noexcept { return bits_ == 0; }

  /// @brief Whether any category of `other` is in the set.
  constexpr bool has(DataChangeSet other) const noexcept
  {
    return (bits_ & other.bits_) != 0;
  }

  constexpr DataChangeSet operator|(DataChangeSet other) const noexcept
  {
    DataChangeSet set;
    set.bits_ = static_cast<Bits>(bits_ | other.bits_);
    return set;
  }

  constexpr DataChangeSet operator&(DataChangeSet other) const noexcept
  {
    DataChangeSet set;
    set.bits_ = static_cast<Bits>(bits_ & other.bits_);
    return set;
  }

  constexpr DataChangeSet& operator|=(DataChangeSet other) noexcept
  {
    bits_ |= other.bits_;
    return *this;
  }

  constexpr bool operator==(DataChangeSet other) const noexcept
  {
    return bits_ == other.bits_;
  }

  constexpr bool operator!=(DataChangeSet other) const noexcept
  {
    return bits_ != other.bits_;
  }

private:
  Bits bits_{0};
};

constexpr DataChangeSet operator|(DataChange lhs, DataChange rhs) noexcept
{
  return DataChangeSet{lhs} | rhs;
}

/**
 * @brief Internal: the indices of the bounded states with `saturate_states`
 *   (Options::bounded_states, or every state when it is empty), and none
 *   without it.
 */
inline std::vector<int> boundedStates(const Options& opts, int state_dim)
{
  if (!opts.saturate_states)
    return {};
  if (!opts.bounded_states.empty())
    return opts.bounded_states;
  std::vector<int> all(state_dim);
  std::iota(all.begin(), all.end(), 0);
  return all;
}

/**
 * @struct MPCData
 * @brief Internal: the MPC problem as set through the MPC setters (model,
 *   weights, references, limits, and the previous input). Owned by MPC,
 *   which validates every value; formulations only read it. Not part of the
 *   stable API and not exposed in Python.
 *
 * Optional members are allocated only when their option is enabled.
 */
struct MPCData
{
  /**
   * @brief Allocate the data and set the defaults (zero model, unit weights,
   *   zero references, infinite limits and slew rates, zero previous input).
   * @param state_dim State dimension.
   * @param input_dim Input dimension.
   * @param horizon_steps Number of horizon steps.
   * @param num_ctrl_pts Number of control points.
   * @param opts Options enabled at construction.
   */
  MPCData(int state_dim,
          int input_dim,
          int horizon_steps,
          int num_ctrl_pts,
          const Options& opts) :
      Ad{Eigen::MatrixXd::Zero(state_dim, state_dim)},
      Bd{Eigen::MatrixXd::Zero(state_dim, input_dim)},
      wd{Eigen::VectorXd::Zero(state_dim)},
      Q_big{state_dim * horizon_steps},
      x_ref{Eigen::VectorXd::Zero(state_dim * horizon_steps)},
      u_min{Eigen::VectorXd::Constant(input_dim, -inf)},
      u_max{Eigen::VectorXd::Constant(input_dim, inf)},
      x0{Eigen::VectorXd::Zero(state_dim)}
  {
    Q_big.setIdentity();
    if (opts.use_input_cost) {
      R_big.setIdentity(input_dim * num_ctrl_pts);
      ctrls_ref.setZero(input_dim * num_ctrl_pts);
    }
    if (opts.slew_initial_input)
      u0_slew.setConstant(input_dim, inf);
    if (opts.slew_initial_input || opts.use_input_rate_cost)
      u_prev.setZero(input_dim);
    if (opts.use_input_rate_cost)
      R_rate.setOnes(input_dim);
    if (opts.slew_control_points)
      ctrls_slew.setConstant(input_dim, inf);
    if (opts.saturate_states) {
      const int num_bounded{
          static_cast<int>(boundedStates(opts, state_dim).size())};
      x_min.setConstant(num_bounded, -inf);
      x_max.setConstant(num_bounded, inf);
      if (opts.soft_state_bounds) {
        x_soft_l1.setZero(num_bounded);
        x_soft_l2.setZero(num_bounded);
      }
    }
    if (opts.num_general_constraints > 0) {
      const int p{opts.num_general_constraints};
      Hx.setZero(p, state_dim);
      Hu.setZero(p, input_dim);
      h_min.setConstant(p, -inf);
      h_max.setConstant(p, inf);
    }
    if (opts.num_terminal_constraints > 0) {
      const int p{opts.num_terminal_constraints};
      Hf.setZero(p, state_dim);
      hf_min.setConstant(p, -inf);
      hf_max.setConstant(p, inf);
    }
  }

  static constexpr double inf{std::numeric_limits<double>::infinity()};

  Eigen::MatrixXd Ad; ///< discrete state matrix
  Eigen::MatrixXd Bd; ///< discrete input matrix
  Eigen::VectorXd wd; ///< discrete model offset

  /// state weights over the horizon (the last block is the terminal weight)
  Eigen::DiagonalMatrix<double, Eigen::Dynamic> Q_big;
  /// input weights for every control point (`use_input_cost`)
  Eigen::DiagonalMatrix<double, Eigen::Dynamic> R_big;
  /// diagonal input rate weights, one per input (`use_input_rate_cost`)
  Eigen::VectorXd R_rate;

  Eigen::VectorXd x_ref;     ///< stacked reference states over the horizon
  Eigen::VectorXd ctrls_ref; ///< stacked reference control points

  Eigen::VectorXd u_min, u_max; ///< input limits
  /// limits of the bounded states (`saturate_states`, see boundedStates())
  Eigen::VectorXd x_min, x_max;
  /// L1 and L2 penalties on the slack of each bounded state
  /// (`soft_state_bounds`)
  Eigen::VectorXd x_soft_l1, x_soft_l2;
  Eigen::VectorXd ctrls_slew; ///< control point slew rate
  Eigen::VectorXd u0_slew;    ///< initial input slew rate
  /// previous input (`slew_initial_input` or `use_input_rate_cost`)
  Eigen::VectorXd u_prev;

  /// general constraints h_min <= Hx x_k + Hu u_k <= h_max at every step
  /// (`num_general_constraints` rows each)
  Eigen::MatrixXd Hx, Hu;
  Eigen::VectorXd h_min, h_max;
  /// terminal constraints hf_min <= Hf x_T <= hf_max
  /// (`num_terminal_constraints` rows each)
  Eigen::MatrixXd Hf;
  Eigen::VectorXd hf_min, hf_max;

  /// the initial state of the last solve() (X0, set by MPC before the QP
  /// parts that depend on it are updated)
  Eigen::VectorXd x0;
};

} // namespace affine_mpc::detail

#endif // AFFINE_MPC_DETAIL_MPC_DATA_HPP
