#ifndef AFFINE_MPC_DETAIL_MPC_DATA_HPP
#define AFFINE_MPC_DETAIL_MPC_DATA_HPP

/**
 * @file detail/mpc_data.hpp
 * @brief Defines MPCData and the change categories: the internal MPC data
 *   (as opposed to the QP data built from it) shared by MPCBase and the
 *   formulations.
 *
 * Internal: installed only because public headers include it (MPCBase holds
 * these types by value); not part of the public API.
 */

#include <Eigen/Core>
#include <cstdint>
#include <limits>

#include "affine_mpc/options.hpp"

namespace affine_mpc::detail {

/**
 * @enum DataChange
 * @brief Internal: categories of MPC data changes. Each category updates
 *   a different part of the QP, so a setter only marks its category and the
 *   QP is updated once per category in MPCBase::prepare() or MPCBase::solve().
 *
 * The initial state x0 is never pending: solve() applies it every call. It is
 * still a category (X0, not part of DataChangeSet::all()) so the QP terms can
 * declare which of their parts depend on it.
 */
enum class DataChange : std::uint8_t
{
  ModelOffset = 1 << 0,   ///< model offset `wd` (QP vectors only)
  Model = 1 << 1,         ///< model matrices `Ad`, `Bd` (QP matrices)
  Weights = 1 << 2,       ///< cost weights `Q`, `Q_f`, `R` (cost matrix)
  References = 1 << 3,    ///< state and input references (cost vector)
  Limits = 1 << 4,        ///< input and state limits, slew rates (bounds)
  PreviousInput = 1 << 5, ///< `u_prev` of the initial slew rate (bounds)
  X0 = 1 << 6,            ///< initial state (applied by every solve())
};

/**
 * @class DataChangeSet
 * @brief Internal: a set of DataChange categories.
 */
class DataChangeSet
{
public:
  constexpr DataChangeSet() noexcept = default;
  constexpr DataChangeSet(DataChange change) noexcept :
      bits_{static_cast<std::uint8_t>(change)}
  {}

  /// @brief Every category that can be pending (all but X0).
  static constexpr DataChangeSet all() noexcept
  {
    DataChangeSet set;
    set.bits_ = (1 << 6) - 1;
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
    set.bits_ = bits_ | other.bits_;
    return set;
  }

  constexpr DataChangeSet operator&(DataChangeSet other) const noexcept
  {
    DataChangeSet set;
    set.bits_ = bits_ & other.bits_;
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
  std::uint8_t bits_{0};
};

constexpr DataChangeSet operator|(DataChange lhs, DataChange rhs) noexcept
{
  return DataChangeSet{lhs} | rhs;
}

/**
 * @struct MPCData
 * @brief Internal: the MPC problem as set through the MPCBase setters (model,
 *   weights, references, limits, and the previous input). Owned by MPCBase,
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
      u_max{Eigen::VectorXd::Constant(input_dim, inf)}
  {
    Q_big.setIdentity();
    if (opts.use_input_cost) {
      R_big.setIdentity(input_dim * num_ctrl_pts);
      ctrls_ref.setZero(input_dim * num_ctrl_pts);
    }
    if (opts.slew_initial_input) {
      u0_slew.setConstant(input_dim, inf);
      u_prev.setZero(input_dim);
    }
    if (opts.slew_control_points)
      ctrls_slew.setConstant(input_dim, inf);
    if (opts.saturate_states) {
      x_min.setConstant(state_dim, -inf);
      x_max.setConstant(state_dim, inf);
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

  Eigen::VectorXd x_ref;     ///< stacked reference states over the horizon
  Eigen::VectorXd ctrls_ref; ///< stacked reference control points

  Eigen::VectorXd u_min, u_max; ///< input limits
  Eigen::VectorXd x_min, x_max; ///< state limits (`saturate_states`)
  Eigen::VectorXd ctrls_slew;   ///< control point slew rate
  Eigen::VectorXd u0_slew;      ///< initial input slew rate
  Eigen::VectorXd u_prev;       ///< previous input (`slew_initial_input`)
};

} // namespace affine_mpc::detail

#endif // AFFINE_MPC_DETAIL_MPC_DATA_HPP
