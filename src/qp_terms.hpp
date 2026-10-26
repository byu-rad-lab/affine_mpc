#ifndef AFFINE_MPC_QP_TERMS_HPP
#define AFFINE_MPC_QP_TERMS_HPP

// Internal (not installed): the features of the MPC problem as QP terms. Each
// term writes one cost or one block of constraint rows through the
// formulation, so the same term works for every formulation.

#include <Eigen/Core>
#include <cstddef>
#include <vector>

#include "affine_mpc/detail/mpc_data.hpp"
#include "affine_mpc/detail/vector_parameterization.hpp"
#include "formulation.hpp"

namespace affine_mpc::detail {

/**
 * One feature of the QP: a cost (added into P and q through the formulation)
 * or a block of constraint rows it owns and overwrites.
 *
 * The matrix part (P or the rows of A) and the vector part (q or the bounds)
 * are updated separately, each when a category it depends on changes. Parts
 * that depend on X0 are updated by every solve(); the others by prepare() or
 * the start of solve(). Parts with no dependencies are written once.
 */
class QPTerm
{
public:
  virtual ~QPTerm() = default;

  /// Declare the constraint rows the term owns (costs own none), its extra
  /// variables, and the entries of P and A it writes. Entries are written by
  /// position (QPEntries), never through the matrices; entries the term writes
  /// through the formulation (state rows, control point costs) are declared
  /// with the formulation's declare functions.
  virtual void declare(QPLayout& /*layout*/, Formulation& /*f*/) {}
  /// Whether the term is a cost (adds into P and q).
  virtual bool isCost() const noexcept { return false; }
  /// Categories that change the term's part of P or of A.
  virtual DataChangeSet matrixDependsOn(const Formulation& f) const = 0;
  /// Categories that change the term's part of q or of the bounds.
  ///
  /// When both parts run for a change, the matrix part runs first (see
  /// QPBuilder::apply()), and the X0 update runs only vector parts. So a
  /// vector part may read state its matrix part caches (e.g. the nonzero
  /// entries of a constraint matrix) if every category that refreshes that
  /// state is also in vectorDependsOn().
  virtual DataChangeSet vectorDependsOn(const Formulation& f) const = 0;
  virtual void updateMatrix(const MPCData& d, Formulation& f, QP& qp) = 0;
  virtual void updateVector(const MPCData& d, Formulation& f, QP& qp) = 0;
  /// For formulations with cost placeholders (see
  /// Formulation::usesCostPlaceholders()): mark the cost entries the term can
  /// write beyond those the formulation's primitives already cover.
  virtual void writeCostPlaceholders(Formulation& /*f*/) {}

  /// The rows declared by declare() (empty for costs).
  const RowBlock& rows() const noexcept { return rows_; }

protected:
  RowBlock rows_{};
};

/// Entries with values fixed at construction (constant rows of A).
struct ConstantEntries
{
  QPEntries entries;
  std::vector<double> values; // in the order of the entries

  void declare(int row, int col, double value)
  {
    entries.declare(row, col);
    values.push_back(value);
  }
  void write()
  {
    for (std::size_t i{0}; i < values.size(); ++i)
      entries.set(static_cast<Eigen::Index>(i), values[i]);
  }
};

/// 1/2 sum_k (x_k - x_ref,k)^T Q_k (x_k - x_ref,k), with Q_{T-1} = Q_f.
class StateCost final : public QPTerm
{
public:
  bool isCost() const noexcept override { return true; }
  DataChangeSet matrixDependsOn(const Formulation& f) const override
  {
    return DataChangeSet{DataChange::Weights} | f.stateMatrixDependsOn();
  }
  DataChangeSet vectorDependsOn(const Formulation& f) const override
  {
    return DataChange::Weights | DataChange::References
           | f.stateMatrixDependsOn() | f.stateOffsetDependsOn();
  }
  void updateMatrix(const MPCData& d, Formulation& f, QP& qp) override;
  void updateVector(const MPCData& d, Formulation& f, QP& qp) override;
};

/// 1/2 sum_i (c_i - c_ref,i)^T R (c_i - c_ref,i) on the control points
/// (`use_input_cost`).
class InputCost final : public QPTerm
{
public:
  bool isCost() const noexcept override { return true; }
  DataChangeSet matrixDependsOn(const Formulation&) const override
  {
    return DataChange::Weights;
  }
  DataChangeSet vectorDependsOn(const Formulation&) const override
  {
    return DataChange::Weights | DataChange::References;
  }
  void updateMatrix(const MPCData& d, Formulation& f, QP& qp) override;
  void updateVector(const MPCData& d, Formulation& f, QP& qp) override;
};

/// 1/2 sum_k (u_k - u_{k-1})^T R_rate (u_k - u_{k-1}) over the sampled inputs,
/// with u_{-1} = u_prev (`use_input_rate_cost`).
///
/// Per input channel, the differences are D Phi c - e_0 u_prev, where Phi maps
/// the channel's control points to its samples and D takes differences (its
/// first row is e_0^T). So the matrix part is G = (D Phi)^T (D Phi) times the
/// channel's weight, and the linear part is -Phi_0^T R_rate u_prev (Phi_0 is
/// the first row of Phi, the basis weights of u_0). G is banded and depends
/// only on the basis, so it is computed once.
class InputRateCost final : public QPTerm
{
public:
  explicit InputRateCost(const VectorParameterization& input_param);
  bool isCost() const noexcept override { return true; }
  DataChangeSet matrixDependsOn(const Formulation&) const override
  {
    return DataChange::Weights;
  }
  DataChangeSet vectorDependsOn(const Formulation&) const override
  {
    return DataChange::Weights | DataChange::PreviousInput;
  }
  void declare(QPLayout& layout, Formulation& f) override;
  void updateMatrix(const MPCData& d, Formulation& f, QP& qp) override;
  void updateVector(const MPCData& d, Formulation& f, QP& qp) override;
  void writeCostPlaceholders(Formulation& f) override;

private:
  struct GEntry // a nonzero G(i, j) between control points i and j
  {
    int i, j;
    double value;
  };
  struct Phi0Entry // a nonzero Phi_0(i)
  {
    int i;
    double value;
  };

  const VectorParameterization& input_param_;
  std::vector<GEntry> G_;
  std::vector<Phi0Entry> phi0_;
  // G on each channel's control points (channel by channel, G_'s order), and
  // its values times the weights and with unit weights (the placeholders)
  QPEntries P_c_;
  Eigen::VectorXd P_c_values_, P_c_placeholders_;
  Eigen::VectorXd q_c_; // the linear part, on all coefficients
};

/// u_min <= input <= u_max, on the control points or, with
/// `saturate_input_trajectory`, on the sampled inputs that make it exact.
class InputBounds final : public QPTerm
{
public:
  InputBounds(const VectorParameterization& input_param,
              bool saturate_input_trajectory);
  void declare(QPLayout& layout, Formulation& f) override;
  DataChangeSet matrixDependsOn(const Formulation&) const override
  {
    return {};
  }
  DataChangeSet vectorDependsOn(const Formulation&) const override
  {
    return DataChange::Limits;
  }
  void updateMatrix(const MPCData& d, Formulation& f, QP& qp) override;
  void updateVector(const MPCData& d, Formulation& f, QP& qp) override;

private:
  const VectorParameterization& input_param_;
  // bounded sample indices (empty: the control points are bounded instead)
  const std::vector<int> samples_;
  const int num_bounded_; // number of bounded inputs (samples or points)
  ConstantEntries A_;
};

/// |u_0 - u_prev| <= u0_slew (`slew_initial_input`).
class InitialSlew final : public QPTerm
{
public:
  explicit InitialSlew(const VectorParameterization& input_param) :
      input_param_{input_param}
  {}
  void declare(QPLayout& layout, Formulation& f) override;
  DataChangeSet matrixDependsOn(const Formulation&) const override
  {
    return {};
  }
  DataChangeSet vectorDependsOn(const Formulation&) const override
  {
    return DataChange::Limits | DataChange::PreviousInput;
  }
  void updateMatrix(const MPCData& d, Formulation& f, QP& qp) override;
  void updateVector(const MPCData& d, Formulation& f, QP& qp) override;

private:
  const VectorParameterization& input_param_;
  ConstantEntries A_;
};

/// |c_{i+1} - c_i| <= control_point_slew (`slew_control_points`).
class ControlPointSlew final : public QPTerm
{
public:
  explicit ControlPointSlew(const VectorParameterization& input_param) :
      input_param_{input_param}
  {}
  void declare(QPLayout& layout, Formulation& f) override;
  DataChangeSet matrixDependsOn(const Formulation&) const override
  {
    return {};
  }
  DataChangeSet vectorDependsOn(const Formulation&) const override
  {
    return DataChange::Limits;
  }
  void updateMatrix(const MPCData& d, Formulation& f, QP& qp) override;
  void updateVector(const MPCData& d, Formulation& f, QP& qp) override;

private:
  const VectorParameterization& input_param_;
  ConstantEntries A_;
};

/// x_min <= x_{k,i} <= x_max at every step k for each bounded state i
/// (`saturate_states`; `bounded_states` selects them).
///
/// Soft (`soft_state_bounds`): one slack s_j >= 0 per bounded state, shared
/// over the horizon, widens both of its limits,
/// x_min,j - s_j <= x_{k,i_j} <= x_max,j + s_j. One row can not hold a slack
/// on both sides, so each limit is its own one-sided row (2 T rows per
/// bounded state), plus one row per bounded state for s >= 0.
/// SoftStatePenalty prices the slacks.
class StateBounds final : public QPTerm
{
public:
  StateBounds(const std::vector<int>& states,
              int state_dim,
              int horizon_steps,
              bool soft);
  void declare(QPLayout& layout, Formulation& f) override;
  DataChangeSet matrixDependsOn(const Formulation& f) const override
  {
    return f.stateMatrixDependsOn();
  }
  DataChangeSet vectorDependsOn(const Formulation& f) const override
  {
    return DataChangeSet{DataChange::Limits} | f.stateOffsetDependsOn();
  }
  void updateMatrix(const MPCData& d, Formulation& f, QP& qp) override;
  void updateVector(const MPCData& d, Formulation& f, QP& qp) override;

  /// The slack variables (empty unless soft).
  const VarBlock& slack() const noexcept { return slack_; }

private:
  const StateRowMap states_; // the bounded states (rows of the identity)
  const int T_;
  const bool soft_;
  RowBlock upper_, lower_, nonneg_; // soft only: x - s <= max, x + s >= min
  StateRows hard_rows_, upper_rows_, lower_rows_; // the formulation's rows
  VarBlock slack_;
  ConstantEntries slack_A_;              // soft only: the slack columns
  Eigen::VectorXd minus_inf_, plus_inf_; // the open sides of the soft rows
};

/// h_min <= Hx x_k + Hu u_k <= h_max at every step k = 0, ..., T-1, p rows per
/// step (`num_general_constraints`).
///
/// The QP's predicted states are x_1, ..., x_T, and x_0 is the measured state:
/// the k = 0 rows are Hu u_0 with Hx x_0 moved into the bounds (so the bounds
/// depend on X0 in every formulation), and the rows for k >= 1 are the
/// formulation's state rows Hx x_k plus Hu u_k on the control points.
class GeneralConstraints final : public QPTerm
{
public:
  GeneralConstraints(const VectorParameterization& input_param,
                     int num_constraints,
                     int state_dim);
  void declare(QPLayout& layout, Formulation& f) override;
  DataChangeSet matrixDependsOn(const Formulation& f) const override
  {
    return DataChangeSet{DataChange::ConstraintMatrices}
           | f.stateMatrixDependsOn();
  }
  DataChangeSet vectorDependsOn(const Formulation& f) const override
  {
    return DataChange::Limits | DataChange::ConstraintMatrices | DataChange::X0
           | f.stateOffsetDependsOn();
  }
  void updateMatrix(const MPCData& d, Formulation& f, QP& qp) override;
  void updateVector(const MPCData& d, Formulation& f, QP& qp) override;

private:
  const VectorParameterization& input_param_;
  const int p_, T_;
  StateRowMap Hx_; // the nonzero entries of Hx (refreshed with the matrix)
  StateRows state_rows_; // the formulation's rows Hx x_k for k >= 1
  // Hu times the basis weights of u_k, a p x m block per step k and band
  // entry, on the control points
  QPEntries Hu_;
  Eigen::VectorXd Hx0_; // Hx x_0
};

/// hf_min <= Hf x_T <= hf_max (`num_terminal_constraints` rows).
class TerminalConstraints final : public QPTerm
{
public:
  TerminalConstraints(int num_constraints, int state_dim, int horizon_steps) :
      T_{horizon_steps}, Hf_{StateRowMap::general(num_constraints, state_dim)}
  {}
  void declare(QPLayout& layout, Formulation& f) override
  {
    rows_ = layout.addRows(Hf_.rows());
    state_rows_ = f.declareStateRows(layout, rows_, Hf_, T_, 1);
  }
  DataChangeSet matrixDependsOn(const Formulation& f) const override
  {
    return DataChangeSet{DataChange::ConstraintMatrices}
           | f.stateMatrixDependsOn();
  }
  DataChangeSet vectorDependsOn(const Formulation& f) const override
  {
    return DataChange::Limits | DataChange::ConstraintMatrices
           | f.stateOffsetDependsOn();
  }
  void updateMatrix(const MPCData& d, Formulation& f, QP& qp) override;
  void updateVector(const MPCData& d, Formulation& f, QP& qp) override;

private:
  const int T_;
  StateRowMap Hf_; // the nonzero entries of Hf (refreshed with the matrix)
  StateRows state_rows_;
};

/// l1^T s + s^T diag(l2) s on the state bound slacks s (`soft_state_bounds`),
/// in the units of the rest of the cost (so 1/2 of it enters the QP).
class SoftStatePenalty final : public QPTerm
{
public:
  explicit SoftStatePenalty(const VarBlock& slack) :
      slack_{slack}, half_l1_{slack.size}
  {}
  bool isCost() const noexcept override { return true; }
  DataChangeSet matrixDependsOn(const Formulation&) const override
  {
    return DataChange::Weights;
  }
  DataChangeSet vectorDependsOn(const Formulation&) const override
  {
    return DataChange::Weights;
  }
  void updateMatrix(const MPCData& d, Formulation& f, QP& qp) override;
  void updateVector(const MPCData& d, Formulation& f, QP& qp) override;

private:
  const VarBlock slack_;
  Eigen::VectorXd half_l1_; // the linear part, l1 / 2
};

} // namespace affine_mpc::detail

#endif // AFFINE_MPC_QP_TERMS_HPP
