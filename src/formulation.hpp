#ifndef AFFINE_MPC_FORMULATION_HPP
#define AFFINE_MPC_FORMULATION_HPP

// Internal (not installed): the row layout and the formulations
// that map state-space costs and constraints onto the QP variables.

#include <Eigen/Core>
#include <vector>

#include "affine_mpc/detail/mpc_data.hpp"
#include "affine_mpc/detail/qp.hpp"
#include "affine_mpc/detail/vector_parameterization.hpp"

namespace affine_mpc::detail {

/// A contiguous block of constraint rows.
struct RowBlock
{
  int offset{0};
  int rows{0};
};

/// A contiguous block of QP variables.
struct VarBlock
{
  int offset{0};
  int size{0};
};

/**
 * A matrix H (p x n) applied to a state in constraint rows, H x_k. It is
 * stored by its nonzero entries in row order, so the rows cost O(nnz(H)) per
 * step: a selection of states (rows of the identity, one entry per row) and a
 * general matrix share the same rows.
 */
class StateRowMap
{
public:
  struct Entry
  {
    int row, state;
    double value;
  };
  /// The rows of the identity for the given states (strictly increasing).
  static StateRowMap selection(const std::vector<int>& states, int state_dim);
  /// Room for a general p x n matrix, set with assign().
  static StateRowMap general(int rows, int state_dim);

  /// Store the nonzero entries of H (p x n), without allocating.
  void assign(const Eigen::Ref<const Eigen::MatrixXd>& H);
  /// Whether the stored entries are exactly the nonzero entries of H (for
  /// debug checks that a cached map is current).
  bool matches(const Eigen::Ref<const Eigen::MatrixXd>& H) const;

  int rows() const noexcept { return rows_; }
  /// Whether H is the identity (a selection of every state), which the
  /// formulations write as one block.
  bool isIdentity() const noexcept { return identity_; }
  const std::vector<Entry>& entries() const noexcept { return entries_; }

private:
  StateRowMap(int rows, bool identity) : rows_{rows}, identity_{identity} {}
  int rows_;
  bool identity_;
  std::vector<Entry> entries_;
};

/// Hands out QP variables (after the formulation's own) and constraint rows in
/// declaration order (construction only).
class QPLayout
{
public:
  explicit QPLayout(int formulation_variables) :
      num_variables_{formulation_variables}
  {}
  VarBlock addVariables(int count)
  {
    const VarBlock block{num_variables_, count};
    num_variables_ += count;
    return block;
  }
  RowBlock addRows(int rows)
  {
    const RowBlock block{num_rows_, rows};
    num_rows_ += rows;
    return block;
  }
  int numVariables() const noexcept { return num_variables_; }
  int numRows() const noexcept { return num_rows_; }

private:
  int num_variables_;
  int num_rows_{0};
};

/**
 * A formulation chooses the QP variables z, with the control points c first
 * (so control point terms are the same in every formulation), and maps costs
 * and constraints on the predicted states x_k onto them. Terms may declare
 * extra variables after the formulation's own (e.g. slacks); their cost
 * entries (diagonal only) are handled here, the same for every formulation.
 *
 * Costs accumulate into P and q between begin...() and end...(). Instead of
 * clearing P or q first, begin...() marks them stale: the first write to a
 * stale part assigns, later writes add, and end...() zeroes the parts no term
 * wrote. With one cost term per part this is a plain assignment, so it costs
 * nothing over writing P and q directly.
 */
class Formulation
{
public:
  /// @param qp The QP the formulation writes (owned by the QP builder).
  explicit Formulation(QP& qp) : qp_{qp} {}
  virtual ~Formulation() = default;
  Formulation(const Formulation&) = delete;
  Formulation& operator=(const Formulation&) = delete;

  /// Number of the formulation's own QP variables (before any extra ones).
  virtual int numVariables() const noexcept = 0;
  /// The extra variables the terms declared (after the formulation's own).
  void setExtraVariables(const VarBlock& extra) noexcept { extra_ = extra; }
  /// Declare the formulation's own constraint rows (the first rows of A).
  virtual RowBlock declareRows(QPLayout& layout) = 0;
  /// Write the parts of the own rows that never change (after allocation).
  virtual void writeConstantRows() {}

  /// Categories that change the map from z to the states' matrix part (terms
  /// on states add them to their matrix and vector dependencies).
  virtual DataChangeSet stateMatrixDependsOn() const noexcept = 0;
  /// Categories that change the map's offset part (vector dependencies only).
  virtual DataChangeSet stateOffsetDependsOn() const noexcept = 0;

  /// The QP parts written by an update, each with the categories behind it.
  struct Update
  {
    DataChangeSet P, A, q, bounds;
  };
  /// Update the state map and the own rows for the changed categories (no x0
  /// dependence).
  virtual Update update(DataChangeSet changes, const MPCData& d) = 0;
  /// Update the parts that depend on x0 (d.x0).
  virtual void updateX0(const MPCData& d) = 0;
  /// Whether updateX0() writes bounds (those are then sent at every solve).
  virtual bool updateX0WritesBounds() const noexcept = 0;

  /// Whether the cost entries hold unit placeholders at initializeSolver(), so
  /// that a weight that is zero then is still in the solver's sparsity
  /// pattern (the weights are re-applied after initialization).
  virtual bool usesCostPlaceholders() const noexcept { return false; }
  /// Write unit placeholders on the cost entries the formulation's own
  /// primitives use (see usesCostPlaceholders()).
  virtual void writeCostPlaceholders() {}
  /// Add a term's placeholder block to the control point block of P (for costs
  /// whose structure the formulation does not know). The block must be
  /// positive semidefinite, like the cost itself, or OSQP rejects P as
  /// nonconvex: the term's cost with unit weights is the natural choice.
  virtual void addControlCostPlaceholders(
      const Eigen::Ref<const Eigen::MatrixXd>& /*placeholders*/)
  {}

  /// Predicted states x = (x_1, ..., x_T) for the QP solution z.
  virtual void
  predictStates(const Eigen::Ref<const Eigen::VectorXd>& z,
                Eigen::Ref<Eigen::VectorXd> x_traj) const noexcept = 0;

  // ---- costs (between begin and end) ---------------------------------------
  void beginCostMatrix()
  {
    P_extra_.stale = true;
    beginOwnCostMatrix();
  }
  void endCostMatrix()
  {
    endOwnCostMatrix();
    finish(P_extra_, qp_.P.diagonal().segment(extra_.offset, extra_.size));
  }
  void beginCostVector()
  {
    q_extra_.stale = true;
    beginOwnCostVector();
  }
  void endCostVector()
  {
    endOwnCostVector();
    finish(q_extra_, qp_.q.segment(extra_.offset, extra_.size));
  }

  /// 1/2 v^T diag(w) v on extra variables v (a block declared by a term).
  void addVariableQuadratic(const VarBlock& block,
                            const Eigen::Ref<const Eigen::VectorXd>& w)
  {
    startPartialWrite(P_extra_,
                      qp_.P.diagonal().segment(extra_.offset, extra_.size));
    qp_.P.diagonal().segment(block.offset, block.size) += w;
  }
  /// g^T v on extra variables v.
  void addVariableLinear(const VarBlock& block,
                         const Eigen::Ref<const Eigen::VectorXd>& g)
  {
    startPartialWrite(q_extra_, qp_.q.segment(extra_.offset, extra_.size));
    qp_.q.segment(block.offset, block.size) += g;
  }

  /// Matrix part of 1/2 sum_k (x_k - r_k)^T Q_k (x_k - r_k), Q diagonal and
  /// stacked over the horizon.
  virtual void
  addStateQuadraticMatrix(const Eigen::Ref<const Eigen::VectorXd>& Q) = 0;
  /// Vector part of the same cost, with the stacked references r.
  virtual void
  addStateQuadraticVector(const Eigen::Ref<const Eigen::VectorXd>& Q,
                          const Eigen::Ref<const Eigen::VectorXd>& r) = 0;
  /// Matrix part of 1/2 (c - c_ref)^T diag(R) (c - c_ref).
  virtual void
  addControlQuadraticMatrix(const Eigen::Ref<const Eigen::VectorXd>& R) = 0;
  /// Vector part of the same cost.
  virtual void
  addControlQuadraticVector(const Eigen::Ref<const Eigen::VectorXd>& R,
                            const Eigen::Ref<const Eigen::VectorXd>& c_ref) = 0;
  /// 1/2 c^T P_c c for a general (symmetric) P_c on the control points.
  virtual void
  addControlMatrix(const Eigen::Ref<const Eigen::MatrixXd>& P_c) = 0;
  /// q_c^T c on the control points.
  virtual void
  addControlLinear(const Eigen::Ref<const Eigen::VectorXd>& q_c) = 0;

  // ---- constraint rows -----------------------------------------------------
  /// Rows H x_k for the predicted states x_k, k = first_step, ...,
  /// first_step + num_steps - 1 (1 <= k <= T), H.rows() rows per step in step
  /// order from block.offset. Afterwards the formulation's columns of these
  /// rows hold exactly H x_k (earlier values there, e.g. a term's entries on
  /// the control points, are cleared) and no other column is touched, so a
  /// term can then add entries on the control points or on its own variables.
  virtual void setStateRows(const RowBlock& block,
                            const StateRowMap& H,
                            int first_step,
                            int num_steps) = 0;
  /// Bounds lo <= H x_k <= hi (lo and hi have size H.rows()) for the rows of
  /// setStateRows() with the same arguments.
  virtual void setStateBounds(const RowBlock& block,
                              const StateRowMap& H,
                              int first_step,
                              int num_steps,
                              const Eigen::Ref<const Eigen::VectorXd>& lo,
                              const Eigen::Ref<const Eigen::VectorXd>& hi) = 0;

protected:
  virtual void beginOwnCostMatrix() = 0;
  virtual void endOwnCostMatrix() = 0;
  virtual void beginOwnCostVector() = 0;
  virtual void endOwnCostVector() = 0;

  // A part of P or q that costs accumulate into (see the class comment)
  struct CostPart
  {
    bool stale{false};   // assign on the next write
    bool written{false}; // may hold nonzero values
  };
  // Prepare a write that covers only some entries of `part` (stored in
  // `whole`): a stale part is zeroed first, since the write must add
  template <class Whole>
  static void startPartialWrite(CostPart& part, Whole&& whole)
  {
    if (part.stale) {
      whole.setZero();
      part.stale = false;
    }
    part.written = true;
  }
  // Zero a part that no term wrote but that may hold old values
  template <class Whole> static void finish(CostPart& part, Whole&& whole)
  {
    if (part.stale && part.written) {
      whole.setZero();
      part.written = false;
    }
    part.stale = false;
  }

  QP& qp_; // owned by the QP builder

private:
  VarBlock extra_{};           // the extra variables (empty without any)
  CostPart P_extra_, q_extra_; // their cost entries (diagonal of P, and q)
};

/**
 * Condensed: z = c, and the states are eliminated with the model,
 * x = S c + v (stacked over the horizon), where S depends on the model
 * matrices and v on the model, its offset, and x0.
 */
class CondensedFormulation final : public Formulation
{
public:
  CondensedFormulation(int state_dim,
                       const VectorParameterization& input_param,
                       QP& qp);

  int numVariables() const noexcept override { return nc_; }
  RowBlock declareRows(QPLayout&) override { return {}; }

  DataChangeSet stateMatrixDependsOn() const noexcept override
  {
    return DataChange::Model;
  }
  DataChangeSet stateOffsetDependsOn() const noexcept override
  {
    return DataChange::Model | DataChange::ModelOffset | DataChange::X0;
  }
  Update update(DataChangeSet changes, const MPCData& d) override;
  void updateX0(const MPCData& d) override;
  bool updateX0WritesBounds() const noexcept override { return false; }
  void
  predictStates(const Eigen::Ref<const Eigen::VectorXd>& z,
                Eigen::Ref<Eigen::VectorXd> x_traj) const noexcept override;

  void
  addStateQuadraticMatrix(const Eigen::Ref<const Eigen::VectorXd>& Q) override;
  void
  addStateQuadraticVector(const Eigen::Ref<const Eigen::VectorXd>& Q,
                          const Eigen::Ref<const Eigen::VectorXd>& r) override;
  void addControlQuadraticMatrix(
      const Eigen::Ref<const Eigen::VectorXd>& R) override;
  void addControlQuadraticVector(
      const Eigen::Ref<const Eigen::VectorXd>& R,
      const Eigen::Ref<const Eigen::VectorXd>& c_ref) override;
  void addControlMatrix(const Eigen::Ref<const Eigen::MatrixXd>& P_c) override;
  void addControlLinear(const Eigen::Ref<const Eigen::VectorXd>& q_c) override;

  void setStateRows(const RowBlock& block,
                    const StateRowMap& H,
                    int first_step,
                    int num_steps) override;
  void setStateBounds(const RowBlock& block,
                      const StateRowMap& H,
                      int first_step,
                      int num_steps,
                      const Eigen::Ref<const Eigen::VectorXd>& lo,
                      const Eigen::Ref<const Eigen::VectorXd>& hi) override;

  const Eigen::MatrixXd& S() const noexcept { return S_; }
  const Eigen::VectorXd& v() const noexcept { return v_; }

private:
  // the formulation's own block of P (q): all of it without extra variables
  auto ownP() { return qp_.P.topLeftCorner(nc_, nc_); }
  auto ownQ() { return qp_.q.head(nc_); }
  void beginOwnCostMatrix() override { P_.stale = true; }
  void endOwnCostMatrix() override { finish(P_, ownP()); }
  void beginOwnCostVector() override { q_.stale = true; }
  void endOwnCostVector() override { finish(q_, ownQ()); }

  void updateS(const MPCData& d); // from the model matrices
  void updateV(const MPCData& d);

  const int n_, m_, T_, nx_, nc_;
  const VectorParameterization& input_param_;
  // Number of leading columns of S that can be nonzero in each step's rows
  // (the control points after the step do not affect its state)
  const std::vector<int> band_cols_;
  Eigen::MatrixXd S_;
  Eigen::VectorXd v_;
  Eigen::MatrixXd weighted_S_;         // diag(Q) S, avoids a temp for P
  Eigen::VectorXd weighted_state_err_; // diag(Q) (v - r), avoids a temp
  // The own blocks of P and q are dense, so each is one part (the state cost
  // writes all of it, the control point cost adds to the control points)
  CostPart P_, q_;
};

/**
 * Sparse: z = (c, x), and the model is kept as equality rows
 * x_{k+1} = Ad x_k + Bd u_k + wd, the formulation's own rows.
 */
class SparseFormulation final : public Formulation
{
public:
  SparseFormulation(int state_dim,
                    const VectorParameterization& input_param,
                    QP& qp);

  int numVariables() const noexcept override { return nc_ + nx_; }
  RowBlock declareRows(QPLayout& layout) override
  {
    return layout.addRows(nx_);
  }
  void writeConstantRows() override;

  DataChangeSet stateMatrixDependsOn() const noexcept override { return {}; }
  DataChangeSet stateOffsetDependsOn() const noexcept override { return {}; }
  Update update(DataChangeSet changes, const MPCData& d) override;
  void updateX0(const MPCData& d) override;
  bool updateX0WritesBounds() const noexcept override { return true; }
  bool usesCostPlaceholders() const noexcept override { return true; }
  void writeCostPlaceholders() override;
  void addControlCostPlaceholders(
      const Eigen::Ref<const Eigen::MatrixXd>& placeholders) override;
  void
  predictStates(const Eigen::Ref<const Eigen::VectorXd>& z,
                Eigen::Ref<Eigen::VectorXd> x_traj) const noexcept override;

  void
  addStateQuadraticMatrix(const Eigen::Ref<const Eigen::VectorXd>& Q) override;
  void
  addStateQuadraticVector(const Eigen::Ref<const Eigen::VectorXd>& Q,
                          const Eigen::Ref<const Eigen::VectorXd>& r) override;
  void addControlQuadraticMatrix(
      const Eigen::Ref<const Eigen::VectorXd>& R) override;
  void addControlQuadraticVector(
      const Eigen::Ref<const Eigen::VectorXd>& R,
      const Eigen::Ref<const Eigen::VectorXd>& c_ref) override;
  void addControlMatrix(const Eigen::Ref<const Eigen::MatrixXd>& P_c) override;
  void addControlLinear(const Eigen::Ref<const Eigen::VectorXd>& q_c) override;

  void setStateRows(const RowBlock& block,
                    const StateRowMap& H,
                    int first_step,
                    int num_steps) override;
  void setStateBounds(const RowBlock& block,
                      const StateRowMap& H,
                      int first_step,
                      int num_steps,
                      const Eigen::Ref<const Eigen::VectorXd>& lo,
                      const Eigen::Ref<const Eigen::VectorXd>& hi) override;

private:
  void beginOwnCostMatrix() override { P_ctrl_.stale = P_state_.stale = true; }
  void endOwnCostMatrix() override;
  void beginOwnCostVector() override { q_ctrl_.stale = q_state_.stale = true; }
  void endOwnCostVector() override;
  // dynamics blocks of A (Bd times the basis weights, and Ad)
  void setDynamicsMatrixBlocks(const MPCData& d);
  // dynamics-row bounds -wd for steps k >= 1 (k = 0 also depends on x0)
  void setDynamicsOffsetBounds(const MPCData& d);

  const int n_, m_, T_, nx_, nc_; // the states start at column nc_
  const VectorParameterization& input_param_;
  // The state cost only has diagonal entries in P, so the cost parts are the
  // control point block of P, the state block of P's diagonal, and the two
  // blocks of q. The control point block is often only written on its
  // diagonal; P_ctrl_offdiag_ records whether its off-diagonal entries may
  // hold values (then a stale block is zeroed whole, not just its diagonal).
  CostPart P_ctrl_, P_state_, q_ctrl_, q_state_;
  bool P_ctrl_offdiag_{false};
};

} // namespace affine_mpc::detail

#endif // AFFINE_MPC_FORMULATION_HPP
