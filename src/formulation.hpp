#ifndef AFFINE_MPC_FORMULATION_HPP
#define AFFINE_MPC_FORMULATION_HPP

// Internal (not installed): the QP storage, row layout, and the formulations
// that map state-space costs and constraints onto the QP variables.

#include <Eigen/Core>

#include "affine_mpc/detail/mpc_data.hpp"
#include "affine_mpc/detail/vector_parameterization.hpp"

namespace affine_mpc::detail {

/// The QP storage owned by MPCBase:
/// minimize 1/2 z^T P z + q^T z subject to l <= A z <= u.
struct QPRef
{
  Eigen::MatrixXd& P;
  Eigen::MatrixXd& A;
  Eigen::VectorXd& q;
  Eigen::VectorXd& l;
  Eigen::VectorXd& u;
};

/// A contiguous block of constraint rows.
struct RowBlock
{
  int offset{0};
  int rows{0};
};

/// Hands out constraint rows in declaration order (construction only).
class QPLayout
{
public:
  RowBlock addRows(int rows)
  {
    const RowBlock block{num_rows_, rows};
    num_rows_ += rows;
    return block;
  }
  int numRows() const noexcept { return num_rows_; }

private:
  int num_rows_{0};
};

/**
 * A formulation chooses the QP variables z, with the control points c first
 * (so control point terms are the same in every formulation), and maps costs
 * and constraints on the predicted states x_k onto them.
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
  explicit Formulation(QPRef qp) : qp_{qp} {}
  virtual ~Formulation() = default;
  Formulation(const Formulation&) = delete;
  Formulation& operator=(const Formulation&) = delete;

  /// Number of QP variables.
  virtual int numVariables() const noexcept = 0;
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
  /// Update the parts that depend on x0.
  virtual void updateX0(const MPCData& d,
                        const Eigen::Ref<const Eigen::VectorXd>& x0) = 0;
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
  virtual void beginCostMatrix() = 0;
  virtual void endCostMatrix() = 0;
  virtual void beginCostVector() = 0;
  virtual void endCostVector() = 0;

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
  /// 1/2 c^T H c for a general (symmetric) H on the control points.
  virtual void addControlMatrix(const Eigen::Ref<const Eigen::MatrixXd>& H) = 0;
  /// g^T c on the control points.
  virtual void addControlLinear(const Eigen::Ref<const Eigen::VectorXd>& g) = 0;

  // ---- constraint rows -----------------------------------------------------
  /// Rows x_k for every step, in the block (state_dim rows per step).
  virtual void setStateBoxRows(const RowBlock& block) = 0;
  /// Bounds lo <= x_k <= hi for every step (lo and hi have size state_dim).
  virtual void
  setStateBoxBounds(const RowBlock& block,
                    const Eigen::Ref<const Eigen::VectorXd>& lo,
                    const Eigen::Ref<const Eigen::VectorXd>& hi) = 0;

protected:
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

  QPRef qp_;
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
                       QPRef qp);

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
  void updateX0(const MPCData& d,
                const Eigen::Ref<const Eigen::VectorXd>& x0) override;
  bool updateX0WritesBounds() const noexcept override { return false; }
  void
  predictStates(const Eigen::Ref<const Eigen::VectorXd>& z,
                Eigen::Ref<Eigen::VectorXd> x_traj) const noexcept override;

  void beginCostMatrix() override { P_.stale = true; }
  void endCostMatrix() override { finish(P_, qp_.P); }
  void beginCostVector() override { q_.stale = true; }
  void endCostVector() override { finish(q_, qp_.q); }
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
  void addControlMatrix(const Eigen::Ref<const Eigen::MatrixXd>& H) override;
  void addControlLinear(const Eigen::Ref<const Eigen::VectorXd>& g) override;

  void setStateBoxRows(const RowBlock& block) override;
  void setStateBoxBounds(const RowBlock& block,
                         const Eigen::Ref<const Eigen::VectorXd>& lo,
                         const Eigen::Ref<const Eigen::VectorXd>& hi) override;

  const Eigen::MatrixXd& S() const noexcept { return S_; }
  const Eigen::VectorXd& v() const noexcept { return v_; }

private:
  void updateS(const MPCData& d); // from the model matrices
  void updateV(const MPCData& d, const Eigen::Ref<const Eigen::VectorXd>& x0);

  const int n_, m_, T_, nx_, nc_;
  const VectorParameterization& input_param_;
  Eigen::MatrixXd S_;
  Eigen::VectorXd v_;
  Eigen::MatrixXd weighted_S_;         // diag(Q) S, avoids a temp for P
  Eigen::VectorXd weighted_state_err_; // diag(Q) (v - r), avoids a temp
  // P and q are dense, so each is one part (the state cost writes all of it,
  // the control point cost adds to the control point entries)
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
                    QPRef qp);

  int numVariables() const noexcept override { return nc_ + nx_; }
  RowBlock declareRows(QPLayout& layout) override
  {
    return layout.addRows(nx_);
  }
  void writeConstantRows() override;

  DataChangeSet stateMatrixDependsOn() const noexcept override { return {}; }
  DataChangeSet stateOffsetDependsOn() const noexcept override { return {}; }
  Update update(DataChangeSet changes, const MPCData& d) override;
  void updateX0(const MPCData& d,
                const Eigen::Ref<const Eigen::VectorXd>& x0) override;
  bool updateX0WritesBounds() const noexcept override { return true; }
  bool usesCostPlaceholders() const noexcept override { return true; }
  void writeCostPlaceholders() override;
  void addControlCostPlaceholders(
      const Eigen::Ref<const Eigen::MatrixXd>& placeholders) override;
  void
  predictStates(const Eigen::Ref<const Eigen::VectorXd>& z,
                Eigen::Ref<Eigen::VectorXd> x_traj) const noexcept override;

  void beginCostMatrix() override { P_ctrl_.stale = P_state_.stale = true; }
  void endCostMatrix() override;
  void beginCostVector() override { q_ctrl_.stale = q_state_.stale = true; }
  void endCostVector() override;
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
  void addControlMatrix(const Eigen::Ref<const Eigen::MatrixXd>& H) override;
  void addControlLinear(const Eigen::Ref<const Eigen::VectorXd>& g) override;

  void setStateBoxRows(const RowBlock& block) override;
  void setStateBoxBounds(const RowBlock& block,
                         const Eigen::Ref<const Eigen::VectorXd>& lo,
                         const Eigen::Ref<const Eigen::VectorXd>& hi) override;

private:
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
