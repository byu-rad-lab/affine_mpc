#ifndef AFFINE_MPC_QP_BUILDER_HPP
#define AFFINE_MPC_QP_BUILDER_HPP

// Internal (not installed): builds and updates the QP from the MPC data with
// a formulation and the list of QP terms selected by the options.

#include <Eigen/Core>
#include <memory>
#include <vector>

#include "affine_mpc/detail/mpc_data.hpp"
#include "affine_mpc/detail/vector_parameterization.hpp"
#include "affine_mpc/mpc_base.hpp"
#include "affine_mpc/options.hpp"
#include "formulation.hpp"
#include "qp_terms.hpp"

namespace affine_mpc::detail {

/// The constraint row blocks, in their order in A (empty when not enabled).
struct QPRows
{
  RowBlock dynamics; ///< the formulation's own rows (sparse model)
  RowBlock input_bounds;
  RowBlock initial_slew;
  RowBlock control_point_slew;
  RowBlock state_bounds; ///< soft: the upper rows, lower rows, then s >= 0
  RowBlock general_constraints;  ///< p rows per step, steps in order
  RowBlock terminal_constraints; ///< the rows on x_T
};

/**
 * Owns the QP, the formulation, and the QP terms. The term list, the order of
 * the rows, and so the QP's sizes are fixed at construction, where the QP is
 * allocated; the lists each update walks are computed once there too.
 */
class QPBuilder
{
public:
  /// Builds the term list and allocates the QP (zero); call
  /// writeConstantParts() before the first update.
  QPBuilder(FormulationKind kind,
            int state_dim,
            const VectorParameterization& input_param,
            const Options& opts);
  // the formulation keeps a reference to qp_
  QPBuilder(const QPBuilder&) = delete;
  QPBuilder& operator=(const QPBuilder&) = delete;

  /// The QP as last written by the updates.
  const QP& qp() const noexcept { return qp_; }
  /// The full symmetric cost matrix as a dense matrix: P itself for dense
  /// storage, else a copy filled on each call into a member allocated at the
  /// first call (not for the control loop).
  const Eigen::MatrixXd& denseCostMatrix() const;

  int numVariables() const noexcept { return num_variables_; }
  int numConstraints() const noexcept { return num_constraints_; }
  const QPRows& rows() const noexcept { return rows_; }
  const Formulation& formulation() const noexcept { return *formulation_; }

  /// Every entry set of P (A) declared at construction, in declaration
  /// order. Writes by position only reach these entries (an entry may be in
  /// several sets); the formulations' bulk writes into dense storage are not
  /// declared. For sparse storage the deduplicated entries (in P, those on
  /// or above the diagonal, plus the diagonal) are the CSC pattern.
  const std::vector<QPEntries*>& declaredP() const noexcept
  {
    return declared_P_;
  }
  const std::vector<QPEntries*>& declaredA() const noexcept
  {
    return declared_A_;
  }

  /// Write the parts of the QP that never change (after allocation).
  void writeConstantParts(const MPCData& d);

  /// Update every part of the QP that depends on the changed categories but
  /// not on x0. Returns the parts written and the categories behind each.
  Formulation::Update apply(DataChangeSet changes, const MPCData& d);
  /// Update every part that depends on x0 (d.x0): q if
  /// applyX0WritesCostVector(), the bounds if applyX0WritesBounds().
  void applyX0(const MPCData& d);
  bool applyX0WritesCostVector() const noexcept { return cost_vector_x0_; }
  bool applyX0WritesBounds() const noexcept { return bounds_x0_; }

  /// Before initializeSolver(): write unit placeholders on every cost entry
  /// that must be in the solver's sparsity pattern even if zero now, if the
  /// formulation uses them, and return the categories to re-apply after
  /// initialization (they overwrite the placeholders).
  DataChangeSet writeCostPattern();

  /// Predicted states for the QP solution z.
  void predictStates(const Eigen::Ref<const Eigen::VectorXd>& z,
                     Eigen::Ref<Eigen::VectorXd> x_traj) const noexcept
  {
    formulation_->predictStates(z, x_traj);
  }

private:
  // Add a term and give it its rows (terms are added in row order)
  template <class Term, class... Args>
  const Term& add(QPLayout& layout, Args&&... args);

  struct Entry
  {
    QPTerm* term;
    DataChangeSet deps;
  };

  QP qp_; // declared before formulation_, which keeps a reference
  mutable Eigen::MatrixXd dense_P_; // denseCostMatrix() for sparse storage
  std::unique_ptr<Formulation> formulation_;
  std::vector<std::unique_ptr<QPTerm>> terms_;
  QPRows rows_;
  int num_variables_, num_constraints_;
  // the entry sets of P and A (owned by the formulation and the terms)
  std::vector<QPEntries*> declared_P_, declared_A_;

  // The cost terms are updated together (they accumulate into P and q)
  std::vector<QPTerm*> cost_terms_;
  DataChangeSet cost_matrix_deps_, cost_vector_deps_;
  bool cost_vector_x0_{false};
  // Row terms: matrix parts with dependencies (the others are constant), and
  // vector parts split by whether they depend on x0 (then updated every solve)
  std::vector<Entry> row_matrices_, row_vectors_;
  std::vector<QPTerm*> constant_row_matrices_, row_vectors_x0_;
  bool bounds_x0_{false};
};

} // namespace affine_mpc::detail

#endif // AFFINE_MPC_QP_BUILDER_HPP
