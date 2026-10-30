#ifndef AFFINE_MPC_OSQP_SOLVER_HPP
#define AFFINE_MPC_OSQP_SOLVER_HPP

/**
 * @file osqp_solver.hpp
 * @brief Defines the OSQPSolver class (a wrapper for OSQP c code).
 */

#include <Eigen/Core>
#include <Eigen/SparseCore>
#include <iosfwd>
#include <memory>
#include <osqp.h>
#include <type_traits>
#include <vector>

#include "affine_mpc/detail/qp.hpp"
#include "affine_mpc/qp_solver.hpp"
#include "affine_mpc/solve_status.hpp"

namespace affine_mpc {

// The MPC classes store the QP as double precision Eigen objects, which would
// be silently converted at every update by a single precision OSQP build
// (OSQP_USE_FLOAT). Fail at compile time instead.
static_assert(std::is_same_v<OSQPFloat, double>,
              "affine_mpc requires OSQP built with double precision "
              "(OSQPFloat must be double; do not define OSQP_USE_FLOAT).");

/**
 * @class OSQPSolver
 * @brief RAII wrapper for OSQP C API, manages QP matrices and solver state.
 *
 * Provides initialization, update, and solve routines for quadratic programs
 * in MPC applications. Handles sparse matrix storage and ensures safe usage
 * of OSQP's memory model. Implements QPSolver for both QP storages: the
 * sparse formulation's CSC matrices go to OSQP directly, and the condensed
 * formulation's dense matrices are copied into CSC buffers (their nonzeros at
 * initialization, plus the diagonal of P).
 *
 * Copy and move operations are deleted to prevent accidental sharing of solver
 * state.
 */
class OSQPSolver : public QPSolver
{
public:
  /// Eigen matrix type for OSQPFloat elements.
  typedef Eigen::Matrix<OSQPFloat, Eigen::Dynamic, Eigen::Dynamic> MatrixXF;
  /// Eigen vector type for OSQPFloat elements.
  typedef Eigen::Matrix<OSQPFloat, Eigen::Dynamic, 1> VectorXF;
  /// Eigen vector type for OSQPInt elements.
  typedef Eigen::Matrix<OSQPInt, Eigen::Dynamic, 1> VectorXI;
  /// Eigen sparse matrix type in OSQP's CSC layout (column-major, OSQPInt
  /// indices). Matrices passed to the solver must be compressed.
  typedef Eigen::SparseMatrix<OSQPFloat, Eigen::ColMajor, OSQPInt>
      SparseMatrixXF;

  /**
   * @brief Construct an OSQPSolver for a problem of given size.
   * @param num_variables Number of QP decision variables.
   * @param num_constraints Number of QP constraints.
   */
  OSQPSolver(int num_variables, int num_constraints);

  /**
   * @brief Construct an OSQPSolver for a problem of given size, with the
   *   settings initialize(const detail::QP&, const QPBlockLayout&) uses.
   * @param num_variables Number of QP decision variables
   *   (MPC::getNumQPVariables()).
   * @param num_constraints Number of QP constraints
   *   (MPC::getNumQPConstraints()).
   * @param settings OSQP solver settings.
   */
  OSQPSolver(int num_variables,
             int num_constraints,
             const OSQPSettings& settings);

  ~OSQPSolver() override = default;

  // Delete copy and move operations (C API state should not be transferred)
  OSQPSolver(const OSQPSolver&) = delete;
  OSQPSolver& operator=(const OSQPSolver&) = delete;
  OSQPSolver(OSQPSolver&&) = delete;
  OSQPSolver& operator=(OSQPSolver&&) = delete;

  /**
   * @brief Returns OSQP's default settings.
   * @return OSQPSettings struct with default values.
   */
  static OSQPSettings getDefaultSettings() noexcept;

  /**
   * @brief Returns recommended settings for affine_mpc usage.
   * @param polish_near_boundaries If true, enables OSQP polish for boundary
   *   solutions, which adds computation but improves solution accuracy near
   *   constraint limits.
   * @return OSQPSettings struct with recommended values.
   */
  static OSQPSettings
  getRecommendedSettings(const bool polish_near_boundaries = false) noexcept;

  /**
   * @brief Returns a view of the current solution vector.
   * @return Eigen::Map to the solution vector (will automatically reflect
   *   changes after each solve).
   */
  [[nodiscard]] const Eigen::Map<const VectorXF>
  getSolutionMap() const noexcept override;

  /**
   * @brief Returns diagnostics from the last solve, as reported by OSQP.
   * @return SolveInfo with default values if the solver is not initialized.
   */
  SolveInfo getSolveInfo() const noexcept override;

  /**
   * @brief Solve the QP and write the solution to the provided vector.
   * @param solution Output vector for the solution. Left unchanged if the
   *   solver is not initialized.
   * @return SolveStatus indicating result.
   */
  [[nodiscard]] SolveStatus solve(Eigen::Ref<VectorXF> solution);

  /**
   * @brief Solve the QP and update the internal solution buffer.
   * @return SolveStatus indicating result.
   */
  [[nodiscard]] SolveStatus solve() override;

  /**
   * @brief Zero OSQP's internal iterates so the next solve starts from zero
   *   instead of the previous solution (osqp_cold_start). The problem data and
   *   the solution buffer are unchanged. No-op if not initialized.
   */
  void coldStart() noexcept override;

  /// @name QPSolver interface
  /// The functions MPC calls, which pass the QP in either storage to the
  /// matching dense or sparse functions below.
  ///@{
  const char* getName() const noexcept override { return "OSQP"; }
  detail::QPStorage nativeStorage() const noexcept override
  {
    return detail::QPStorage::Sparse;
  }
  /// Both storages: dense matrices are copied into CSC buffers.
  bool supportsStorage(detail::QPStorage) const noexcept override
  {
    return true;
  }
  /// Initializes with the settings given at construction (the recommended
  /// settings for the constructor without settings). OSQP computes its cost
  /// scaling from P and q at setup and again at every matrix update, so this
  /// sets OSQP up with q = 0 and then gives it `qp.q`, and updateMatrices()
  /// holds q = 0 during a matrix update: the cost scaling depends on P alone,
  /// not on x0 or the references.
  [[nodiscard]] bool initialize(const detail::QP& qp,
                                const QPBlockLayout& layout) override;
  bool isInitialized() const noexcept override { return initialized_; }
  [[nodiscard]] bool updateMatrices(const detail::QP& qp,
                                    bool cost_matrix,
                                    bool constraint_matrix) override;
  [[nodiscard]] bool updateCostVector(const detail::QP& qp) override;
  [[nodiscard]] bool updateBounds(const detail::QP& qp) override;
  [[nodiscard]] bool
  isWithinSparsityPattern(const detail::QP& qp) const override;
  ///@}

  /// @brief The settings initialize(const detail::QP&, const QPBlockLayout&)
  ///   uses.
  const OSQPSettings& getSettings() const noexcept { return settings_; }

  /**
   * @brief Initialize the OSQP solver with QP matrices and settings.
   * @param P Quadratic cost matrix (upper triangular).
   * @param A Constraint matrix.
   * @param q Linear cost vector.
   * @param l Lower bounds vector.
   * @param u Upper bounds vector.
   * @param settings OSQP solver settings.
   * @return True if initialization succeeds, false otherwise.
   */
  [[nodiscard]] bool initialize(const Eigen::Ref<const MatrixXF>& P,
                                const Eigen::Ref<const MatrixXF>& A,
                                const Eigen::Ref<const VectorXF>& q,
                                const Eigen::Ref<const VectorXF>& l,
                                const Eigen::Ref<const VectorXF>& u,
                                const OSQPSettings& settings);

  /**
   * @brief Initialize the OSQP solver with sparse QP matrices and settings.
   *
   *   The solver's sparsity pattern is the stored entries (the matrices'
   *   structure), whatever their values now: unlike for dense matrices, a
   *   stored entry that is zero now may become nonzero later. Later updates
   *   must pass matrices with the same stored entries; their values go to
   *   OSQP directly (P with only its upper triangle stored) or are copied
   *   without scanning the matrices.
   * @param P Quadratic cost matrix: the upper triangle (entries below the
   *   diagonal are ignored), with every diagonal entry stored (possibly zero).
   * @param A Constraint matrix.
   * @param q Linear cost vector.
   * @param l Lower bounds vector.
   * @param u Upper bounds vector.
   * @param settings OSQP solver settings.
   * @return True if initialization succeeds, false otherwise (also if P or A
   *   is not compressed or P does not store its whole diagonal).
   */
  [[nodiscard]] bool initialize(const SparseMatrixXF& P,
                                const SparseMatrixXF& A,
                                const Eigen::Ref<const VectorXF>& q,
                                const Eigen::Ref<const VectorXF>& l,
                                const Eigen::Ref<const VectorXF>& u,
                                const OSQPSettings& settings);

  /**
   * @brief Update the quadratic cost matrix P (values only, sparsity fixed).
   * @param P New cost matrix (only elements matching original sparsity
   *   structure are updated).
   * @return True if update succeeds, false otherwise.
   */
  [[nodiscard]] bool updateCostMatrix(const Eigen::Ref<const MatrixXF>& P);

  /**
   * @brief Update the constraint matrix A (values only, sparsity fixed).
   * @param A New constraint matrix (only elements matching original sparsity
   *   structure are updated).
   * @return True if update succeeds, false otherwise.
   */
  [[nodiscard]] bool
  updateConstraintMatrix(const Eigen::Ref<const MatrixXF>& A);

  /**
   * @brief Update both P and A in a single OSQP update (values only, sparsity
   *   fixed).
   *
   *   OSQP rescales its data and refactors on every matrix update, so this
   *   does that once instead of twice.
   * @return True if update succeeds, false otherwise.
   */
  [[nodiscard]] bool updateMatrices(const Eigen::Ref<const MatrixXF>& P,
                                    const Eigen::Ref<const MatrixXF>& A);

  /**
   * @brief Sparse versions of updateCostMatrix(), updateConstraintMatrix(),
   *   and updateMatrices(), for a solver initialized with sparse matrices.
   *
   *   The matrices must have the stored entries (not only the pattern) of the
   *   ones passed to initialize(); only their values may differ.
   * @return True if the update succeeds, false otherwise (also if the solver
   *   was initialized with dense matrices).
   */
  [[nodiscard]] bool updateCostMatrix(const SparseMatrixXF& P);
  /// @copydoc updateCostMatrix(const SparseMatrixXF&)
  [[nodiscard]] bool updateConstraintMatrix(const SparseMatrixXF& A);
  /// @copydoc updateCostMatrix(const SparseMatrixXF&)
  [[nodiscard]] bool updateMatrices(const SparseMatrixXF& P,
                                    const SparseMatrixXF& A);

  /**
   * @brief Update the linear cost vector q.
   * @param q New cost vector.
   * @return True if update succeeds, false otherwise.
   */
  [[nodiscard]] bool updateCostVector(const Eigen::Ref<const VectorXF>& q);

  /**
   * @brief Update the lower and upper bounds vectors.
   * @param l New lower bounds.
   * @param u New upper bounds.
   * @return True if update succeeds, false otherwise.
   */
  [[nodiscard]] bool updateBounds(const Eigen::Ref<const VectorXF>& l,
                                  const Eigen::Ref<const VectorXF>& u);

  /**
   * @brief Check whether P and A fit the sparsity pattern fixed at
   *   initialization (no nonzeros where the initial matrices had zeros).
   *
   *   Intended for debugging: it scans every entry of both dense matrices.
   *   The update functions silently skip entries outside the pattern.
   * @param P Cost matrix (only the upper triangle is read).
   * @param A Constraint matrix.
   * @return True if both fit the pattern. False if either has a nonzero
   *   outside it, or if the solver is not initialized.
   */
  [[nodiscard]] bool
  isWithinSparsityPattern(const Eigen::Ref<const MatrixXF>& P,
                          const Eigen::Ref<const MatrixXF>& A) const;

  /**
   * @brief Sparse version of isWithinSparsityPattern(), for a solver
   *   initialized with sparse matrices: whether every nonzero stored value
   *   (in P's upper triangle) is in the pattern, which holds for matrices
   *   with the stored entries of the ones passed to initialize().
   * @return False if a nonzero value is outside the pattern, if the solver is
   *   not initialized, or if it was initialized with dense matrices.
   */
  [[nodiscard]] bool isWithinSparsityPattern(const SparseMatrixXF& P,
                                             const SparseMatrixXF& A) const;

private:
  /**
   * @brief Counts the number of non-zeros in the upper triangle of a matrix.
   * @param mat Input matrix.
   * @return Number of non-zero elements in upper triangle.
   */
  int countUpperTriangle(const Eigen::Ref<const MatrixXF>& mat);

  // Copy the values of P and A within the stored sparsity pattern into the
  // CSC value buffers passed to OSQP.
  void fillCostValues(const Eigen::Ref<const MatrixXF>& P);
  void fillConstraintValues(const Eigen::Ref<const MatrixXF>& A);
  // The value array of a sparse P (A) as OSQP's values: the matrix's own
  // values if they are OSQP's in order, else the buffer filled from them
  const OSQPFloat* costValues(const SparseMatrixXF& P);
  const OSQPFloat* constraintValues(const SparseMatrixXF& A);
  // Set up after the CSC buffers are filled
  bool setup(const Eigen::Ref<const VectorXF>& q,
             const Eigen::Ref<const VectorXF>& l,
             const Eigen::Ref<const VectorXF>& u,
             const OSQPSettings& settings);

  /**
   * @brief Initializes the cost matrix storage for OSQP.
   * @param P Quadratic cost matrix.
   */
  void initializeCostMatrix(const Eigen::Ref<const MatrixXF>& P);

  /**
   * @brief Initializes the constraint matrix storage for OSQP.
   * @param A Constraint matrix.
   */
  void initializeConstraintMatrix(const Eigen::Ref<const MatrixXF>& A);

  // The sparse versions: keep every stored entry, zero or not (of P, those on
  // or above the diagonal); false if P does not store its whole diagonal
  bool initializeCostMatrix(const SparseMatrixXF& P);
  void initializeConstraintMatrix(const SparseMatrixXF& A);

  // Copy l and u into l_buf_ and u_buf_ with infinities clamped to OSQP_INFTY.
  void clampBounds(const Eigen::Ref<const VectorXF>& l,
                   const Eigen::Ref<const VectorXF>& u);

  // Choosing to modernize even though OSQP documentation uses raw pointers
  std::unique_ptr<::OSQPSolver, decltype(&osqp_cleanup)> solver_;
  std::unique_ptr<OSQPCscMatrix, decltype(&OSQPCscMatrix_free)> P_;
  std::unique_ptr<OSQPCscMatrix, decltype(&OSQPCscMatrix_free)> A_;
  bool initialized_;
  bool P_is_set_;
  bool A_is_set_;
  // The kind of matrices passed to initialize() (the updates take the same)
  bool sparse_input_{false};
  // Where each of OSQP's values of P (A) is in the matrices passed to the
  // updates, computed once in initialize(): dense, the position
  // col * outer stride + row; sparse, the index in the value array. With them
  // an update copies the values without walking the pattern.
  std::vector<OSQPInt> P_src_, A_src_;
  // dense: the outer strides the positions were computed for (an update with
  // other strides walks the pattern instead)
  Eigen::Index P_stride_{0}, A_stride_{0};
  // OSQP only treats bounds beyond OSQP_INFTY as unbounded; with IEEE
  // infinities it can not detect convergence, so bounds are clamped here.
  VectorXF l_buf_, u_buf_;
  // A zero cost vector: OSQP computes its cost scaling from q at setup and at
  // every matrix update, so the QP interface passes q = 0 then (see
  // initialize(const detail::QP&, const QPBlockLayout&))
  VectorXF zero_q_;
  // settings for initialize(const detail::QP&, const QPBlockLayout&)
  OSQPSettings settings_;

protected:          // Not private for unit tests
  const OSQPInt n_; ///< Number of decision variables.
  const OSQPInt m_; ///< Number of constraints.
  OSQPInt P_nnz_;   ///< Number of non-zeros in P.
  OSQPInt A_nnz_;   ///< Number of non-zeros in A.
  VectorXF P_x_;    ///< Value buffer for P.
  VectorXF A_x_;    ///< Value buffer for A.
  VectorXI P_i_;    ///< Row indices for P.
  VectorXI A_i_;    ///< Row indices for A.
  VectorXI P_p_;    ///< Column pointers for P.
  VectorXI A_p_;    ///< Column pointers for A.
  /// Sparse input only: whether OSQP's values of P (A) are the matrix's own
  /// stored values (every stored entry kept), which then go to OSQP without a
  /// copy (P_x_ and A_x_ then hold the values of initialize() only).
  bool P_direct_{false}, A_direct_{false};
};

/**
 * @brief Stream a detailed multiline summary of OSQP solver settings.
 * @param os Output stream.
 * @param settings OSQP settings to print.
 * @return Reference to the output stream.
 */
std::ostream& operator<<(std::ostream& os, const OSQPSettings& settings);

} // namespace affine_mpc

#endif // AFFINE_MPC_OSQP_SOLVER_HPP
