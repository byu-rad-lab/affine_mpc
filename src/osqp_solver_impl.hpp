#ifndef AFFINE_MPC_OSQP_SOLVER_IMPL_HPP
#define AFFINE_MPC_OSQP_SOLVER_IMPL_HPP

// Internal: the parts of OSQPSolver that use OSQP's types, which stay out of
// the installed headers.

#include <Eigen/Core>
#include <Eigen/SparseCore>
#include <memory>
#include <osqp.h>
#include <vector>

#include "affine_mpc/detail/qp.hpp"
#include "affine_mpc/osqp_settings.hpp"
#include "affine_mpc/osqp_solver.hpp"
#include "affine_mpc/solve_status.hpp"

namespace affine_mpc {

/// OSQP's settings struct with the values of `settings`.
::OSQPSettings toOSQP(const OSQPSettings& settings) noexcept;

/**
 * RAII wrapper of OSQP's C API that takes the QP matrices directly: it builds
 * the CSC matrices OSQP keeps (from dense matrices, their nonzeros plus the
 * diagonal of P; from sparse ones, their stored entries), and passes later
 * values within that pattern. OSQPSolver adapts it to QPSolver; tests use it
 * on its own.
 *
 * The sparse matrices are the QP's (detail::QPIndex indices). Their indices
 * are copied into OSQP's index type at initialization, so the index types may
 * differ (a system OSQP built without OSQP_USE_LONG); the values go to OSQP
 * without a copy when it keeps every stored entry.
 */
struct OSQPSolver::Impl
{
  /// Eigen matrix type for OSQPFloat elements.
  using MatrixXF = Eigen::Matrix<OSQPFloat, Eigen::Dynamic, Eigen::Dynamic>;
  /// Eigen vector type for OSQPFloat elements.
  using VectorXF = Eigen::Matrix<OSQPFloat, Eigen::Dynamic, 1>;
  /// Eigen vector type for OSQPInt elements.
  using VectorXI = Eigen::Matrix<OSQPInt, Eigen::Dynamic, 1>;
  /// The QP's CSC matrices. Matrices passed to the solver must be compressed.
  using SparseMatrixXF = detail::QP::SparseMatrix;

  /**
   * @brief Construct for a problem of given size.
   * @param num_variables Number of QP decision variables.
   * @param num_constraints Number of QP constraints.
   */
  Impl(int num_variables, int num_constraints);

  Impl(const Impl&) = delete;
  Impl& operator=(const Impl&) = delete;
  Impl(Impl&&) = delete;
  Impl& operator=(Impl&&) = delete;

  bool isInitialized() const noexcept { return initialized_; }

  /// A view of the current solution (empty before initialization).
  const Eigen::Map<const VectorXF> getSolutionMap() const noexcept;

  /// OSQP's dual solution of the last solve; false, leaving y unchanged, if
  /// not initialized.
  [[nodiscard]] bool getDualSolution(Eigen::Ref<Eigen::VectorXd> y) const;

  /// Diagnostics of the last solve (defaults if not initialized).
  SolveInfo getSolveInfo() const noexcept;

  /// Solve and write the solution (left unchanged if not initialized).
  [[nodiscard]] SolveStatus solve(Eigen::Ref<VectorXF> solution);

  /// Solve, updating OSQP's solution buffer.
  [[nodiscard]] SolveStatus solve();

  /// osqp_cold_start; no-op if not initialized.
  void coldStart() noexcept;

  /**
   * @brief Initialize OSQP with dense QP matrices: its pattern is the
   *   nonzeros of A and of P's upper triangle, plus P's diagonal.
   * @return True if initialization succeeds (false also if
   *   settings.allocate_solution is zero).
   */
  [[nodiscard]] bool initialize(const Eigen::Ref<const MatrixXF>& P,
                                const Eigen::Ref<const MatrixXF>& A,
                                const Eigen::Ref<const VectorXF>& q,
                                const Eigen::Ref<const VectorXF>& l,
                                const Eigen::Ref<const VectorXF>& u,
                                const OSQPSettings& settings);

  /**
   * @brief Initialize OSQP with sparse QP matrices: its pattern is the stored
   *   entries (of P, those on or above the diagonal), whatever their values
   *   now. Later updates must pass matrices with the same stored entries;
   *   their values go to OSQP directly or are copied without scanning the
   *   matrices.
   * @param P Cost matrix: the upper triangle (entries below the diagonal are
   *   ignored), with every diagonal entry stored (possibly zero).
   * @return True if initialization succeeds, false otherwise (also if P or A
   *   is not compressed or P does not store its whole diagonal).
   */
  [[nodiscard]] bool initialize(const SparseMatrixXF& P,
                                const SparseMatrixXF& A,
                                const Eigen::Ref<const VectorXF>& q,
                                const Eigen::Ref<const VectorXF>& l,
                                const Eigen::Ref<const VectorXF>& u,
                                const OSQPSettings& settings);

  // Dense updates of P, A, or both (one OSQP update, so one rescaling and
  // refactorization): values only, entries outside the pattern are skipped.
  // False if not initialized or initialized with sparse matrices.
  [[nodiscard]] bool updateCostMatrix(const Eigen::Ref<const MatrixXF>& P);
  [[nodiscard]] bool
  updateConstraintMatrix(const Eigen::Ref<const MatrixXF>& A);
  [[nodiscard]] bool updateMatrices(const Eigen::Ref<const MatrixXF>& P,
                                    const Eigen::Ref<const MatrixXF>& A);

  // Sparse updates, with the stored entries of the matrices passed to
  // initialize(). False if not initialized or initialized with dense
  // matrices.
  [[nodiscard]] bool updateCostMatrix(const SparseMatrixXF& P);
  [[nodiscard]] bool updateConstraintMatrix(const SparseMatrixXF& A);
  [[nodiscard]] bool updateMatrices(const SparseMatrixXF& P,
                                    const SparseMatrixXF& A);

  [[nodiscard]] bool updateCostVector(const Eigen::Ref<const VectorXF>& q);
  [[nodiscard]] bool updateBounds(const Eigen::Ref<const VectorXF>& l,
                                  const Eigen::Ref<const VectorXF>& u);

  /// Dense: whether P (upper triangle) and A have no nonzeros outside the
  /// pattern (scans every entry). False if not initialized or initialized
  /// with sparse matrices.
  [[nodiscard]] bool
  isWithinSparsityPattern(const Eigen::Ref<const MatrixXF>& P,
                          const Eigen::Ref<const MatrixXF>& A) const;
  /// Sparse: whether every nonzero stored value (in P's upper triangle) is
  /// in the pattern. False if not initialized or initialized with dense
  /// matrices.
  [[nodiscard]] bool isWithinSparsityPattern(const SparseMatrixXF& P,
                                             const SparseMatrixXF& A) const;

  // The data, read by tests

  const OSQPInt n_;  ///< Number of decision variables.
  const OSQPInt m_;  ///< Number of constraints.
  OSQPInt P_nnz_{0}; ///< Number of entries of OSQP's P.
  OSQPInt A_nnz_{0}; ///< Number of entries of OSQP's A.
  VectorXF P_x_;     ///< Value buffer for P.
  VectorXF A_x_;     ///< Value buffer for A.
  VectorXI P_i_;     ///< Row indices for P.
  VectorXI A_i_;     ///< Row indices for A.
  VectorXI P_p_;     ///< Column pointers for P.
  VectorXI A_p_;     ///< Column pointers for A.
  /// Sparse input only: whether OSQP's values of P (A) are the matrix's own
  /// stored values (every stored entry kept), which then go to OSQP without a
  /// copy (P_x_ and A_x_ then hold the values of initialize() only).
  bool P_direct_{false}, A_direct_{false};

private:
  // Number of entries of the upper triangle of P kept: its nonzeros and its
  // diagonal
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

  // Build the CSC buffers from dense matrices
  void initializeCostMatrix(const Eigen::Ref<const MatrixXF>& P);
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
  bool initialized_{false};
  bool P_is_set_{false};
  bool A_is_set_{false};
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
};

} // namespace affine_mpc

#endif // AFFINE_MPC_OSQP_SOLVER_IMPL_HPP
