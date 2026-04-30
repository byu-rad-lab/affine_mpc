#ifndef AFFINE_MPC_OSQP_SOLVER_HPP
#define AFFINE_MPC_OSQP_SOLVER_HPP

#include <Eigen/Core>
#include <memory>
#include <osqp.h>

#include "affine_mpc/solve_status.hpp"

/**
 * @file osqp_solver.hpp
 * @brief Defines the OSQPSolver class (a wrapper for OSQP c code).
 */

namespace affine_mpc {

/**
 * @class OSQPSolver
 * @brief RAII wrapper for OSQP C API, manages QP matrices and solver state.
 *
 * Provides initialization, update, and solve routines for quadratic programs
 * in MPC applications. Handles sparse matrix storage and ensures safe usage
 * of OSQP's memory model.
 *
 * Copy and move operations are deleted to prevent accidental sharing of solver
 * state.
 */
class OSQPSolver
{
public:
  /// Eigen matrix type for OSQPFloat elements.
  typedef Eigen::Matrix<OSQPFloat, Eigen::Dynamic, Eigen::Dynamic> MatrixXF;
  /// Eigen vector type for OSQPFloat elements.
  typedef Eigen::Matrix<OSQPFloat, Eigen::Dynamic, 1> VectorXF;
  /// Eigen vector type for OSQPInt elements.
  typedef Eigen::Matrix<OSQPInt, Eigen::Dynamic, 1> VectorXI;

  /**
   * @brief Construct an OSQPSolver for a problem of given size.
   * @param num_variables Number of QP decision variables.
   * @param num_constraints Number of QP constraints.
   */
  OSQPSolver(int num_variables, int num_constraints);

  virtual ~OSQPSolver() = default;

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
  getSolutionMap() const noexcept;

  /**
   * @brief Returns the time taken (seconds) by the last solve, as reported by
   *   OSQP.
   * @return Solve time in seconds.
   */
  OSQPFloat getSolveTime() const noexcept;

  /**
   * @brief Solve the QP and write the solution to the provided vector.
   * @param solution Output vector for the solution.
   * @return SolveStatus indicating result.
   */
  [[nodiscard]] SolveStatus solve(Eigen::Ref<VectorXF> solution);

  /**
   * @brief Solve the QP and update the internal solution buffer.
   * @return SolveStatus indicating result.
   */
  [[nodiscard]] SolveStatus solve();

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

private:
  /**
   * @brief Counts the number of non-zeros in the upper triangle of a matrix.
   * @param mat Input matrix.
   * @return Number of non-zero elements in upper triangle.
   */
  int countUpperTriangle(const Eigen::Ref<const MatrixXF>& mat);

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

  // Choosing to modernize even though OSQP documentation uses raw pointers
  std::unique_ptr<::OSQPSolver, decltype(&osqp_cleanup)> solver_;
  std::unique_ptr<OSQPCscMatrix, decltype(&OSQPCscMatrix_free)> P_;
  std::unique_ptr<OSQPCscMatrix, decltype(&OSQPCscMatrix_free)> A_;
  bool initialized_;
  bool P_is_set_;
  bool A_is_set_;

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
};

} // namespace affine_mpc

#endif // AFFINE_MPC_OSQP_SOLVER_HPP
