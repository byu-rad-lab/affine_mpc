#ifndef AFFINE_MPC_QP_SOLVER_HPP
#define AFFINE_MPC_QP_SOLVER_HPP

/**
 * @file qp_solver.hpp
 * @brief Defines the QPSolver interface that MPC solves its QP through.
 */

#include <Eigen/Core>
#include <vector>

#include "affine_mpc/detail/qp.hpp"
#include "affine_mpc/quadratic_program.hpp"
#include "affine_mpc/solve_status.hpp"

namespace affine_mpc {

/**
 * @struct QPBlockLayout
 * @brief The named blocks of QP variables and constraint rows, in order (see
 *   MPC::getQPVariableBlocks() and MPC::getQPConstraintBlocks()).
 */
struct QPBlockLayout
{
  std::vector<QPBlock> variables;   ///< blocks of the variables z
  std::vector<QPBlock> constraints; ///< blocks of the rows of A
};

/**
 * @class QPSolver
 * @brief Interface of the QP solvers an MPC can use (OSQPSolver, and
 *   DAQPSolver when built with it).
 *
 * The solver receives the MPC's QP
 *   minimize 1/2 z^T P z + q^T z subject to l <= A z <= u
 * in the storage the formulation chose (dense for the condensed formulation,
 * compressed sparse column for the sparse one) and translates it to its own
 * form. MPC::initializeSolver() calls initialize() once; afterwards MPC sends
 * only the parts that changed (update...()) before each solve().
 *
 * The QP passed to initialize() is the one passed to every later call, and
 * it stays at the same address while the solver is used, so a solver may
 * keep pointers into it.
 *
 * @warning Experimental: implementing QPSolver outside the library is
 *   supported, but this interface (its methods, QPBlockLayout, and the
 *   internal detail::QP and detail::QPStorage types it takes) may change in
 *   any minor release. Using OSQPSolver or DAQPSolver through
 *   MPC::initializeSolver() is stable.
 *
 * Copy and move operations are deleted, since solvers own C solver state.
 */
class QPSolver
{
public:
  virtual ~QPSolver() = default;

  QPSolver(const QPSolver&) = delete;
  QPSolver& operator=(const QPSolver&) = delete;
  QPSolver(QPSolver&&) = delete;
  QPSolver& operator=(QPSolver&&) = delete;

  /// @brief Name of the solver (e.g. "OSQP").
  virtual const char* getName() const noexcept = 0;

  /// @brief The QP storage the solver uses without converting it.
  virtual detail::QPStorage nativeStorage() const noexcept = 0;

  /// @brief Whether initialize() accepts a QP in the given storage (by
  ///   default only the native one).
  virtual bool supportsStorage(const detail::QPStorage storage) const noexcept
  {
    return storage == nativeStorage();
  }

  /**
   * @brief Set the solver up for the QP (sizes, structure, and values).
   * @param qp The QP, in a storage supportsStorage() accepts.
   * @param layout The named variable and constraint blocks of the QP.
   * @return True if the setup succeeded.
   */
  [[nodiscard]] virtual bool initialize(const detail::QP& qp,
                                        const QPBlockLayout& layout) = 0;

  /// @brief Whether initialize() has succeeded.
  virtual bool isInitialized() const noexcept = 0;

  /**
   * @brief Take new values of P and/or A from the QP.
   * @return False if the solver rejected the update or is not initialized.
   */
  [[nodiscard]] virtual bool updateMatrices(const detail::QP& qp,
                                            bool cost_matrix,
                                            bool constraint_matrix) = 0;

  /**
   * @brief Take new values of q from the QP.
   * @return False if the solver rejected the update or is not initialized.
   */
  [[nodiscard]] virtual bool updateCostVector(const detail::QP& qp) = 0;

  /**
   * @brief Take new values of l and u from the QP.
   * @return False if the solver rejected the update or is not initialized.
   */
  [[nodiscard]] virtual bool updateBounds(const detail::QP& qp) = 0;

  /**
   * @brief Solve the QP with the data of the last updates.
   * @return The result, NotInitialized if the solver is not initialized.
   */
  [[nodiscard]] virtual SolveStatus solve() = 0;

  /**
   * @brief A view of the solution of the last solve().
   *
   *   The memory it views is fixed by initialize() and owned by the solver,
   *   so the view stays valid (and reflects later solves) while the solver
   *   exists. Empty before initialization.
   */
  virtual const Eigen::Map<const Eigen::VectorXd>
  getSolutionMap() const noexcept = 0;

  /// @brief Diagnostics of the last solve() (SolveInfo's solver fields).
  virtual SolveInfo getSolveInfo() const noexcept = 0;

  /// @brief Make the next solve() start without the previous solve's
  ///   information (warm start). No-op if not initialized.
  virtual void coldStart() noexcept = 0;

  /**
   * @brief Whether the QP's matrices fit the structure fixed by initialize()
   *   (debugging aid; may scan the matrices).
   */
  [[nodiscard]] virtual bool
  isWithinSparsityPattern(const detail::QP& qp) const = 0;

protected:
  QPSolver() = default;
};

} // namespace affine_mpc

#endif // AFFINE_MPC_QP_SOLVER_HPP
