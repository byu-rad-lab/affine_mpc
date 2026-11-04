#ifndef AFFINE_MPC_OSQP_SOLVER_HPP
#define AFFINE_MPC_OSQP_SOLVER_HPP

/**
 * @file osqp_solver.hpp
 * @brief Defines the OSQPSolver class (the QPSolver for OSQP); its settings
 *   are in osqp_settings.hpp.
 */

#include <Eigen/Core>
#include <memory>

#include "affine_mpc/detail/qp.hpp"
#include "affine_mpc/osqp_settings.hpp"
#include "affine_mpc/qp_solver.hpp"
#include "affine_mpc/solve_status.hpp"

namespace affine_mpc {

/**
 * @class OSQPSolver
 * @brief QPSolver adapter for OSQP, the default solver of MPC.
 *
 * Implements QPSolver for both QP storages: the sparse formulation's CSC
 * matrices go to OSQP directly, and the condensed formulation's dense
 * matrices are copied into CSC buffers (their nonzeros at initialization,
 * plus the diagonal of P). OSQP's sparsity pattern is fixed by initialize();
 * later updates change only the values within it.
 *
 * OSQP is a private dependency of the library: its headers and types do not
 * appear in this one.
 *
 * Copy and move operations are deleted to prevent accidental sharing of solver
 * state.
 */
class OSQPSolver : public QPSolver
{
public:
  /// Internal: OSQP's data and the functions that pass matrices to it
  /// directly (defined in the library's sources).
  struct Impl;

  /**
   * @brief Construct an OSQPSolver for a problem of given size, with the
   *   recommended settings (getRecommendedSettings()).
   * @param num_variables Number of QP decision variables
   *   (MPC::getNumQPVariables()).
   * @param num_constraints Number of QP constraints
   *   (MPC::getNumQPConstraints()).
   */
  OSQPSolver(int num_variables, int num_constraints);

  /**
   * @brief Construct an OSQPSolver for a problem of given size, with the
   *   settings initialize() uses.
   * @param num_variables Number of QP decision variables
   *   (MPC::getNumQPVariables()).
   * @param num_constraints Number of QP constraints
   *   (MPC::getNumQPConstraints()).
   * @param settings OSQP solver settings.
   */
  OSQPSolver(int num_variables,
             int num_constraints,
             const OSQPSettings& settings);

  ~OSQPSolver() override;

  /**
   * @brief Returns OSQP's default settings (`OSQPSettings{}`).
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

  /// @brief The settings initialize() uses.
  const OSQPSettings& getSettings() const noexcept { return settings_; }

  /// @name QPSolver interface
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
  /// sets OSQP up with a q too small to affect it and then gives it `qp.q`,
  /// and updateMatrices() holds that q during a matrix update: the cost
  /// scaling depends on P alone, not on x0 or the references. Fails if
  /// OSQPSettings::allocate_solution is zero.
  [[nodiscard]] bool initialize(const detail::QP& qp,
                                const QPBlockLayout& layout) override;
  bool isInitialized() const noexcept override;
  [[nodiscard]] bool updateMatrices(const detail::QP& qp,
                                    bool cost_matrix,
                                    bool constraint_matrix) override;
  [[nodiscard]] bool updateCostVector(const detail::QP& qp) override;
  [[nodiscard]] bool updateBounds(const detail::QP& qp) override;
  [[nodiscard]] SolveStatus solve() override;
  const Eigen::Map<const Eigen::VectorXd>
  getSolutionMap() const noexcept override;
  /// OSQP's dual solution y of the last solve (one value per constraint row,
  /// P z + q + A^T y = 0 at a solution).
  [[nodiscard]] bool
  getDualSolution(Eigen::Ref<Eigen::VectorXd> y) const override;
  /// The residuals are OSQP's unscaled max-norm residuals of its last
  /// iterate (of the polished solution when polishing succeeds), NaN before
  /// the first solve.
  SolveInfo getSolveInfo() const noexcept override;
  /// Zeros OSQP's iterates (osqp_cold_start); the problem data and the
  /// solution are unchanged.
  void coldStart() noexcept override;
  /// Whether P and A have no nonzeros outside the pattern fixed at
  /// initialization (a scan of every entry of dense matrices).
  [[nodiscard]] bool
  isWithinSparsityPattern(const detail::QP& qp) const override;
  ///@}

private:
  std::unique_ptr<Impl> impl_;
  // settings for initialize()
  OSQPSettings settings_;
  // The cost vector OSQP gets whenever it computes its cost scaling (setup
  // and every matrix update), so that scaling depends on P alone (see the
  // constructor)
  Eigen::VectorXd setup_q_;
};

} // namespace affine_mpc

#endif // AFFINE_MPC_OSQP_SOLVER_HPP
