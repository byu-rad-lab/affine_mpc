#ifndef AFFINE_MPC_OSQP_SETTINGS_HPP
#define AFFINE_MPC_OSQP_SETTINGS_HPP

/**
 * @file osqp_settings.hpp
 * @brief Defines OSQPSettings, the settings of the OSQP solver, without
 *   depending on OSQP's headers.
 */

#include <iosfwd>

namespace affine_mpc {

/**
 * @struct OSQPSettings
 * @brief The settings of OSQP 1.0 (its `OSQPSettings` struct), with the same
 *   field names and, by default, OSQP's default values
 *   (`osqp_set_default_settings()`).
 *
 * OSQPSolver::getRecommendedSettings() gives the settings
 * MPC::initializeSolver() uses by default, which differ from OSQP's defaults
 * (tighter tolerances, including eps_dual_inf, warm starting, and no
 * printing). See OSQP's
 * documentation for the meaning of each setting:
 * https://osqp.org/docs/interfaces/solver_settings.html.
 *
 * OSQP stores its on/off switches as integers; they are `bool` here. Fields
 * that are counts or levels, such as scaling, stay `int`.
 */
struct OSQPSettings
{
  /// Linear system solver (OSQP's `osqp_linsys_solver_type`).
  enum class LinsysSolverType
  {
    UnknownSolver = 0, ///< OSQP_UNKNOWN_SOLVER
    DirectSolver,      ///< OSQP_DIRECT_SOLVER (QDLDL in the builtin algebra)
    IndirectSolver     ///< OSQP_INDIRECT_SOLVER (conjugate gradient)
  };

  /// Preconditioner of the conjugate gradient method (OSQP's
  /// `osqp_precond_type`).
  enum class PreconditionerType
  {
    NoPreconditioner = 0,  ///< OSQP_NO_PRECONDITIONER
    DiagonalPreconditioner ///< OSQP_DIAGONAL_PRECONDITIONER (Jacobi)
  };

  /// When OSQP adapts rho (OSQP's `OSQP_ADAPTIVE_RHO_UPDATE_*` constants).
  enum class AdaptiveRhoType
  {
    Disabled = 0, ///< OSQP_ADAPTIVE_RHO_UPDATE_DISABLED: rho stays fixed
    /// OSQP_ADAPTIVE_RHO_UPDATE_ITERATIONS: every adaptive_rho_interval
    /// iterations
    Iterations,
    /// OSQP_ADAPTIVE_RHO_UPDATE_TIME: after adaptive_rho_fraction of the setup
    /// time; needs OSQP built with profiling (OSQP_ENABLE_PROFILING, on when
    /// affine_mpc fetches OSQP), or initialization fails
    Time,
    /// OSQP_ADAPTIVE_RHO_UPDATE_KKT_ERROR: when the KKT error has decreased
    /// to adaptive_rho_fraction of its value at the last adaptation
    KKTError
  };

  // Linear algebra settings
  /// Device identifier (CUDA algebra only).
  int device{0};
  /// Linear system solver.
  LinsysSolverType linsys_solver{LinsysSolverType::DirectSolver};

  // Control settings
  /// Allocate the solution at setup. OSQPSolver reads the solution OSQP
  /// allocates, so its initialization fails if this is zero.
  bool allocate_solution{true};
  /// Print OSQP's progress at every solve.
  bool verbose{true};
  /// Level of detail of profiler annotations.
  int profiler_level{0};
  /// Start each solve from the previous solution.
  bool warm_starting{true};
  /// Number of data scaling (equilibration) iterations; zero disables it.
  int scaling{10};
  /// Polish the ADMM solution: an extra solve after convergence that makes
  /// the solution more accurate where constraints are active.
  bool polishing{false};

  // ADMM parameters
  /// ADMM penalty parameter (step size).
  double rho{0.1};
  /// Whether rho is a vector (different for equality and inequality rows).
  bool rho_is_vec{true};
  /// ADMM regularization parameter.
  double sigma{1e-6};
  /// ADMM relaxation parameter: 1 is standard ADMM, above 1 over-relaxation.
  double alpha{1.6};

  // Conjugate gradient settings (indirect solver)
  /// Maximum number of conjugate gradient iterations per solve.
  int cg_max_iter{20};
  /// Number of consecutive zero conjugate gradient iterations before its
  /// tolerance is halved.
  int cg_tol_reduction{10};
  /// Conjugate gradient tolerance, as a fraction of the ADMM residuals.
  double cg_tol_fraction{0.15};
  /// Preconditioner of the conjugate gradient method.
  PreconditionerType cg_precond{PreconditionerType::DiagonalPreconditioner};

  // Adaptive rho
  /// When rho is adapted.
  AdaptiveRhoType adaptive_rho{AdaptiveRhoType::Iterations};
  /// Iterations between rho adaptations (Iterations); zero chooses it from
  /// check_termination.
  int adaptive_rho_interval{50};
  /// Fraction of the setup time (Time) or of the KKT error at the last
  /// adaptation (KKTError) at which rho is adapted.
  double adaptive_rho_fraction{0.4};
  /// Minimum ratio between a new rho and the current one to change it.
  double adaptive_rho_tolerance{5.0};

  // Termination
  /// Maximum number of ADMM iterations.
  int max_iter{4000};
  /// Absolute tolerance of the residuals.
  double eps_abs{1e-3};
  /// Relative tolerance of the residuals.
  double eps_rel{1e-3};
  /// Primal infeasibility tolerance.
  double eps_prim_inf{1e-4};
  /// Dual infeasibility tolerance.
  double eps_dual_inf{1e-4};
  /// Check termination on the scaled residuals.
  bool scaled_termination{false};
  /// Iterations between termination checks; zero disables the checks.
  int check_termination{25};
  /// Also require the duality gap to be within the tolerances.
  bool check_dualgap{true};
  /// Time limit of a solve (seconds).
  double time_limit{1e10};

  // Polishing
  /// Regularization parameter of polishing.
  double delta{1e-6};
  /// Number of iterative refinement steps of polishing.
  int polish_refine_iter{3};
};

/**
 * @brief Stream a detailed multiline summary of OSQP solver settings.
 * @param os Output stream.
 * @param settings OSQP settings to print.
 * @return Reference to the output stream.
 */
std::ostream& operator<<(std::ostream& os, const OSQPSettings& settings);

} // namespace affine_mpc

#endif // AFFINE_MPC_OSQP_SETTINGS_HPP
