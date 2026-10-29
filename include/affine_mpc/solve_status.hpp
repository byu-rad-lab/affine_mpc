#ifndef AFFINE_MPC_SOLVE_STATUS_HPP
#define AFFINE_MPC_SOLVE_STATUS_HPP

/**
 * @file solve_status.hpp
 * @brief Defines the SolveStatus enum and the SolveInfo diagnostics of a
 *   solve.
 */

#include <iosfwd>

namespace affine_mpc {

/**
 * @enum SolveStatus
 * @brief High-level status codes for QP solver results.
 *
 * Each QP solver maps its own status codes to this simplified set of
 * categories for MPC usage.
 */
enum class SolveStatus
{
  Success,          ///< QP solved successfully
  NotInitialized,   ///< Solver not initialized
  SolvedInaccurate, ///< QP solved but solution may be inaccurate
  PrimalInfeasible, ///< Problem is primal infeasible
  DualInfeasible,   ///< Problem is dual infeasible
  MaxIterReached,   ///< The solver hit its iteration limit
  TimeLimitReached, ///< The solver hit its time limit
  OtherFailure,     ///< Any other solver error
  UpdateFailed      ///< The solver rejected a data update; QP not solved
};

/**
 * @struct SolveInfo
 * @brief Solver diagnostics from the most recent solve.
 *
 * `mpc_solve_time - qp_solve_time` is the time the library spends outside the
 * QP solver (QP assembly, data updates, and refactorization). Work done in
 * MPC::prepare() is reported separately in `prepare_time`.
 */
struct SolveInfo
{
  /// Result of the solve. NotInitialized before initializeSolver(), and
  /// OtherFailure after initialization but before the first solve.
  SolveStatus status{SolveStatus::NotInitialized};

  /// Number of solver iterations.
  int iterations{0};

  /// QP objective value 1/2 z^T P z + q^T z at the solution. This omits
  /// constant terms of the MPC cost (they depend on x0 and the references), so
  /// it is not the full MPC cost.
  double objective{0.0};

  /// Time the QP solver spent solving (seconds). For OSQP, the solve
  /// including polishing, zero when OSQP is built without profiling.
  double qp_solve_time{0.0};

  /// Duration of the whole last MPC::solve() call after initialization,
  /// including QP assembly, data updates, refactorization, and the solver
  /// (seconds). Measured with `std::chrono::steady_clock`, so it does not
  /// depend on the solver's profiling.
  double mpc_solve_time{0.0};

  /// Total duration of the MPC::prepare() calls between the previous
  /// solve() and this one (seconds), zero if there were none. This is work
  /// done before the measurement, so `prepare_time + mpc_solve_time` is the
  /// MPC's work for the step while `mpc_solve_time` alone is its latency.
  double prepare_time{0.0};
};

/**
 * @brief Stream output operator for SolveStatus.
 * @param os Output stream.
 * @param status SolveStatus value.
 * @return Reference to the output stream.
 */
std::ostream& operator<<(std::ostream& os, const SolveStatus status);

/**
 * @brief Stream a one-line summary of SolveInfo.
 * @param os Output stream.
 * @param info SolveInfo value.
 * @return Reference to the output stream.
 */
std::ostream& operator<<(std::ostream& os, const SolveInfo& info);

} // namespace affine_mpc

#endif // AFFINE_MPC_SOLVE_STATUS_HPP
