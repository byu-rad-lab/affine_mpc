#ifndef AFFINE_MPC_DAQP_SETTINGS_HPP
#define AFFINE_MPC_DAQP_SETTINGS_HPP

/**
 * @file daqp_settings.hpp
 * @brief Defines DAQPSettings and hasDAQP(), which are available in every
 *   build, so code can choose DAQP with MPC::initializeSolver(const
 *   DAQPSettings&, ModelSparsity) without depending on how the library was
 *   built.
 */

namespace affine_mpc {

/**
 * @struct DAQPSettings
 * @brief The DAQP settings DAQPSolver exposes; the defaults are DAQP's.
 */
struct DAQPSettings
{
  /// Tolerance for primal infeasibility: constraints may be violated by up to
  /// this much.
  double primal_tol{1e-6};
  /// Tolerance for dual infeasibility (the sign of the multipliers).
  double dual_tol{1e-12};
  /// Maximum number of active-set iterations.
  int iter_limit{10000};
  /// Time limit for a solve (seconds); zero or negative for none. Needs DAQP
  /// built with profiling (the fetched DAQP is).
  double time_limit{0.0};
  /**
   * Proximal-point regularization, which lets DAQP solve QPs whose cost
   * matrix P is only positive semidefinite (singular): it solves a sequence
   * of QPs with P + eps I. Negative (the default, -1e-6): automatic, used only
   * when P is singular or too ill-conditioned to factorize, starting from
   * |eps_prox|. Positive: always used, with this eps. Zero: never used, so a
   * singular P makes initialization fail (MPC::initializeSolver() returns
   * false) or, after a cost matrix update, the next solve return
   * UpdateFailed.
   */
  double eps_prox{-1e-6};
};

/**
 * @brief Whether the library was built with DAQP (`-DAFFINE_MPC_WITH_DAQP=ON`),
 *   so that DAQPSolver and MPC::initializeSolver(const DAQPSettings&,
 *   ModelSparsity) can be used.
 *
 *   It reports how the library was built, not how the calling code is
 *   compiled; code linking to it through CMake also sees
 *   `AFFINE_MPC_WITH_DAQP` defined when this is true.
 */
bool hasDAQP() noexcept;

} // namespace affine_mpc

#endif // AFFINE_MPC_DAQP_SETTINGS_HPP
