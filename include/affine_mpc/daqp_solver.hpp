#ifndef AFFINE_MPC_DAQP_SOLVER_HPP
#define AFFINE_MPC_DAQP_SOLVER_HPP

/**
 * @file daqp_solver.hpp
 * @brief Defines the DAQPSolver class (the DAQP dense active-set solver for
 *   the condensed formulation); its settings are in daqp_settings.hpp.
 *
 * Available only when the library is built with
 * `-DAFFINE_MPC_WITH_DAQP=ON`, which also defines `AFFINE_MPC_WITH_DAQP` for
 * code linking to it.
 */

#ifndef AFFINE_MPC_WITH_DAQP
#error "affine_mpc was built without DAQP (needs -DAFFINE_MPC_WITH_DAQP=ON)"
#endif

#include <Eigen/Core>
#include <memory>

#include "affine_mpc/daqp_settings.hpp"
#include "affine_mpc/detail/qp.hpp"
#include "affine_mpc/qp_solver.hpp"
#include "affine_mpc/solve_status.hpp"

namespace affine_mpc {

/**
 * @class DAQPSolver
 * @brief QPSolver adapter for DAQP, a dense dual active-set solver, for the
 *   condensed formulation only.
 *
 * DAQP solves small dense QPs exactly (to its tolerances) and is often
 * much faster than OSQP on them. It takes
 *   minimize 1/2 z^T P z + q^T z
 *   subject to lb <= z <= ub (simple bounds), bl <= A_g z <= bu,
 * so initialize() sorts the rows of the MPC's constraint matrix: rows with a
 * single nonzero (input bounds on control points, the initial slew, slack
 * bounds) become simple bounds on their variable, and the other rows are
 * copied row-major into DAQP's general constraints. This split is fixed at
 * initialization, like the sparsity pattern of OSQP: a row with a single
 * nonzero then must keep it. The dense cost matrix P and the cost vector q
 * are read in place, without copies.
 *
 * Updates of P or A refactorize immediately (so MPC::prepare() does that
 * work); vector updates are applied at the next solve(). Warm start keeps the
 * working set (the active constraints) of the previous solve; coldStart()
 * empties it. solve() does not allocate.
 *
 * SolveInfo::qp_solve_time is DAQP's solve time: the active-set iterations,
 * excluding the forming of DAQP's internal problem from new data (factorizing
 * P and transforming A, q, and the bounds), which is part of mpc_solve_time
 * (or prepare_time for matrix updates applied by MPC::prepare()).
 *
 * Status mapping: DAQP's optimal and soft-optimal results are Success,
 * optimal-inexact SolvedInaccurate, infeasible PrimalInfeasible, unbounded
 * DualInfeasible, the iteration and time limits MaxIterReached and
 * TimeLimitReached, and every other result OtherFailure.
 */
class DAQPSolver final : public QPSolver
{
public:
  /// @brief Construct with the given settings (DAQP's defaults if omitted).
  explicit DAQPSolver(const DAQPSettings& settings = DAQPSettings{});
  ~DAQPSolver() override;

  const char* getName() const noexcept override { return "DAQP"; }
  /// Dense: the condensed formulation's storage (the only one supported).
  detail::QPStorage nativeStorage() const noexcept override
  {
    return detail::QPStorage::Dense;
  }
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
  /// DAQP's multipliers in row order: the multiplier of a variable bound
  /// goes to the simple row that sets it (divided by its coefficient), and
  /// the other simple rows of that variable get zero.
  [[nodiscard]] bool
  getDualSolution(Eigen::Ref<Eigen::VectorXd> y) const override;
  /// The residuals are NaN: DAQP does not report them.
  SolveInfo getSolveInfo() const noexcept override;
  void coldStart() noexcept override;
  /// Whether every row taken as a simple bound still has a single nonzero
  /// (the dense P and the general rows have no pattern).
  [[nodiscard]] bool
  isWithinSparsityPattern(const detail::QP& qp) const override;

  /// @brief The settings given at construction.
  const DAQPSettings& getSettings() const noexcept;
  /// @brief Number of rows of A taken as simple bounds (after initialize()).
  int getNumSimpleBoundRows() const noexcept;
  /// @brief Number of rows of A taken as general constraints (after
  ///   initialize()).
  int getNumGeneralRows() const noexcept;
  /// @brief Whether DAQP regularizes P (proximal-point iterations) because it
  ///   is singular or ill-conditioned, or because DAQPSettings::eps_prox is
  ///   positive (after initialize()).
  bool usesProximalIterations() const noexcept;

private:
  struct Impl; // holds the DAQP types, which stay out of this header
  std::unique_ptr<Impl> impl_;
};

} // namespace affine_mpc

#endif // AFFINE_MPC_DAQP_SOLVER_HPP
