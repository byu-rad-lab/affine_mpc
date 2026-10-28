#include "affine_mpc/daqp_solver.hpp"
#include "affine_mpc/mpc.hpp"
#include "affine_mpc/options.hpp"
#include "affine_mpc/osqp_solver.hpp"
#include "affine_mpc/parameterization.hpp"
#include "affine_mpc/quadratic_program.hpp"

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <gtest/gtest.h>
#include <memory>
#include <string>
#include <vector>

#include "utils.hpp"

using namespace Eigen;
namespace ampc = affine_mpc;

namespace {

const Matrix2d msd_Ac{{0.0, 1.0}, {-0.6, -0.1}};
const Vector2d msd_Bc{0.0, 0.2};

// Mass-spring-damper from rest at 0 to 1 m, with every enabled option set
void configure(ampc::MPC& mpc, const ampc::Options& opts)
{
  mpc.setModelContinuous2Discrete(msd_Ac, msd_Bc, Vector2d::Zero(), 0.1);
  mpc.setInputLimits(VectorXd::Constant(1, 0.0), VectorXd::Constant(1, 3.0));
  mpc.setStateWeights(Vector2d{1.0, 0.1});
  mpc.setReferenceState(Vector2d{1.0, 0.0});
  if (opts.input_cost != ampc::InputCostMode::Off) {
    mpc.setInputWeights(VectorXd::Constant(1, 1e-3));
    mpc.setReferenceInput(VectorXd::Constant(1, 3.0));
  }
  if (opts.slew_initial_input)
    mpc.setSlewRateInitial(VectorXd::Constant(1, 0.5));
  if (opts.slew_control_points)
    mpc.setSlewRate(VectorXd::Constant(1, 0.5));
  if (opts.state_bounds != ampc::StateBoundMode::Off) {
    // the velocity limit is active during the transient
    mpc.setStateLimits(VectorXd::Constant(1, -0.15),
                       VectorXd::Constant(1, 0.15));
  }
  if (opts.state_bounds == ampc::StateBoundMode::Soft)
    mpc.setStateBoundPenalties(VectorXd::Constant(1, 10.0),
                               VectorXd::Constant(1, 100.0));
  if (opts.num_general_constraints > 0)
    mpc.setGeneralConstraints(RowVector2d{1.0, 0.5}, MatrixXd::Zero(1, 1),
                              VectorXd::Constant(1, -10.0),
                              VectorXd::Constant(1, 0.9));
  if (opts.num_terminal_constraints > 0)
    mpc.setTerminalConstraints(RowVector2d{1.0, 1.0},
                               VectorXd::Constant(1, -10.0),
                               VectorXd::Constant(1, 0.95));
}

OSQPSettings referenceSettings()
{
  OSQPSettings settings{ampc::OSQPSolver::getRecommendedSettings(true)};
  settings.eps_abs = settings.eps_rel = 1e-10;
  settings.max_iter = 1000000;
  settings.warm_starting = false;
  return settings;
}

double objective(const ampc::QuadraticProgram& qp, const VectorXd& z)
{
  return 0.5 * z.dot(qp.P * z) + qp.q.dot(z);
}

// Largest violation of l <= A z <= u
double maxViolation(const ampc::QuadraticProgram& qp, const VectorXd& z)
{
  const VectorXd Az{qp.A * z};
  return std::max(0.0,
                  std::max((qp.l - Az).maxCoeff(), (Az - qp.u).maxCoeff()));
}

struct DAQPCase
{
  std::string name;
  ampc::Parameterization param;
  ampc::Options opts;
};

std::vector<DAQPCase> daqpCases()
{
  const auto bspline{ampc::Parameterization::bspline(20, 2, 6)};
  const auto linear{ampc::Parameterization::linearInterp(10, 4)};
  using IB = ampc::InputBoundMode;
  using IC = ampc::InputCostMode;
  using SB = ampc::StateBoundMode;
  return {
      {"defaults", linear, {}},
      {"inputBoundsControlPoints",
       bspline,
       {.input_bounds = IB::ControlPoints}},
      {"inputBoundsRefined",
       bspline,
       {.input_bounds = IB::Refined, .refined_bound_interval = 4}},
      {"inputBoundsExact", bspline, {.input_bounds = IB::Exact}},
      {"inputCostControlPoints", bspline, {.input_cost = IC::ControlPoints}},
      {"inputCostTrajectory", bspline, {.input_cost = IC::Trajectory}},
      {"inputRateCost",
       bspline,
       {.input_cost = IC::ControlPoints, .use_input_rate_cost = true}},
      {"slew",
       linear,
       {.input_cost = IC::ControlPoints,
        .slew_initial_input = true,
        .slew_control_points = true}},
      {"hardStateBounds",
       bspline,
       {.state_bounds = SB::Hard, .bounded_states = {1}}},
      {"softStateBounds",
       bspline,
       {.state_bounds = SB::Soft, .bounded_states = {1}}},
      {"generalAndTerminalConstraints",
       bspline,
       {.num_general_constraints = 1, .num_terminal_constraints = 1}},
      {"moveBlocking",
       ampc::Parameterization::moveBlocking(10, 10),
       {.input_cost = IC::ControlPoints, .slew_initial_input = true}},
  };
}

// Reads the whole QP solution (control points and slacks)
class SolutionMPC : public ampc::MPC
{
public:
  using MPC::MPC;
  VectorXd solution() const { return solution_map_; }
};

} // namespace

class DAQPConsistencyTester : public testing::TestWithParam<DAQPCase>
{};

INSTANTIATE_TEST_SUITE_P(Cases,
                         DAQPConsistencyTester,
                         testing::ValuesIn(daqpCases()),
                         [](const testing::TestParamInfo<DAQPCase>& info) {
                           return info.param.name;
                         });

// DAQP against OSQP (recommended settings) and a tight-tolerance OSQP
// reference, on a closed-loop run in which the model changes halfway (so the
// matrices are updated too). OSQP does not reach even its recommended
// tolerance on some steps with active state bounds; those steps are not
// compared with it.
TEST_P(DAQPConsistencyTester, givenCondensedMPC_DAQPMatchesReference)
{
  const DAQPCase& c{GetParam()};
  SolutionMPC daqp{2, 1, c.param, c.opts};
  ampc::MPC osqp{2, 1, c.param, c.opts}, ref{2, 1, c.param, c.opts};
  for (ampc::MPC* mpc : {static_cast<ampc::MPC*>(&daqp), &osqp, &ref})
    configure(*mpc, c.opts);
  ASSERT_TRUE(daqp.initializeSolver(std::make_unique<ampc::DAQPSolver>()));
  ASSERT_TRUE(osqp.initializeSolver());
  ASSERT_TRUE(ref.initializeSolver(referenceSettings()));

  Vector2d x{Vector2d::Zero()};
  VectorXd u(1);
  int compared{0};
  const int steps{40};
  for (int k{0}; k < steps; ++k) {
    if (k == steps / 2) {
      for (ampc::MPC* mpc : {static_cast<ampc::MPC*>(&daqp), &osqp, &ref})
        mpc->setModelContinuous2Discrete(1.2 * msd_Ac, msd_Bc,
                                         Vector2d{0.0, 0.01}, 0.1);
    }
    ASSERT_EQ(daqp.solve(x), ampc::SolveStatus::Success) << "step " << k;
    const bool osqp_solved{osqp.solve(x) == ampc::SolveStatus::Success};
    const bool ref_solved{ref.solve(x) == ampc::SolveStatus::Success};

    // DAQP's solution is feasible (to its 1e-6 tolerance) and its objective
    // is the QP's at that solution
    const ampc::QuadraticProgram qp{daqp.getQP()};
    const VectorXd z{daqp.solution()};
    const ampc::SolveInfo info{daqp.getSolveInfo()};
    EXPECT_LE(maxViolation(qp, z), 1e-6) << "step " << k;
    EXPECT_NEAR(objective(qp, z), info.objective,
                1e-9 * (1.0 + std::abs(info.objective)))
        << "step " << k;

    VectorXd cp_daqp(daqp.getNumControlPoints()), cp_other(cp_daqp.size());
    daqp.getInputControlPoints(cp_daqp);
    if (ref_solved) {
      // The objective is the sharp check: P has flat directions in some
      // cases, along which the solutions may differ more
      ++compared;
      const double ref_obj{ref.getSolveInfo().objective};
      EXPECT_NEAR(info.objective, ref_obj, 1e-6 * (1.0 + std::abs(ref_obj)))
          << "step " << k;
      ref.getInputControlPoints(cp_other);
      EXPECT_TRUE(expectEigenNear(cp_daqp, cp_other, 1e-3)) << "step " << k;
    }
    if (osqp_solved) {
      osqp.getInputControlPoints(cp_other);
      EXPECT_TRUE(expectEigenNear(cp_daqp, cp_other, 2e-2)) << "step " << k;
    }

    daqp.getNextInput(u);
    daqp.propagateModel(x, u, x);
  }
  EXPECT_GE(compared, steps * 3 / 4);
}

// Warm start keeps the working set; a cold start gives the same optimum with
// at least as many iterations in total. The dual active-set iterations stop
// once the constraints are violated by at most the primal tolerance (1e-6),
// so the objectives agree to about that tolerance times the multipliers.
TEST_P(DAQPConsistencyTester, givenWarmStart_MatchesColdStartWithFewerIters)
{
  const DAQPCase& c{GetParam()};
  ampc::MPC warm{2, 1, c.param, c.opts}, cold{2, 1, c.param, c.opts};
  configure(warm, c.opts);
  configure(cold, c.opts);
  ASSERT_TRUE(warm.initializeSolver(std::make_unique<ampc::DAQPSolver>()));
  ASSERT_TRUE(cold.initializeSolver(std::make_unique<ampc::DAQPSolver>()));

  Vector2d x{Vector2d::Zero()};
  VectorXd u_warm(1), u_cold(1);
  int iters_warm{0}, iters_cold{0};
  for (int k{0}; k < 40; ++k) {
    cold.resetWarmStart();
    ASSERT_EQ(warm.solve(x), ampc::SolveStatus::Success);
    ASSERT_EQ(cold.solve(x), ampc::SolveStatus::Success);
    const ampc::SolveInfo info_warm{warm.getSolveInfo()};
    const ampc::SolveInfo info_cold{cold.getSolveInfo()};
    iters_warm += info_warm.iterations;
    iters_cold += info_cold.iterations;
    EXPECT_NEAR(info_warm.objective, info_cold.objective,
                1e-5 * (1.0 + std::abs(info_cold.objective)))
        << "step " << k;
    warm.getNextInput(u_warm);
    cold.getNextInput(u_cold);
    EXPECT_NEAR(u_warm(0), u_cold(0), 1e-3) << "step " << k;
    warm.propagateModel(x, u_warm, x);
  }
  EXPECT_LE(iters_warm, iters_cold);
}

TEST(DAQPSolverTester, givenSparseFormulation_ThrowsInvalidArgument)
{
  ampc::MPC mpc{2, 1, ampc::Parameterization::linearInterp(10, 4),
                ampc::Formulation::Sparse};
  configure(mpc, {});
  expectInvalidArgumentWithMessage(
      [&] { (void)mpc.initializeSolver(std::make_unique<ampc::DAQPSolver>()); },
      "DAQP does not support the sparse formulation");
  EXPECT_FALSE(mpc.isSolverInitialized());
}

TEST(DAQPSolverTester, givenMPC_SplitsRowsIntoSimpleBoundsAndGeneralRows)
{
  // input bounds on 4 control points, the initial slew, and the velocity
  // bound of the first step (which depends on the first control point only)
  // are simple bounds; the 3 control point slew rows and the other 9
  // velocity bounds are general rows
  const ampc::Options opts{.slew_initial_input = true,
                           .slew_control_points = true,
                           .state_bounds = ampc::StateBoundMode::Hard,
                           .bounded_states = {1}};
  ampc::MPC mpc{2, 1, ampc::Parameterization::linearInterp(10, 4), opts};
  configure(mpc, opts);
  auto solver{std::make_unique<ampc::DAQPSolver>()};
  const ampc::DAQPSolver& daqp{*solver};
  ASSERT_TRUE(mpc.initializeSolver(std::move(solver)));
  EXPECT_EQ(daqp.getNumSimpleBoundRows(), 4 + 1 + 1);
  EXPECT_EQ(daqp.getNumGeneralRows(), 3 + 9);
  EXPECT_EQ(daqp.getNumSimpleBoundRows() + daqp.getNumGeneralRows(),
            mpc.getNumQPConstraints());
  EXPECT_FALSE(daqp.usesProximalIterations());
  EXPECT_TRUE(mpc.isWithinSparsityPattern());
  EXPECT_STREQ(daqp.getName(), "DAQP");
}

TEST(DAQPSolverTester, givenSolve_ReportsSolveInfo)
{
  const ampc::Options opts{.input_cost = ampc::InputCostMode::ControlPoints};
  ampc::MPC mpc{2, 1, ampc::Parameterization::linearInterp(10, 4), opts};
  configure(mpc, opts);
  ASSERT_TRUE(mpc.initializeSolver(std::make_unique<ampc::DAQPSolver>()));
  EXPECT_EQ(mpc.getSolveInfo().status, ampc::SolveStatus::OtherFailure);
  ASSERT_EQ(mpc.solve(Vector2d::Zero()), ampc::SolveStatus::Success);
  const ampc::SolveInfo info{mpc.getSolveInfo()};
  EXPECT_EQ(info.status, ampc::SolveStatus::Success);
  EXPECT_GE(info.iterations, 1);
  EXPECT_GT(info.qp_solve_time, 0.0);
  EXPECT_LE(info.qp_solve_time, info.mpc_solve_time);
  const ampc::QuadraticProgram qp{mpc.getQP()};
  VectorXd cp(4);
  mpc.getInputControlPoints(cp);
  EXPECT_NEAR(info.objective, objective(qp, cp), 1e-9);
}

// Velocity bound that the input can not keep from x0
TEST(DAQPSolverTester,
     givenInfeasibleProblem_ReportsPrimalInfeasibleAndRecovers)
{
  const ampc::Options opts{.state_bounds = ampc::StateBoundMode::Hard,
                           .bounded_states = {1}};
  ampc::MPC mpc{2, 1, ampc::Parameterization::linearInterp(10, 4), opts};
  configure(mpc, opts);
  ASSERT_TRUE(mpc.initializeSolver(std::make_unique<ampc::DAQPSolver>()));
  EXPECT_EQ(mpc.solve(Vector2d{0.0, 2.0}), ampc::SolveStatus::PrimalInfeasible);
  EXPECT_EQ(mpc.getSolveInfo().status, ampc::SolveStatus::PrimalInfeasible);
  EXPECT_EQ(mpc.solve(Vector2d::Zero()), ampc::SolveStatus::Success);
}

TEST(DAQPSolverTester, givenIterationLimit_ReportsMaxIterReached)
{
  const ampc::Options opts{.state_bounds = ampc::StateBoundMode::Hard,
                           .bounded_states = {1}};
  ampc::MPC mpc{2, 1, ampc::Parameterization::bspline(20, 2, 6), opts};
  configure(mpc, opts);
  ASSERT_TRUE(mpc.initializeSolver(
      std::make_unique<ampc::DAQPSolver>(ampc::DAQPSettings{.iter_limit = 1})));
  EXPECT_EQ(mpc.solve(Vector2d::Zero()), ampc::SolveStatus::MaxIterReached);
}

// P singular: no input cost, and only the terminal position weighted, so P
// has rank one. DAQP's automatic proximal iterations find a minimizer (one of
// many, so compare the objective, not the solution).
TEST(DAQPSolverTester, givenSingularCostMatrix_SolvesWithProximalIterations)
{
  const ampc::Options opts{.slew_control_points = true};
  const auto param{ampc::Parameterization::linearInterp(10, 4)};
  ampc::MPC daqp{2, 1, param, opts}, ref{2, 1, param, opts};
  for (ampc::MPC* mpc : {&daqp, &ref}) {
    configure(*mpc, opts);
    mpc->setStateWeights(Vector2d::Zero(), Vector2d{1.0, 0.0});
  }
  auto solver{std::make_unique<ampc::DAQPSolver>()};
  const ampc::DAQPSolver& daqp_solver{*solver};
  ASSERT_TRUE(daqp.initializeSolver(std::move(solver)));
  ASSERT_TRUE(ref.initializeSolver(referenceSettings()));
  EXPECT_TRUE(daqp_solver.usesProximalIterations());

  Vector2d x{Vector2d::Zero()};
  VectorXd u(1);
  for (int k{0}; k < 30; ++k) {
    ASSERT_EQ(daqp.solve(x), ampc::SolveStatus::Success) << "step " << k;
    ASSERT_EQ(ref.solve(x), ampc::SolveStatus::Success) << "step " << k;
    const ampc::QuadraticProgram qp{daqp.getQP()};
    VectorXd cp(4);
    daqp.getInputControlPoints(cp);
    EXPECT_LE(maxViolation(qp, cp), 1e-6) << "step " << k;
    EXPECT_NEAR(objective(qp, cp), ref.getSolveInfo().objective, 1e-6)
        << "step " << k;
    EXPECT_NEAR(daqp.getSolveInfo().objective, objective(qp, cp), 1e-6)
        << "step " << k;
    daqp.getNextInput(u);
    daqp.propagateModel(x, u, x);
  }
}

// Without proximal iterations a singular P is rejected at initialization
TEST(DAQPSolverTester, givenSingularCostMatrixWithoutProx_FailsToInitialize)
{
  const ampc::Options opts{};
  ampc::MPC mpc{2, 1, ampc::Parameterization::linearInterp(10, 4), opts};
  configure(mpc, opts);
  mpc.setStateWeights(Vector2d::Zero(), Vector2d{1.0, 0.0});
  EXPECT_FALSE(mpc.initializeSolver(
      std::make_unique<ampc::DAQPSolver>(ampc::DAQPSettings{.eps_prox = 0.0})));
  EXPECT_FALSE(mpc.isSolverInitialized());
  EXPECT_EQ(mpc.solve(Vector2d::Zero()), ampc::SolveStatus::NotInitialized);
}

// ... and a weight change that makes P singular is a failed update, which
// recovers once P is positive definite again
TEST(DAQPSolverTester, givenWeightsMakingCostSingularWithoutProx_UpdateFails)
{
  const ampc::Options opts{};
  ampc::MPC mpc{2, 1, ampc::Parameterization::linearInterp(10, 4), opts};
  configure(mpc, opts);
  ASSERT_TRUE(mpc.initializeSolver(
      std::make_unique<ampc::DAQPSolver>(ampc::DAQPSettings{.eps_prox = 0.0})));
  ASSERT_EQ(mpc.solve(Vector2d::Zero()), ampc::SolveStatus::Success);
  mpc.setStateWeights(Vector2d::Zero(), Vector2d{1.0, 0.0});
  EXPECT_EQ(mpc.solve(Vector2d::Zero()), ampc::SolveStatus::UpdateFailed);
  mpc.setStateWeights(Vector2d{1.0, 0.1});
  EXPECT_EQ(mpc.solve(Vector2d::Zero()), ampc::SolveStatus::Success);
}

// Soft state bounds with only an L1 penalty: the slacks have no quadratic
// cost, so P is singular in their directions (OSQP converges slowly here, so
// the reference does not reach its tolerance on every step)
TEST(DAQPSolverTester, givenSoftBoundsWithOnlyL1Penalty_MatchesReference)
{
  const ampc::Options opts{.state_bounds = ampc::StateBoundMode::Soft,
                           .bounded_states = {1}};
  const auto param{ampc::Parameterization::bspline(20, 2, 6)};
  SolutionMPC daqp{2, 1, param, opts};
  ampc::MPC ref{2, 1, param, opts};
  for (ampc::MPC* mpc : {static_cast<ampc::MPC*>(&daqp), &ref}) {
    configure(*mpc, opts);
    mpc->setStateBoundPenalties(VectorXd::Constant(1, 10.0), VectorXd::Zero(1));
  }
  auto solver{std::make_unique<ampc::DAQPSolver>()};
  const ampc::DAQPSolver& daqp_solver{*solver};
  ASSERT_TRUE(daqp.initializeSolver(std::move(solver)));
  ASSERT_TRUE(ref.initializeSolver(referenceSettings()));
  EXPECT_TRUE(daqp_solver.usesProximalIterations());

  Vector2d x{Vector2d::Zero()};
  VectorXd u(1);
  int compared{0};
  const int steps{40};
  for (int k{0}; k < steps; ++k) {
    ASSERT_EQ(daqp.solve(x), ampc::SolveStatus::Success) << "step " << k;
    const ampc::QuadraticProgram qp{daqp.getQP()};
    const VectorXd z{daqp.solution()};
    const double obj{daqp.getSolveInfo().objective};
    EXPECT_LE(maxViolation(qp, z), 1e-6) << "step " << k;
    EXPECT_NEAR(objective(qp, z), obj, 1e-9 * (1.0 + std::abs(obj)))
        << "step " << k;
    if (ref.solve(x) == ampc::SolveStatus::Success) {
      ++compared;
      const double ref_obj{ref.getSolveInfo().objective};
      EXPECT_NEAR(obj, ref_obj, 1e-5 * (1.0 + std::abs(ref_obj)))
          << "step " << k;
      VectorXd cp_daqp(6), cp_ref(6);
      daqp.getInputControlPoints(cp_daqp);
      ref.getInputControlPoints(cp_ref);
      EXPECT_TRUE(expectEigenNear(cp_daqp, cp_ref, 1e-3)) << "step " << k;
    }
    daqp.getNextInput(u);
    daqp.propagateModel(x, u, x);
  }
  EXPECT_GE(compared, steps * 3 / 4);
}
