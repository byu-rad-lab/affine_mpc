#include "affine_mpc/condensed_mpc.hpp"
#include "affine_mpc/options.hpp"
#include "affine_mpc/parameterization.hpp"
#include "affine_mpc/sparse_mpc.hpp"

#include <Eigen/Core>
#include <cmath>
#include <gtest/gtest.h>

#include "utils.hpp"

using namespace Eigen;
namespace ampc = affine_mpc;

class ConsistencyTester
{
public:
  ConsistencyTester(const int n,
                    const int m,
                    const ampc::Parameterization& param,
                    const ampc::Options& opts = {}) :
      condensed{n, m, param, opts},
      sparse{n, m, param, opts},
      opts{opts},
      n{n},
      m{m} {
        // nothing to do, member initializer list does all the work
      };

  void setup()
  {
    Eigen::Matrix2d A;
    Eigen::Vector2d B, w;
    A << 0, 1, -0.6, -0.1;
    B << 0, 0.2;
    w.setZero();
    const double ts{0.1};

    auto setup = [&](auto& mpc) {
      mpc.setModelContinuous2Discrete(A, B, w, 0.1);
      mpc.setInputLimits(VectorXd::Constant(m, 0.0),
                         VectorXd::Constant(m, 3.0));
      mpc.setStateWeights(Vector2d{1.0, 0.1});
      mpc.setReferenceState(Vector2d{1.0, 0.0});
      if (opts.use_input_cost) {
        mpc.setInputWeights(VectorXd::Constant(m, 1e-4));
        mpc.setReferenceInput(VectorXd::Zero(m));
      }
      if (opts.slew_initial_input) {
        VectorXd slew = VectorXd::Constant(m, 0.5);
        mpc.setSlewRateInitial(slew);
      }
      if (opts.slew_control_points) {
        VectorXd slew = VectorXd::Constant(m, 0.5);
        mpc.setSlewRate(slew);
      }
      auto settings{ampc::OSQPSolver::getRecommendedSettings(true)};
      settings.warm_starting = true;
      ASSERT_TRUE(mpc.initializeSolver(settings));
    };
    setup(condensed);
    setup(sparse);
  }

  void setModel()
  {
    Eigen::Matrix2d A;
    Eigen::Vector2d B, w;
    A << 0, 1, -0.6, -0.1;
    B << 0, 0.2;
    w.setZero();
    const double ts{0.1};
    condensed.setModelContinuous2Discrete(A, B, w, ts);
    sparse.setModelContinuous2Discrete(A, B, w, ts);
  }

  void setInputLimits()
  {
    condensed.setInputLimits(VectorXd::Constant(m, 0.0),
                             VectorXd::Constant(m, 3.0));
    sparse.setInputLimits(VectorXd::Constant(m, 0.0),
                          VectorXd::Constant(m, 3.0));
  }

  // member variables for testing
  ampc::CondensedMPC condensed;
  ampc::SparseMPC sparse;
  ampc::Options opts;
  const int n, m;
};

// ---- CondensedMPC vs SparseMPC consistency test ----------------------------

TEST(ConsistencyTester, givenSameSystem_CondensedAndSparseMPCAgree)
{
  const int n{2}, m{1}, T{10}, nc{10};
  const ampc::Parameterization param{
      ampc::Parameterization::linearInterp(T, nc)};
  const ampc::Options opts{.use_input_cost = true};

  ConsistencyTester tester{n, m, param, opts};
  tester.setup();

  const Vector2d x0{0.5, -0.2};
  ASSERT_EQ(tester.condensed.solve(x0), ampc::SolveStatus::Success);
  ASSERT_EQ(tester.sparse.solve(x0), ampc::SolveStatus::Success);

  VectorXd u_condensed{m}, u_sparse{m};
  tester.condensed.getNextInput(u_condensed);
  tester.sparse.getNextInput(u_sparse);

  ASSERT_TRUE(expectEigenNear(u_condensed, u_sparse, 1e-4));

  // Also verify the full predicted trajectory agrees
  VectorXd x_traj_condensed{n * T}, x_traj_sparse{n * T};
  tester.condensed.getPredictedStateTrajectory(x_traj_condensed);
  tester.sparse.getPredictedStateTrajectory(x_traj_sparse);

  ASSERT_TRUE(expectEigenNear(x_traj_condensed, x_traj_sparse, 1e-4));
}

TEST(ConsistencyTester,
     givenInitialSlewRate_CondensedAndSparseMPCSolveCorrectly)
{
  const int n{2}, m{1}, T{10}, nc{10};
  const ampc::Parameterization param{
      ampc::Parameterization::linearInterp(T, nc)};
  const ampc::Options opts{.slew_initial_input = true};


  ConsistencyTester tester{n, m, param, opts};
  tester.setup();

  VectorXd slew = VectorXd::Constant(m, 0.25);
  tester.condensed.setSlewRateInitial(slew);
  tester.sparse.setSlewRateInitial(slew);

  const Vector2d x0{0, 0};
  ASSERT_EQ(tester.condensed.solve(x0), ampc::SolveStatus::Success);
  ASSERT_EQ(tester.sparse.solve(x0), ampc::SolveStatus::Success);

  VectorXd u_condensed{m}, u_sparse{m};
  tester.condensed.getNextInput(u_condensed);
  tester.sparse.getNextInput(u_sparse);

  // solution without slew rate would be u_max=3, so with slew rate the
  // solution should be the slew rate itself
  ASSERT_TRUE(expectEigenNear(u_condensed, slew, 1e-4));
  ASSERT_TRUE(expectEigenNear(u_sparse, slew, 1e-4));

  // testing that u_prev is automatically updated after solve
  ASSERT_EQ(tester.condensed.solve(x0), ampc::SolveStatus::Success);
  ASSERT_EQ(tester.sparse.solve(x0), ampc::SolveStatus::Success);

  tester.condensed.getNextInput(u_condensed);
  tester.sparse.getNextInput(u_sparse);

  ASSERT_TRUE(expectEigenNear(u_condensed, 2 * slew, 1e-4));
  ASSERT_TRUE(expectEigenNear(u_sparse, 2 * slew, 1e-4));

  // testing manually-set u_prev
  VectorXd u_prev = VectorXd::Constant(m, 0.1);
  tester.condensed.setPreviousInput(u_prev);
  tester.sparse.setPreviousInput(u_prev);

  ASSERT_EQ(tester.condensed.solve(x0), ampc::SolveStatus::Success);
  ASSERT_EQ(tester.sparse.solve(x0), ampc::SolveStatus::Success);

  tester.condensed.getNextInput(u_condensed);
  tester.sparse.getNextInput(u_sparse);

  ASSERT_TRUE(expectEigenNear(u_condensed, u_prev + slew, 1e-4));
  ASSERT_TRUE(expectEigenNear(u_sparse, u_prev + slew, 1e-4));
}

TEST(ConsistencyTester, initializeSolverWithoutModel_Throws)
{
  const int n{2}, m{1}, T{5}, nc{3};
  const auto param{ampc::Parameterization::linearInterp(T, nc)};
  ConsistencyTester tester{n, m, param};

  // model not set
  expectLogicErrorWithMessage(
      [&tester]() { bool blah = tester.condensed.initializeSolver(); },
      "Model must be set before initializing solver");
  expectLogicErrorWithMessage(
      [&tester]() { bool blah = tester.sparse.initializeSolver(); },
      "Model must be set before initializing solver");
}

TEST(ConsistencyTester, initializeSolverWithoutInputLimits_Throws)
{
  const int n{2}, m{1}, T{5}, nc{3};
  const auto param{ampc::Parameterization::linearInterp(T, nc)};
  ConsistencyTester tester{n, m, param};
  tester.setModel();

  expectLogicErrorWithMessage(
      [&tester]() { bool blah = tester.condensed.initializeSolver(); },
      "Input limits must be set before initializing solver");
  expectLogicErrorWithMessage(
      [&tester]() { bool blah = tester.sparse.initializeSolver(); },
      "Input limits must be set before initializing solver");
}

TEST(ConsistencyTester, initializeSolverWithoutSettingInitialSlewRate_Throws)
{
  const int n{2}, m{1}, T{5}, nc{3};
  const auto param{ampc::Parameterization::linearInterp(T, nc)};
  ConsistencyTester tester{n, m, param, {.slew_initial_input = true}};
  tester.setModel();
  tester.setInputLimits();

  expectLogicErrorWithMessage(
      [&tester]() { bool blah = tester.condensed.initializeSolver(); },
      "Initial slew rate must be set before initializing solver");
  expectLogicErrorWithMessage(
      [&tester]() { bool blah = tester.sparse.initializeSolver(); },
      "Initial slew rate must be set before initializing solver");
}

TEST(ConsistencyTester, initializeSolverWithoutSettingSlewRate_Throws)
{
  const int n{2}, m{1}, T{5}, nc{3};
  const auto param{ampc::Parameterization::linearInterp(T, nc)};
  ConsistencyTester tester{n, m, param, {.slew_control_points = true}};
  tester.setModel();
  tester.setInputLimits();

  expectLogicErrorWithMessage(
      [&tester]() { bool blah = tester.condensed.initializeSolver(); },
      "Slew rate must be set before initializing solver");
  expectLogicErrorWithMessage(
      [&tester]() { bool blah = tester.sparse.initializeSolver(); },
      "Slew rate must be set before initializing solver");
}

TEST(ConsistencyTester, initializeSolverWithoutSettingStateLimits_Throws)
{
  const int n{2}, m{1}, T{5}, nc{3};
  const auto param{ampc::Parameterization::linearInterp(T, nc)};
  ConsistencyTester tester{n, m, param, {.saturate_states = true}};
  tester.setModel();
  tester.setInputLimits();

  expectLogicErrorWithMessage(
      [&tester]() { bool blah = tester.condensed.initializeSolver(); },
      "State limits must be set before initializing solver");
  expectLogicErrorWithMessage(
      [&tester]() { bool blah = tester.sparse.initializeSolver(); },
      "State limits must be set before initializing solver");
}

TEST(ConsistencyTester, doubleInitializeSolver_IsNoOp)
{
  const int n{2}, m{1}, T{5}, nc{3};
  const auto param{ampc::Parameterization::linearInterp(T, nc)};
  ConsistencyTester tester{n, m, param};
  tester.setup();

  // second call must succeed and not corrupt
  ASSERT_TRUE(tester.condensed.initializeSolver());
  ASSERT_TRUE(tester.sparse.initializeSolver());

  ASSERT_EQ(tester.condensed.solve(Vector2d::Zero()),
            ampc::SolveStatus::Success);
  ASSERT_EQ(tester.sparse.solve(Vector2d::Zero()), ampc::SolveStatus::Success);

  // unconstrained optimal drives toward goal: u should be positive
  VectorXd u{m};
  tester.condensed.getNextInput(u);
  ASSERT_GT(u(0), 0.0);
  tester.sparse.getNextInput(u);
  ASSERT_GT(u(0), 0.0);
}

TEST(ConsistencyTester, tryToSolveBeforeInitialingSolver_ReturnsNotInitialized)
{
  const int n{2}, m{1}, T{5}, nc{3};
  const auto param{ampc::Parameterization::linearInterp(T, nc)};
  ConsistencyTester tester{n, m, param};
  ASSERT_EQ(tester.condensed.solve(Vector2d::Zero()),
            ampc::SolveStatus::NotInitialized);
  ASSERT_EQ(tester.sparse.solve(Vector2d::Zero()),
            ampc::SolveStatus::NotInitialized);
}

TEST(ConsistencyTester, givenSlewControlPoints_CondensedAndSparseMPCAgree)
{
  const int n{2}, m{1}, T{10}, nc{10};
  const auto param{ampc::Parameterization::linearInterp(T, nc)};
  const ampc::Options opts{.use_input_cost = true, .slew_control_points = true};
  ConsistencyTester tester{n, m, param, opts};
  tester.setup();

  VectorXd slew = VectorXd::Constant(m, 1.0);
  tester.condensed.setSlewRate(slew);
  tester.sparse.setSlewRate(slew);

  const Vector2d x0{0.5, -0.2};
  ASSERT_EQ(tester.condensed.solve(x0), ampc::SolveStatus::Success);
  ASSERT_EQ(tester.sparse.solve(x0), ampc::SolveStatus::Success);

  VectorXd u_condensed{m}, u_sparse{m};
  tester.condensed.getNextInput(u_condensed);
  tester.sparse.getNextInput(u_sparse);
  ASSERT_TRUE(expectEigenNear(u_condensed, u_sparse, 1e-4));

  // Verify slew constraint is respected in both
  VectorXd u_traj_condensed{m * nc}, u_traj_sparse{m * nc};
  tester.condensed.getInputControlPoints(u_traj_condensed);
  tester.sparse.getInputControlPoints(u_traj_sparse);

  for (int i{0}; i < nc - 1; ++i) {
    EXPECT_LE(std::abs(u_traj_condensed(i + 1) - u_traj_condensed(i)),
              slew(0) + 1e-4);
    EXPECT_LE(std::abs(u_traj_sparse(i + 1) - u_traj_sparse(i)),
              slew(0) + 1e-4);
  }

  // Full predicted trajectories must also agree
  VectorXd x_traj_condensed{n * T}, x_traj_sparse{n * T};
  tester.condensed.getPredictedStateTrajectory(x_traj_condensed);
  tester.sparse.getPredictedStateTrajectory(x_traj_sparse);
  ASSERT_TRUE(expectEigenNear(x_traj_condensed, x_traj_sparse, 1e-4));
}

TEST(ConsistencyTester, givenBothSlewOptions_CondensedAndSparseMPCAgree)
{
  const int n{2}, m{1}, T{10}, nc{10};
  const ampc::Parameterization param{
      ampc::Parameterization::linearInterp(T, nc)};
  const ampc::Options opts{.use_input_cost = true,
                           .slew_initial_input = true,
                           .slew_control_points = true};

  ConsistencyTester tester{n, m, param, opts};
  tester.setup();

  VectorXd slew = VectorXd::Constant(m, 1.0);
  VectorXd slew0 = VectorXd::Constant(m, 0.5);
  tester.condensed.setSlewRate(slew);
  tester.sparse.setSlewRate(slew);
  tester.condensed.setSlewRateInitial(slew0);
  tester.sparse.setSlewRateInitial(slew0);

  const Vector2d x0{0.5, -0.2};
  ASSERT_EQ(tester.condensed.solve(x0), ampc::SolveStatus::Success);
  ASSERT_EQ(tester.sparse.solve(x0), ampc::SolveStatus::Success);

  VectorXd u_condensed{m}, u_sparse{m};
  tester.condensed.getNextInput(u_condensed);
  tester.sparse.getNextInput(u_sparse);
  ASSERT_TRUE(expectEigenNear(u_condensed, u_sparse, 1e-4));
}

TEST(ConsistencyTester,
     givenKnownSolution_PredictedStateTrajMatchesManualPropagation)
{
  const int n{2}, m{1}, T{5}, nc{5};
  const auto param{ampc::Parameterization::linearInterp(T, nc)};
  ConsistencyTester tester{n, m, param, {.use_input_cost = true}};
  tester.setup();

  const Vector2d x0{0.3, -0.1};
  ASSERT_EQ(tester.condensed.solve(x0), ampc::SolveStatus::Success);
  ASSERT_EQ(tester.sparse.solve(x0), ampc::SolveStatus::Success);

  // Get the input trajectory the solver chose
  VectorXd u_traj_c{m * T}, u_traj_s{m * T};
  tester.condensed.getInputTrajectory(u_traj_c);
  tester.sparse.getInputTrajectory(u_traj_s);


  VectorXd x_traj_c{n * T}, x_traj_s{n * T};
  tester.condensed.getPredictedStateTrajectory(x_traj_c);
  tester.sparse.getPredictedStateTrajectory(x_traj_s);

  // Manually propagate the model
  VectorXd x_c = x0;
  VectorXd x_s = x0;
  for (int k{0}; k < T; ++k) {
    const auto u_c = u_traj_c.segment(k * m, m);
    const auto u_s = u_traj_s.segment(k * m, m);
    tester.condensed.propagateModel(x_c, u_c, x_c);
    tester.sparse.propagateModel(x_s, u_s, x_s);

    const VectorXd x_c_expected = x_traj_c.segment(k * n, n);
    const VectorXd x_s_expected = x_traj_s.segment(k * n, n);
    ASSERT_TRUE(expectEigenNear(x_c, x_c_expected, 1e-6));
    ASSERT_TRUE(expectEigenNear(x_s, x_s_expected, 1e-6));
  }
}

TEST(ConsistencyTester, givenMultiInputSystem_CondensedAndSparseMPCAgree)
{
  // 3 states, 2 inputs, horizon=8, nc=4
  // Verifies m-dependent indexing in spline weights and constraint formation
  const int n{3}, m{2}, T{8}, nc{4};
  const auto param{ampc::Parameterization::linearInterp(T, nc)};
  const ampc::Options opts{.use_input_cost = true};
  ConsistencyTester tester{n, m, param, opts};

  // Simple 3-state, 2-input discrete system
  Eigen::Matrix3d Ad;
  Eigen::Matrix<double, 3, 2> Bd;
  Eigen::Vector3d wd;
  Ad << 0.9, 0.1, 0.0, 0.0, 0.8, 0.1, 0.0, 0.0, 0.9;
  Bd << 0.1, 0.0, 0.0, 0.1, 0.05, 0.05;
  wd.setZero();

  auto setup = [&](auto& mpc) {
    mpc.setModelDiscrete(Ad, Bd, wd);
    mpc.setInputLimits(VectorXd::Constant(m, -2.0), VectorXd::Constant(m, 2.0));
    mpc.setStateWeights(Eigen::Vector3d{1.0, 1.0, 1.0});
    mpc.setReferenceState(Eigen::Vector3d{1.0, 0.0, 0.0});
    mpc.setInputWeights(VectorXd::Constant(m, 1e-3));
    mpc.setReferenceInput(VectorXd::Zero(m));
    ASSERT_TRUE(mpc.initializeSolver());
  };
  setup(tester.condensed);
  setup(tester.sparse);

  const Eigen::Vector3d x0{0.0, 0.0, 0.0};
  ASSERT_EQ(tester.condensed.solve(x0), ampc::SolveStatus::Success);
  ASSERT_EQ(tester.sparse.solve(x0), ampc::SolveStatus::Success);

  VectorXd u_condensed{m}, u_sparse{m};
  tester.condensed.getNextInput(u_condensed);
  tester.sparse.getNextInput(u_sparse);
  ASSERT_TRUE(expectEigenNear(u_condensed, u_sparse, 1e-4));

  VectorXd x_traj_condensed{n * T}, x_traj_sparse{n * T};
  tester.condensed.getPredictedStateTrajectory(x_traj_condensed);
  tester.sparse.getPredictedStateTrajectory(x_traj_sparse);
  ASSERT_TRUE(expectEigenNear(x_traj_condensed, x_traj_sparse, 1e-4));
}

TEST(ConsistencyTester,
     givenInputTrajectorySaturation_CondensedAndSparseMPCAgree)
{
  const int n{2}, m{1}, T{10}, nc{5}, deg{2};
  const auto param{ampc::Parameterization::bspline(T, deg, nc)};
  const ampc::Options opts{.saturate_input_trajectory = true};
  ConsistencyTester tester{n, m, param, opts};
  tester.setup(); // input limits are [0, 3]

  const Vector2d x0{0, 0};
  ASSERT_EQ(tester.condensed.solve(x0), ampc::SolveStatus::Success);
  ASSERT_EQ(tester.sparse.solve(x0), ampc::SolveStatus::Success);

  VectorXd u_traj_condensed{m * T}, u_traj_sparse{m * T};
  tester.condensed.getInputTrajectory(u_traj_condensed);
  tester.sparse.getInputTrajectory(u_traj_sparse);
  ASSERT_TRUE(expectEigenNear(u_traj_condensed, u_traj_sparse, 1e-4));

  // reaching x_ref quickly requires u > u_max, so the upper limit is active
  EXPECT_NEAR(u_traj_sparse.maxCoeff(), 3.0, 1e-4);
  EXPECT_GE(u_traj_sparse.minCoeff(), -1e-4);

  VectorXd x_traj_condensed{n * T}, x_traj_sparse{n * T};
  tester.condensed.getPredictedStateTrajectory(x_traj_condensed);
  tester.sparse.getPredictedStateTrajectory(x_traj_sparse);
  ASSERT_TRUE(expectEigenNear(x_traj_condensed, x_traj_sparse, 1e-4));
}

// ---- Sparsity pattern fixed at initialization ------------------------------

namespace {

// mass-spring-damper discretized at 0.1 s (every entry of Ad and Bd nonzero)
void setMsdModel(ampc::MPCBase& mpc)
{
  Matrix2d A;
  A << 0, 1, -0.6, -0.1;
  const Vector2d B{0, 0.2}, w{0, 0};
  mpc.setModelContinuous2Discrete(A, B, w, 0.1);
}

void configureMsd(ampc::MPCBase& mpc, const ampc::Options& opts)
{
  setMsdModel(mpc);
  mpc.setInputLimits(VectorXd::Constant(1, 0.0), VectorXd::Constant(1, 3.0));
  mpc.setReferenceState(Vector2d{1.0, 0.0});
  if (opts.use_input_cost)
    mpc.setReferenceInput(VectorXd::Zero(1));
  if (opts.saturate_states)
    mpc.setStateLimits(Vector2d::Constant(-10), Vector2d::Constant(10));
}

} // namespace

TEST(ConsistencyTester, givenZeroWeightsAtInit_SparseMPCAppliesLaterWeights)
{
  const int n{2}, m{1}, T{10}, nc{5};
  const auto param{ampc::Parameterization::linearInterp(T, nc)};
  const ampc::Options opts{.use_input_cost = true};
  const auto settings{ampc::OSQPSolver::getRecommendedSettings(true)};

  ampc::SparseMPC late{n, m, param, opts};
  configureMsd(late, opts);
  late.setWeights(Vector2d{1, 0}, VectorXd::Zero(m));
  ASSERT_TRUE(late.initializeSolver(settings));

  const Vector2d Q{1, 0.5};
  const VectorXd R{VectorXd::Constant(m, 1e-2)};
  late.setWeights(Q, R);

  ampc::SparseMPC reference{n, m, param, opts};
  configureMsd(reference, opts);
  reference.setWeights(Q, R);
  ASSERT_TRUE(reference.initializeSolver(settings));

  const Vector2d x0{0.5, -0.2};
  ASSERT_EQ(late.solve(x0), ampc::SolveStatus::Success);
  ASSERT_EQ(reference.solve(x0), ampc::SolveStatus::Success);
  EXPECT_TRUE(late.isWithinSparsityPattern());

  VectorXd u_late{m * T}, u_reference{m * T};
  late.getInputTrajectory(u_late);
  reference.getInputTrajectory(u_reference);
  ASSERT_TRUE(expectEigenNear(u_late, u_reference, 1e-4));
}

TEST(ConsistencyTester, givenModelOutsideInitPattern_IsWithinPatternIsFalse)
{
  const int n{2}, m{1}, T{10}, nc{5};
  const auto param{ampc::Parameterization::linearInterp(T, nc)};
  // state saturation puts the condensed prediction matrix S into A
  const ampc::Options opts{.saturate_states = true};

  // Ad = 0.9 I and Bd = [0, 0.1] leave the first state uncoupled, so the
  // initialized pattern has no entries for that coupling
  const Matrix2d Ad_diag{Matrix2d::Identity() * 0.9};
  const Vector2d Bd{0, 0.1}, wd{0, 0};
  const Vector2d x0{0.5, -0.2};

  auto check = [&](ampc::MPCBase& mpc) {
    configureMsd(mpc, opts);
    mpc.setModelDiscrete(Ad_diag, Bd, wd);
    EXPECT_TRUE(mpc.isWithinSparsityPattern()); // not initialized yet
    ASSERT_TRUE(mpc.initializeSolver());
    ASSERT_EQ(mpc.solve(x0), ampc::SolveStatus::Success);
    EXPECT_TRUE(mpc.isWithinSparsityPattern());

    // the solver silently drops the new coupling terms
    setMsdModel(mpc);
    (void)mpc.solve(x0);
    EXPECT_FALSE(mpc.isWithinSparsityPattern());

    // back within the pattern (w never affects the pattern)
    mpc.setModelDiscrete(Ad_diag * 0.5, Bd, Vector2d{0.1, 0.2});
    ASSERT_EQ(mpc.solve(x0), ampc::SolveStatus::Success);
    EXPECT_TRUE(mpc.isWithinSparsityPattern());
  };

  ampc::CondensedMPC condensed{n, m, param, opts};
  check(condensed);

  ampc::SparseMPC sparse{n, m, param, opts};
  check(sparse);
}

TEST(ConsistencyTester, givenCondensedModelZeroBecomingNonzero_CanStayInPattern)
{
  // Without state saturation, A_qp has no model terms and P = S^T Q S is
  // already dense, so a model zero becoming nonzero adds no QP nonzeros
  const int n{2}, m{1}, T{10};
  ampc::CondensedMPC mpc{n, m, T};

  Matrix2d Ad;
  Ad << 1, 0.1, 0, 0.99;
  const Vector2d Bd{0, 0.2}, wd{0, 0};
  mpc.setModelDiscrete(Ad, Bd, wd);
  mpc.setInputLimits(VectorXd::Constant(m, -5.0), VectorXd::Constant(m, 5.0));
  mpc.setReferenceState(Vector2d{1.0, 0.0});
  ASSERT_TRUE(mpc.initializeSolver());

  Ad(1, 0) = -0.06;
  mpc.setModelDiscrete(Ad, Bd, wd);
  ASSERT_EQ(mpc.solve(Vector2d::Zero()), ampc::SolveStatus::Success);
  EXPECT_TRUE(mpc.isWithinSparsityPattern());
}

TEST(ConsistencyTester, askedForSolveInfo_ReportsLastSolve)
{
  const int n{2}, m{1}, T{10}, nc{5};
  const auto param{ampc::Parameterization::linearInterp(T, nc)};
  const ampc::Options opts{.use_input_cost = true};

  auto check = [&](ampc::MPCBase& mpc) {
    EXPECT_EQ(mpc.getSolveInfo().status, ampc::SolveStatus::NotInitialized);

    configureMsd(mpc, opts);
    ASSERT_TRUE(mpc.initializeSolver());
    // OSQP reports "unsolved" between initialization and the first solve
    EXPECT_EQ(mpc.getSolveInfo().status, ampc::SolveStatus::OtherFailure);

    const ampc::SolveStatus status{mpc.solve(Vector2d{0.5, -0.2})};
    ASSERT_EQ(status, ampc::SolveStatus::Success);
    const ampc::SolveInfo info{mpc.getSolveInfo()};
    EXPECT_EQ(info.status, status);
    EXPECT_GT(info.iterations, 0);
    EXPECT_TRUE(std::isfinite(info.objective));
    EXPECT_GE(info.run_time, info.solve_time);
  };

  ampc::CondensedMPC condensed{n, m, param, opts};
  check(condensed);

  ampc::SparseMPC sparse{n, m, param, opts};
  check(sparse);
}

TEST(ConsistencyTester, askedForQPCostMatrix_ReflectsLastSolve)
{
  const int n{2}, m{1}, T{10}, nc{5};
  const auto param{ampc::Parameterization::linearInterp(T, nc)};
  const ampc::Options opts{.use_input_cost = true};
  const Vector2d Q{2.0, 0.0};
  const VectorXd R{VectorXd::Constant(m, 0.5)};
  const Vector2d x0{0.5, -0.2};

  // SparseMPC: P is diagonal with R on the control points and Q on the states
  ampc::SparseMPC sparse{n, m, param, opts};
  configureMsd(sparse, opts);
  sparse.setWeights(Q, R);
  ASSERT_TRUE(sparse.initializeSolver());
  // unit placeholder weights until the first solve
  EXPECT_TRUE(sparse.getQPCostMatrix().isIdentity());
  ASSERT_EQ(sparse.solve(x0), ampc::SolveStatus::Success);

  const int ctrls_dim{m * nc}, x_traj_dim{n * T};
  MatrixXd P_expected{
      MatrixXd::Zero(ctrls_dim + x_traj_dim, ctrls_dim + x_traj_dim)};
  P_expected.diagonal().head(ctrls_dim) = R.replicate(nc, 1);
  P_expected.diagonal().tail(x_traj_dim) = Q.replicate(T, 1);
  EXPECT_TRUE(expectEigenNear(sparse.getQPCostMatrix(), P_expected, 1e-15));

  // CondensedMPC: P is dense over the control points and updated at the next
  // solve after a weight change
  ampc::CondensedMPC condensed{n, m, param, opts};
  configureMsd(condensed, opts);
  condensed.setWeights(Q, R);
  ASSERT_TRUE(condensed.initializeSolver());
  ASSERT_EQ(condensed.solve(x0), ampc::SolveStatus::Success);

  const MatrixXd P_first{condensed.getQPCostMatrix()};
  ASSERT_EQ(P_first.rows(), ctrls_dim);
  ASSERT_EQ(P_first.cols(), ctrls_dim);
  EXPECT_TRUE(P_first.isApprox(P_first.transpose()));

  // R only adds to the diagonal of P = S^T Q S + R
  condensed.setInputWeights(R * 3);
  EXPECT_TRUE(expectEigenNear(condensed.getQPCostMatrix(), P_first, 1e-15));
  ASSERT_EQ(condensed.solve(x0), ampc::SolveStatus::Success);
  MatrixXd P_diff{condensed.getQPCostMatrix() - P_first};
  MatrixXd P_diff_expected{MatrixXd::Zero(ctrls_dim, ctrls_dim)};
  P_diff_expected.diagonal() = (2 * R).replicate(nc, 1);
  EXPECT_TRUE(expectEigenNear(P_diff, P_diff_expected, 1e-12));
}
