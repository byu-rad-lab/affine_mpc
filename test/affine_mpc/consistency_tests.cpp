#include "affine_mpc/condensed_mpc.hpp"
#include "affine_mpc/options.hpp"
#include "affine_mpc/parameterization.hpp"
#include "affine_mpc/sparse_mpc.hpp"

#include <Eigen/Core>
#include <cmath>
#include <cstdlib>
#include <gtest/gtest.h>
#include <limits>

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

TEST(ConsistencyTester, askedIfSolverInitialized_TracksInitializeSolver)
{
  const int n{2}, m{1}, T{5}, nc{3};
  const auto param{ampc::Parameterization::linearInterp(T, nc)};
  ConsistencyTester tester{n, m, param};
  EXPECT_FALSE(tester.condensed.isSolverInitialized());
  EXPECT_FALSE(tester.sparse.isSolverInitialized());

  tester.setup();
  EXPECT_TRUE(tester.condensed.isSolverInitialized());
  EXPECT_TRUE(tester.sparse.isSolverInitialized());
}

TEST(ConsistencyTester, givenWrongSizeSolveAndGetterArgs_Throws)
{
  const int n{2}, m{1}, T{5}, nc{3};
  const auto param{ampc::Parameterization::linearInterp(T, nc)};
  ConsistencyTester tester{n, m, param};

  // sizes are validated before the initialization check
  EXPECT_THROW((void)tester.condensed.solve(VectorXd::Zero(3)),
               std::invalid_argument);
  EXPECT_THROW((void)tester.sparse.solve(VectorXd::Zero(3)),
               std::invalid_argument);

  tester.setup();
  auto check = [&](ampc::MPCBase& mpc) {
    expectInvalidArgumentWithMessage(
        [&]() { (void)mpc.solve(VectorXd::Zero(3)); },
        "[MPCBase::solve] x0 must have size 2, got 3.");
    ASSERT_EQ(mpc.solve(Vector2d::Zero()), ampc::SolveStatus::Success);

    VectorXd wrong{VectorXd::Zero(4)};
    expectInvalidArgumentWithMessage([&]() { mpc.getNextInput(wrong); },
                                     "u0 must have size 1, got 4");
    expectInvalidArgumentWithMessage(
        [&]() { mpc.getInputControlPoints(wrong); },
        "control_points must have size 3, got 4");
    expectInvalidArgumentWithMessage([&]() { mpc.getInputTrajectory(wrong); },
                                     "u_traj must have size 5, got 4");
    expectInvalidArgumentWithMessage(
        [&]() { mpc.getPredictedStateTrajectory(wrong); },
        "x_traj must have size 10, got 4");
  };
  check(tester.condensed);
  check(tester.sparse);
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

TEST(ConsistencyTester,
     givenDeg1InputTrajectorySaturation_CondensedAndSparseMPCAgree)
{
  // non-integer knots: only knot-adjacent samples are constrained
  const int n{2}, m{1}, T{10};
  VectorXd knots{6};
  knots << 0, 0, 2.5, 6.5, 9, 9;
  const ampc::Parameterization param{T, 1, knots};
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

  // every sample respects the limits, and the upper limit is active
  EXPECT_NEAR(u_traj_sparse.maxCoeff(), 3.0, 1e-4);
  EXPECT_GE(u_traj_sparse.minCoeff(), -1e-4);

  VectorXd x_traj_condensed{n * T}, x_traj_sparse{n * T};
  tester.condensed.getPredictedStateTrajectory(x_traj_condensed);
  tester.sparse.getPredictedStateTrajectory(x_traj_sparse);
  ASSERT_TRUE(expectEigenNear(x_traj_condensed, x_traj_sparse, 1e-4));
}

TEST(ConsistencyTester,
     givenDeg1InputTrajectorySaturation_ControlPointExceedsLimitsButInputsDoNot)
{
  // The initial slew limit (u_prev = 0, slew = 0.5) keeps c0 = u_0 <= 0.5. A
  // low reference (steady-state input 0.9) is reached fastest by ramping to
  // u_max = 3 and backing off, so the optimizer pushes the (unsampled) control
  // point at knot 2.5 above u_max while every sample stays within limits.
  const int n{2}, m{1}, T{10};
  VectorXd knots{6};
  knots << 0, 0, 2.5, 6.5, 9, 9;
  const ampc::Parameterization param{T, 1, knots};
  const Vector2d x0{0, 0}, x_ref{0.3, 0};
  const double u_max{3.0};

  ConsistencyTester tester{
      n,
      m,
      param,
      {.slew_initial_input = true, .saturate_input_trajectory = true}};
  tester.setup(); // input limits are [0, 3]
  for (ampc::MPCBase* mpc : {static_cast<ampc::MPCBase*>(&tester.condensed),
                             static_cast<ampc::MPCBase*>(&tester.sparse)}) {
    mpc->setReferenceState(x_ref);
    ASSERT_EQ(mpc->solve(x0), ampc::SolveStatus::Success);
    VectorXd ctrls{m * param.num_control_points}, u_traj{m * T};
    mpc->getInputControlPoints(ctrls);
    mpc->getInputTrajectory(u_traj);
    EXPECT_GT(ctrls(1), u_max + 1e-2);
    EXPECT_NEAR(u_traj.maxCoeff(), u_max, 1e-4);
    EXPECT_GE(u_traj.minCoeff(), -1e-4);
  }

  // saturating the control points instead caps u_2 at 0.2*0.5 + 0.8*3 = 2.5
  ConsistencyTester tester_ctrls{n, m, param, {.slew_initial_input = true}};
  tester_ctrls.setup();
  tester_ctrls.condensed.setReferenceState(x_ref);
  ASSERT_EQ(tester_ctrls.condensed.solve(x0), ampc::SolveStatus::Success);
  VectorXd ctrls{m * param.num_control_points}, u_traj{m * T};
  tester_ctrls.condensed.getInputControlPoints(ctrls);
  tester_ctrls.condensed.getInputTrajectory(u_traj);
  EXPECT_LE(ctrls.maxCoeff(), u_max + 1e-4);
  EXPECT_LE(u_traj(2), 2.5 + 1e-4);
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

TEST(ConsistencyTester, askedToResetWarmStart_NextSolveStartsFromZero)
{
  const int n{2}, m{1}, T{10}, nc{5};
  const auto param{ampc::Parameterization::linearInterp(T, nc)};
  const ampc::Options opts{.use_input_cost = true};
  OSQPSettings settings{ampc::OSQPSolver::getRecommendedSettings()};
  // deterministic iteration counts: fixed rho, check termination every step
  settings.adaptive_rho = false;
  settings.check_termination = 1;
  const Vector2d x0{0.5, -0.2};

  auto check = [&](ampc::MPCBase& mpc) {
    mpc.resetWarmStart(); // no-op before initialization
    configureMsd(mpc, opts);
    ASSERT_TRUE(mpc.initializeSolver(settings));

    ASSERT_EQ(mpc.solve(x0), ampc::SolveStatus::Success);
    const int iters_from_zero{mpc.getSolveInfo().iterations};
    VectorXd u_first{m};
    mpc.getNextInput(u_first);

    ASSERT_EQ(mpc.solve(x0), ampc::SolveStatus::Success);
    EXPECT_LT(mpc.getSolveInfo().iterations, iters_from_zero);
    VectorXd u_warm{m}, u_after_reset{m};
    mpc.getNextInput(u_warm);

    // the getters still return the last solve's results
    mpc.resetWarmStart();
    mpc.getNextInput(u_after_reset);
    expectEigenNear(u_after_reset, u_warm, 0.0);

    ASSERT_EQ(mpc.solve(x0), ampc::SolveStatus::Success);
    EXPECT_EQ(mpc.getSolveInfo().iterations, iters_from_zero);
    VectorXd u_cold{m};
    mpc.getNextInput(u_cold);
    expectEigenNear(u_cold, u_first, 0.0);
  };

  ampc::CondensedMPC condensed{n, m, param, opts};
  check(condensed);

  ampc::SparseMPC sparse{n, m, param, opts};
  check(sparse);
}

TEST(ConsistencyTester,
     givenNonFiniteUpdate_SolveReportsUpdateFailedAndRecovers)
{
  const int n{2}, m{1}, T{10}, nc{5};
  const auto param{ampc::Parameterization::linearInterp(T, nc)};
  OSQPSettings settings{ampc::OSQPSolver::getRecommendedSettings()};
  // deterministic iteration counts: fixed rho, check termination every step
  settings.adaptive_rho = false;
  settings.check_termination = 1;
  const Vector2d x0{0.5, -0.2};
  const double nan{std::numeric_limits<double>::quiet_NaN()};

  // OSQP rejects the matrix update, so solve() must not report Success on
  // the stale QP. Restoring valid data must recover the original solve.
  auto check = [&](ampc::MPCBase& mpc, const ampc::Options& opts,
                   bool nan_in_weights) {
    configureMsd(mpc, opts);
    mpc.setStateWeights(Vector2d::Ones());
    ASSERT_TRUE(mpc.initializeSolver(settings));

    ASSERT_EQ(mpc.solve(x0), ampc::SolveStatus::Success);
    const int iters_first{mpc.getSolveInfo().iterations};
    VectorXd u_first{m}, u{m};
    mpc.getNextInput(u_first);

    if (nan_in_weights)
      mpc.setStateWeights(Vector2d{nan, 1.0});
    else
      mpc.setModelDiscrete(Matrix2d::Identity(), Vector2d{0.0, nan},
                           Vector2d::Zero());
    EXPECT_EQ(mpc.solve(x0), ampc::SolveStatus::UpdateFailed);
    // stays failed while the data is invalid
    EXPECT_EQ(mpc.solve(x0), ampc::SolveStatus::UpdateFailed);
    // the getters still return the last successful solve
    mpc.getNextInput(u);
    expectEigenNear(u, u_first, 0.0);

    if (nan_in_weights)
      mpc.setStateWeights(Vector2d::Ones());
    else
      setMsdModel(mpc);
    mpc.resetWarmStart();
    ASSERT_EQ(mpc.solve(x0), ampc::SolveStatus::Success);
    EXPECT_EQ(mpc.getSolveInfo().iterations, iters_first);
    mpc.getNextInput(u);
    expectEigenNear(u, u_first, 1e-12);
  };

  for (const bool saturate_states : {false, true}) {
    const ampc::Options opts{.use_input_cost = true,
                             .saturate_states = saturate_states};
    for (const bool nan_in_weights : {false, true}) {
      SCOPED_TRACE(::testing::Message()
                   << "saturate_states=" << saturate_states
                   << " nan_in_weights=" << nan_in_weights);
      {
        SCOPED_TRACE("CondensedMPC");
        ampc::CondensedMPC condensed{n, m, param, opts};
        check(condensed, opts, nan_in_weights);
      }
      {
        SCOPED_TRACE("SparseMPC");
        ampc::SparseMPC sparse{n, m, param, opts};
        check(sparse, opts, nan_in_weights);
      }
    }
  }
}

// ---- exhaustive QP equivalence over every option combination ---------------

// Exposes the assembled QP (P, A, q, l, u) of a formulation for comparison.
template <class Formulation> class QPAccess : public Formulation
{
public:
  using Formulation::Formulation;
  void assemble(const Ref<const VectorXd>& x0) { (void)this->qpUpdateX0(x0); }
  const MatrixXd& P() const { return this->P_; }
  const MatrixXd& A() const { return this->A_; }
  const VectorXd& q() const { return this->q_; }
  const VectorXd& l() const { return this->l_; }
  const VectorXd& u() const { return this->u_; }
};

// Checks that CondensedMPC and SparseMPC define the same QP, without solving:
// for random control points z, with y = [z; x(z)] where x(z) propagates the
// model, the costs must differ by the same constant for every z, the sparse
// model rows must hold exactly, and every other constraint row must have the
// same margins to its bounds (the rows match one to one after the model rows).
void expectSameQP(const QPAccess<ampc::CondensedMPC>& condensed,
                  const QPAccess<ampc::SparseMPC>& sparse,
                  const ampc::Parameterization& param,
                  const VectorXd& x0)
{
  const int n{static_cast<int>(x0.size())};
  const int nz{static_cast<int>(condensed.q().size())};
  const int m{nz / param.num_control_points};
  const int T{param.horizon_steps};
  const int num_model_rows{n * T};
  const double tol{1e-9};

  ASSERT_EQ(sparse.A().rows(), num_model_rows + condensed.A().rows());

  std::srand(1);
  double cost_offset{0.0};
  for (int trial{0}; trial < 4; ++trial) {
    const VectorXd z{2.0 * VectorXd::Random(nz)};

    // y = [z; x(z)]
    const VectorXd u_traj{param.evaluate(m, z)};
    VectorXd y{nz + num_model_rows}, x{x0};
    y.head(nz) = z;
    for (int k{0}; k < T; ++k) {
      condensed.propagateModel(x, u_traj.segment(k * m, m), x);
      y.segment(nz + k * n, n) = x;
    }

    const double cost_c{0.5 * z.dot(condensed.P() * z) + condensed.q().dot(z)};
    const double cost_s{0.5 * y.dot(sparse.P() * y) + sparse.q().dot(y)};
    if (trial == 0)
      cost_offset = cost_c - cost_s;
    else
      EXPECT_NEAR(cost_c - cost_s, cost_offset, tol * (1.0 + std::abs(cost_c)))
          << "cost differs by more than a constant";

    const VectorXd Ay{sparse.A() * y};
    const VectorXd Az{condensed.A() * z};
    const int num_shared{static_cast<int>(Az.size())};
    const VectorXd model_rows{Ay.head(num_model_rows)};
    const VectorXd model_l{sparse.l().head(num_model_rows)};
    const VectorXd model_u{sparse.u().head(num_model_rows)};
    expectEigenNear(model_rows, model_l, tol);
    expectEigenNear(model_u, model_l, 0.0);

    const VectorXd lower_margin_s{Ay.tail(num_shared)
                                  - sparse.l().tail(num_shared)};
    const VectorXd upper_margin_s{sparse.u().tail(num_shared)
                                  - Ay.tail(num_shared)};
    const VectorXd lower_margin_c{Az - condensed.l()};
    const VectorXd upper_margin_c{condensed.u() - Az};
    expectEigenNear(lower_margin_c, lower_margin_s, tol);
    expectEigenNear(upper_margin_c, upper_margin_s, tol);
  }
}

TEST(ConsistencyTester, givenEveryOptionCombination_CondensedAndSparseQPsMatch)
{
  const int T{10}, nc{5};

  struct System
  {
    const char* name;
    MatrixXd Ad, Bd;
    VectorXd wd, x0;
  };
  System siso{"SISO", MatrixXd(2, 2), MatrixXd(2, 1), Vector2d{0.0, 0.01},
              Vector2d{0.3, -0.1}};
  siso.Ad << 0.997, 0.0998, -0.0599, 0.987;
  siso.Bd << 0.001, 0.02;
  System mimo{"MIMO", MatrixXd(3, 3), MatrixXd(3, 2),
              Vector3d{0.01, 0.0, -0.02}, Vector3d{0.2, -0.1, 0.3}};
  mimo.Ad << 0.9, 0.1, 0.0, 0.0, 0.8, 0.1, 0.0, 0.0, 0.9;
  mimo.Bd << 0.1, 0.0, 0.0, 0.1, 0.05, 0.05;

  for (int mask{0}; mask < 32; ++mask) {
    ampc::Options opts;
    opts.use_input_cost = mask & 1;
    opts.slew_initial_input = mask & 2;
    opts.slew_control_points = mask & 4;
    opts.saturate_states = mask & 8;
    opts.saturate_input_trajectory = mask & 16;

    for (int degree{0}; degree <= 3; ++degree) {
      const auto param{ampc::Parameterization::bspline(T, degree, nc)};
      for (const System* sys : {&siso, &mimo}) {
        SCOPED_TRACE(::testing::Message()
                     << sys->name << " degree=" << degree
                     << " use_input_cost=" << opts.use_input_cost
                     << " slew_initial_input=" << opts.slew_initial_input
                     << " slew_control_points=" << opts.slew_control_points
                     << " saturate_states=" << opts.saturate_states
                     << " saturate_input_trajectory="
                     << opts.saturate_input_trajectory);
        const int n{static_cast<int>(sys->Ad.rows())};
        const int m{static_cast<int>(sys->Bd.cols())};

        // distinct, nonuniform values so mismatched rows or columns show up
        auto configure = [&](ampc::MPCBase& mpc, double scale) {
          mpc.setModelDiscrete(sys->Ad, scale * sys->Bd, scale * sys->wd);
          mpc.setInputLimits(-scale * VectorXd::LinSpaced(m, 1.0, 2.0),
                             scale * VectorXd::LinSpaced(m, 1.5, 2.5));
          mpc.setStateWeights(scale * VectorXd::LinSpaced(n, 1.0, 2.0),
                              scale * VectorXd::LinSpaced(n, 3.0, 4.0));
          mpc.setReferenceStateTrajectory(
              scale * VectorXd::LinSpaced(n * T, 0.5, 1.5));
          if (opts.use_input_cost) {
            mpc.setInputWeights(scale * VectorXd::LinSpaced(m, 0.1, 0.2));
            mpc.setReferenceInputControlPoints(
                scale * VectorXd::LinSpaced(m * nc, -0.3, 0.3));
          }
          if (opts.slew_initial_input) {
            mpc.setSlewRateInitial(scale * VectorXd::LinSpaced(m, 0.4, 0.6));
            mpc.setPreviousInput(scale * VectorXd::LinSpaced(m, 0.1, 0.2));
          }
          if (opts.slew_control_points)
            mpc.setSlewRate(scale * VectorXd::LinSpaced(m, 0.3, 0.5));
          if (opts.saturate_states)
            mpc.setStateLimits(-scale * VectorXd::LinSpaced(n, 5.0, 6.0),
                               scale * VectorXd::LinSpaced(n, 7.0, 8.0));
        };

        QPAccess<ampc::CondensedMPC> condensed{n, m, param, opts};
        QPAccess<ampc::SparseMPC> sparse{n, m, param, opts};
        configure(condensed, 1.0);
        configure(sparse, 1.0);
        ASSERT_TRUE(condensed.initializeSolver());
        ASSERT_TRUE(sparse.initializeSolver());

        // after initialization (sparse weights are applied at the next update)
        condensed.assemble(sys->x0);
        sparse.assemble(sys->x0);
        {
          SCOPED_TRACE("after initialization");
          expectSameQP(condensed, sparse, param, sys->x0);
        }

        // after runtime updates of every parameter, within the sparsity pattern
        configure(condensed, 1.3);
        configure(sparse, 1.3);
        const VectorXd x0_new{-0.5 * sys->x0};
        condensed.assemble(x0_new);
        sparse.assemble(x0_new);
        {
          SCOPED_TRACE("after runtime updates");
          expectSameQP(condensed, sparse, param, x0_new);
        }
        EXPECT_TRUE(condensed.isWithinSparsityPattern());
        EXPECT_TRUE(sparse.isWithinSparsityPattern());
      }
    }
  }
}
