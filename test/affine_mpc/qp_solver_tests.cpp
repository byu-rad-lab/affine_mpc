#include "affine_mpc/mpc.hpp"
#include "affine_mpc/options.hpp"
#include "affine_mpc/osqp_solver.hpp"
#include "affine_mpc/parameterization.hpp"
#include "affine_mpc/qp_solver.hpp"

#include <Eigen/Core>
#include <gtest/gtest.h>
#include <memory>

#include "utils.hpp"

using namespace Eigen;
namespace ampc = affine_mpc;

namespace {

void setupMassSpringDamper(ampc::MPC& mpc)
{
  Matrix2d Ac;
  Ac << 0, 1, -0.6, -0.1;
  mpc.setModelContinuous2Discrete(Ac, Vector2d{0, 0.2}, Vector2d::Zero(), 0.1);
  mpc.setInputLimits(VectorXd::Constant(1, 0.0), VectorXd::Constant(1, 3.0));
  mpc.setSlewRate(VectorXd::Constant(1, 1.0));
  mpc.setWeights(Vector2d{1.0, 0.1}, VectorXd::Constant(1, 1e-5));
  mpc.setReferenceState(Vector2d{1.0, 0.0});
  mpc.setReferenceInput(VectorXd::Constant(1, 3.0));
}

const ampc::Options msd_opts{.input_cost = ampc::InputCostMode::ControlPoints,
                             .slew_control_points = true};

} // namespace

class QPSolverInterfaceTester : public testing::TestWithParam<ampc::Formulation>
{};

INSTANTIATE_TEST_SUITE_P(Formulations,
                         QPSolverInterfaceTester,
                         testing::Values(ampc::Formulation::Condensed,
                                         ampc::Formulation::Sparse));

TEST_P(QPSolverInterfaceTester,
       givenOSQPSolverObject_SolvesExactlyLikeOSQPSettings)
{
  const auto param{ampc::Parameterization::linearInterp(10, 3)};
  ampc::MPC by_settings{2, 1, param, msd_opts, GetParam()};
  ampc::MPC by_object{2, 1, param, msd_opts, GetParam()};
  setupMassSpringDamper(by_settings);
  setupMassSpringDamper(by_object);

  OSQPSettings settings{ampc::OSQPSolver::getRecommendedSettings(true)};
  settings.eps_abs = settings.eps_rel = 1e-7;
  ASSERT_TRUE(by_settings.initializeSolver(settings));
  ASSERT_TRUE(by_object.initializeSolver(std::make_unique<ampc::OSQPSolver>(
      by_object.getNumQPVariables(), by_object.getNumQPConstraints(),
      settings)));

  Vector2d x{Vector2d::Zero()};
  VectorXd u1(1), u2(1);
  for (int k{0}; k < 30; ++k) {
    if (k == 15) {
      // matrix updates take the same path
      Matrix2d Ac;
      Ac << 0, 1, -0.8, -0.1;
      for (ampc::MPC* mpc : {&by_settings, &by_object})
        mpc->setModelContinuous2Discrete(Ac, Vector2d{0, 0.2}, Vector2d::Zero(),
                                         0.1);
    }
    ASSERT_EQ(by_settings.solve(x), ampc::SolveStatus::Success);
    ASSERT_EQ(by_object.solve(x), ampc::SolveStatus::Success);
    by_settings.getNextInput(u1);
    by_object.getNextInput(u2);
    EXPECT_EQ(u1(0), u2(0)) << "step " << k;
    EXPECT_EQ(by_settings.getSolveInfo().iterations,
              by_object.getSolveInfo().iterations);
    by_settings.propagateModel(x, u1, x);
  }
}

TEST(QPSolverInterfaceTester, givenNullSolver_ThrowsInvalidArgument)
{
  const auto param{ampc::Parameterization::linearInterp(10, 3)};
  ampc::MPC mpc{2, 1, param, msd_opts};
  setupMassSpringDamper(mpc);
  expectInvalidArgumentWithMessage(
      [&] { (void)mpc.initializeSolver(std::unique_ptr<ampc::QPSolver>{}); },
      "[MPC::initializeSolver] The solver must not be null.");
  EXPECT_FALSE(mpc.isSolverInitialized());
}

TEST(QPSolverInterfaceTester, givenInitializedSolver_ThrowsInvalidArgument)
{
  const auto param{ampc::Parameterization::linearInterp(10, 3)};
  ampc::MPC first{2, 1, param, msd_opts}, second{2, 1, param, msd_opts};
  setupMassSpringDamper(first);
  setupMassSpringDamper(second);
  auto solver{std::make_unique<ampc::OSQPSolver>(first.getNumQPVariables(),
                                                 first.getNumQPConstraints())};
  const auto P{
      MatrixXd::Identity(first.getNumQPVariables(), first.getNumQPVariables())};
  const auto A{MatrixXd::Identity(first.getNumQPConstraints(),
                                  first.getNumQPVariables())};
  const VectorXd q{VectorXd::Zero(first.getNumQPVariables())};
  const VectorXd b{VectorXd::Ones(first.getNumQPConstraints())};
  ASSERT_TRUE(solver->initialize(P, A, q, -b, b,
                                 ampc::OSQPSolver::getDefaultSettings()));
  EXPECT_THROW((void)second.initializeSolver(std::move(solver)),
               std::invalid_argument);
}

TEST(QPSolverInterfaceTester, givenInitializedMPC_IgnoresAnotherSolver)
{
  const auto param{ampc::Parameterization::linearInterp(10, 3)};
  ampc::MPC mpc{2, 1, param, msd_opts};
  setupMassSpringDamper(mpc);
  ASSERT_TRUE(mpc.initializeSolver());
  EXPECT_TRUE(mpc.initializeSolver(std::unique_ptr<ampc::QPSolver>{}));
  EXPECT_EQ(mpc.solve(Vector2d::Zero()), ampc::SolveStatus::Success);
}

TEST(QPSolverInterfaceTester, beforeInitialization_SolverQueriesAreSafe)
{
  const auto param{ampc::Parameterization::linearInterp(10, 3)};
  ampc::MPC mpc{2, 1, param, msd_opts};
  setupMassSpringDamper(mpc);
  mpc.resetWarmStart();
  EXPECT_EQ(mpc.getSolveInfo().status, ampc::SolveStatus::NotInitialized);
  EXPECT_TRUE(mpc.isWithinSparsityPattern());
  EXPECT_EQ(mpc.solve(Vector2d::Zero()), ampc::SolveStatus::NotInitialized);
}
