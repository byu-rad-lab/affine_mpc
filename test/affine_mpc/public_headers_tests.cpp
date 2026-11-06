// Uses the library through its public headers only. This target gets no
// OSQP include directories (OSQP is a private dependency), and the check
// below also fails if a public header includes an OSQP header found some
// other way.
#include <affine_mpc/affine_mpc.hpp>

#if defined(OSQP_H) || defined(OSQP_API_TYPES_H) || defined(OSQP_CONFIGURE_H)
#error "a public affine_mpc header includes an OSQP header"
#endif

#include <Eigen/Core>
#include <gtest/gtest.h>
#include <memory>

namespace ampc = affine_mpc;

TEST(PublicHeadersTester, givenOnlyPublicHeaders_SetsUpAndSolves)
{
  const auto param{ampc::Parameterization::moveBlocking(10, 3)};
  ampc::MPC mpc{2, 1, param};
  Eigen::Matrix2d Ac;
  Ac << 0, 1, -0.6, -0.1;
  mpc.setModelContinuous2Discrete(Ac, Eigen::Vector2d{0, 0.2},
                                  Eigen::Vector2d::Zero(), 0.1);
  mpc.setInputLimits(Eigen::VectorXd::Constant(1, 0.0),
                     Eigen::VectorXd::Constant(1, 3.0));
  mpc.setStateWeights(Eigen::Vector2d{1.0, 0.1});
  mpc.setReferenceState(Eigen::Vector2d{1.0, 0.0});

  ampc::OSQPSettings settings{ampc::OSQPSolver::getRecommendedSettings()};
  settings.linsys_solver = ampc::OSQPSettings::LinsysSolverType::DirectSolver;
  settings.max_iter = 2000;
  auto solver{std::make_unique<ampc::OSQPSolver>(
      mpc.getNumQPVariables(), mpc.getNumQPConstraints(), settings)};
  EXPECT_EQ(solver->getSettings().max_iter, 2000);
  ASSERT_TRUE(mpc.initializeSolver(std::move(solver)));
  EXPECT_EQ(mpc.solve(Eigen::Vector2d::Zero()), ampc::SolveStatus::Success);
}
