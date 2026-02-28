#include "affine_mpc/condensed_mpc.hpp"
#include "affine_mpc/options.hpp"
#include "affine_mpc/parameterization.hpp"
#include "affine_mpc/sparse_mpc.hpp"

#include <Eigen/Core>
#include <gtest/gtest.h>

#include "utils.hpp"

using namespace Eigen;
namespace ampc = affine_mpc;

// ---- CondensedMPC vs SparseMPC consistency test ----------------------------

TEST(ConsistencyTest, givenSameSystem_CondensedAndSparseMPCAgree)
{
  const int n{2}, m{1}, T{10}, nc{10};
  const ampc::Parameterization param{
      ampc::Parameterization::linearInterp(T, nc)};
  const ampc::Options opts{.use_input_cost = true};

  ampc::CondensedMPC condensed{n, m, param, opts};
  ampc::SparseMPC sparse{n, m, param, opts};

  Matrix2d A;
  Vector2d B, w;
  A << 0, 1, -0.6, -0.1;
  B << 0, 0.2;
  w.setZero();

  // Apply identical setup to both
  auto setup = [&](auto& mpc) {
    mpc.setModelContinuous2Discrete(A, B, w, 0.1);
    mpc.setWeights(Vector2d{1.0, 0.1}, VectorXd::Constant(m, 1e-4));
    mpc.setReferenceState(Vector2d{1.0, 0.0});
    mpc.setInputLimits(VectorXd::Constant(m, 0.0), VectorXd::Constant(m, 3.0));
    mpc.setReferenceInput(VectorXd::Zero(m));
    auto settings{ampc::OSQPSolver::getRecommendedSettings(true)};
    ASSERT_TRUE(mpc.initializeSolver(settings));
  };
  setup(condensed);
  setup(sparse);

  const Vector2d x0{0.5, -0.2};
  ASSERT_TRUE(condensed.solve(x0));
  ASSERT_TRUE(sparse.solve(x0));

  VectorXd u_condensed{m}, u_sparse{m};
  condensed.getNextInput(u_condensed);
  sparse.getNextInput(u_sparse);

  ASSERT_TRUE(expectEigenNear(u_condensed, u_sparse, 1e-4));

  // Also verify the full predicted trajectory agrees
  VectorXd x_traj_condensed{n * T}, x_traj_sparse{n * T};
  condensed.getPredictedStateTrajectory(x_traj_condensed);
  sparse.getPredictedStateTrajectory(x_traj_sparse);

  ASSERT_TRUE(expectEigenNear(x_traj_condensed, x_traj_sparse, 1e-4));
}
