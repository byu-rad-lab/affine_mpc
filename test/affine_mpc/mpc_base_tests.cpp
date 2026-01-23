#include <Eigen/Core>
#include <gtest/gtest.h>

#include "affine_mpc/mpc_base.hpp"
#include "utils.hpp"


// Tests use varying parameters for the MPCBase class, so a GTest fixture is not
// used
class MPCBaseTester : public affine_mpc::MPCBase
{
public:
  MPCBaseTester(const int n,
                const int m,
                const int T,
                const int mu,
                const bool Ju = false) :
      MPCBase(n, m, T, mu, 1, Eigen::VectorXd{0}, Ju)
  {}
  virtual ~MPCBaseTester() = default;
  void
  getPredictedStateTrajectory(Eigen::Ref<Eigen::VectorXd> x_traj) const override
  {}
  auto getAd() { return Ad_; }
  auto getBd() { return Bd_; }
  auto getWd() { return wd_; }
  auto getQbig() { return Q_big_; }
  auto getRbig() { return R_big_; }
  auto getStateTrajectory() { return x_goal_; }
  auto getInputTrajectory() { return u_goal_; }

protected:
  void convertToQP(const Eigen::Ref<const Eigen::VectorXd>& x0) override {}
};

TEST(MPCBaseTester, givenContinuousLinearSystem_DiscretizesCorrectly)
{
  const int n{2}, m{1}, T{5}, mu{3};
  MPCBaseTester base{n, m, T, mu};

  Eigen::MatrixXd A{n, n}, B{n, m}, w{n, 1};
  A << 0, 1, -0.6, -0.1;
  B << 0, 0.2;
  w.setZero();

  double ts{0.1};
  base.setModelContinuous2Discrete(A, B, w, ts);

  Eigen::MatrixXd Ad_expected{n, n}, Bd_expected{n, m}, wd_expected{n, 1};
  Ad_expected << 0.99701147, 0.09940219, -0.05964131, 0.98707125;
  Bd_expected << 0.00099618, 0.01988044;
  wd_expected.setZero();

  ASSERT_TRUE(expectEigenNear(Ad_expected, base.getAd(), 1e-6));
  ASSERT_TRUE(expectEigenNear(Bd_expected, base.getBd(), 1e-6));
  ASSERT_TRUE(expectEigenNear(wd_expected, base.getWd(), 1e-6));
}

// This is a Quadcopter system (with throttle and angular rates as inputs)
// linearized about equilibrium
TEST(MPCBaseTester,
     givenContinuousSystemLinearizedAtEquilibrium_DiscretizesCorrectly)
{
  const int n{9}, m{4}, T{5}, mu{3};
  MPCBaseTester base{n, m, T, mu};

  Eigen::MatrixXd A{n, n}, B{n, m}, w{n, 1};
  A.setZero();
  A(0, 6) = A(1, 7) = A(2, 8) = 1;
  A(6, 4) = -9.81;
  A(7, 3) = 9.81;
  A(6, 6) = A(7, 7) = -0.1;

  B.setZero();
  B(8, 0) = -9.81 / 0.5;
  B(3, 1) = B(4, 2) = B(5, 3) = 1;

  w.setZero();

  double dt{0.1};
  base.setModelContinuous2Discrete(A, B, w, dt);

  Eigen::MatrixXd Ad_expected{n, n}, Bd_expected{n, m}, wd_expected{n, 1};
  // clang-format off
  Ad_expected << 1, 0, 0, 0,     -0.0488869, 0, 0.0995017, 0,   0,
                 0, 1, 0, 0.0488869, 0,      0, 0,   0.0995017, 0,
                 0, 0, 1, 0,         0,      0, 0,         0,   0.1,
                 0, 0, 0, 1,         0,      0, 0,         0,   0,
                 0, 0, 0, 0,         1,      0, 0,         0,   0,
                 0, 0, 0, 0,         0,      1, 0,         0,   0,
                 0, 0, 0, 0,     -0.9761113, 0, 0.9900498, 0,   0,
                 0, 0, 0, 0.9761113, 0,      0, 0,   0.9900498, 0,
                 0, 0, 0, 0,         0,      0, 0,         0,   1;

  Bd_expected << 0,      0, -0.0016309, 0,
                 0,   0.0016309, 0,     0,
                -0.0981, 0,      0,     0,
                 0,      0.1,    0,     0,
                 0,      0,      0.1,   0,
                 0,      0,      0,     0.1,
                 0,      0, -0.0488869, 0,
                 0,   0.0488869, 0,     0,
                -1.962,  0,      0,     0;
  // clang-format on

  wd_expected.setZero();

  ASSERT_TRUE(expectEigenNear(Ad_expected, base.getAd(), 1e-6));
  ASSERT_TRUE(expectEigenNear(Bd_expected, base.getBd(), 1e-6));
  ASSERT_TRUE(expectEigenNear(wd_expected, base.getWd(), 1e-6));
}

TEST(MPCBaseTester, givenQandR_FormsQbigAndRbigCorrectly)
{
  const int n{2}, m{2}, T{3}, mu{3};
  const bool use_input_cost{true};
  MPCBaseTester base{m, n, T, mu, use_input_cost};
  Eigen::Vector2d Q{1, 2}, R{3, 4};
  base.setWeights(Q, R);

  Eigen::DiagonalMatrix<double, n * T> Qbig_expected;
  Qbig_expected.diagonal() << Q, Q, Q;
  Eigen::DiagonalMatrix<double, m * mu> Rbig_expected;
  Rbig_expected.diagonal() << R, R, R;


  ASSERT_TRUE(expectEigenNear(Qbig_expected.diagonal(),
                              base.getQbig().diagonal(), 1e-6));
  ASSERT_TRUE(expectEigenNear(Rbig_expected.diagonal(),
                              base.getRbig().diagonal(), 1e-6));
}

TEST(MPCBaseTester, askedToUpdateTrajectories_updatesCorrectly)
{
  const int n{2}, m{2}, T{3}, mu{3};
  const bool use_input_cost{true};
  MPCBaseTester base{n, m, T, mu, use_input_cost};

  Eigen::Vector2d x_des{1, 2}, u_des{3, 4};
  base.setReferenceState(x_des);
  base.setReferenceInput(u_des);

  Eigen::Matrix<double, n * T, 1> x_traj_expected;
  x_traj_expected << x_des, x_des, x_des;
  Eigen::Matrix<double, m * mu, 1> u_traj_expected;
  u_traj_expected << u_des, u_des, u_des;

  ASSERT_TRUE(
      expectEigenNear(x_traj_expected, base.getStateTrajectory(), 1e-6));
  ASSERT_TRUE(
      expectEigenNear(u_traj_expected, base.getInputTrajectory(), 1e-6));
}
