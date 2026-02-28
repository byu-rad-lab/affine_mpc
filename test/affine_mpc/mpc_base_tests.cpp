#include <Eigen/Core>
#include <gtest/gtest.h>
#include <unsupported/Eigen/Splines>

#include "affine_mpc/mpc_base.hpp"
#include "affine_mpc/parameterization.hpp"
#include "utils.hpp"

namespace ampc = affine_mpc;
using namespace Eigen;

// Tests use varying parameters for the MPCBase class, so a GTest fixture is not
// used
class MPCBaseTester : public ampc::MPCBase
{
public:
  MPCBaseTester(const int n,
                const int m,
                const ampc::Parameterization& param,
                const ampc::Options& opts = {}) :
      MPCBase{n, m, param, opts, m * param.num_control_points, 0}
  {}
  virtual ~MPCBaseTester() = default;
  void getPredictedStateTrajectory(
      Eigen::Ref<Eigen::VectorXd> x_traj) const noexcept override final
  {}
  const auto getSplineSegmentIdxs() const { return this->spline_segment_idxs_; }
  const auto getSplineKnots() const { return this->spline_knots_; }
  const auto getSplineWeights() const { return this->spline_weights_; }
  const auto getAd() { return Ad_; }
  const auto getBd() { return Bd_; }
  const auto getWd() { return wd_; }
  const auto getQbig() { return Q_big_; }
  const auto getRbig() { return R_big_; }
  const auto getStateTrajectory() { return x_goal_; }
  const auto getInputTrajectory() { return u_goal_; }

protected:
  void qpUpdateX0(const Eigen::Ref<const Eigen::VectorXd>& x0) override final {}
  bool qpUpdateModel() override final { return true; }
  bool qpUpdateReferences() override final { return true; }
  bool qpUpdateInputLimits() override final { return true; }
  bool qpUpdateStateLimits() override final { return true; }
  bool qpUpdateSlewRate() override final { return true; }
};

TEST(MPCBaseTester, givenParams_FormsSplineCorrectly)
{
  const int n{2}, m{1};                // not important for this test
  const int T{10}, n_ctrls{5}, deg{2}; // define expected behavior
  const ampc::Parameterization param{
      ampc::Parameterization::bspline(T, n_ctrls, deg)};
  MPCBaseTester msd_mpc{n, m, param};

  VectorXd knots{n_ctrls + deg + 1}, knots_expected{n_ctrls + deg + 1};
  knots = msd_mpc.getSplineKnots();
  knots_expected << 0, 0, 0, 3, 6, 9, 9, 9;
  ASSERT_TRUE(expectEigenNear(knots, knots_expected, 1e-15));

  VectorXi segment_idxs{T}, segment_idxs_expected{T};
  segment_idxs = msd_mpc.getSplineSegmentIdxs();
  segment_idxs_expected << 0, 0, 0, 1, 1, 1, 2, 2, 2, 2;
  ASSERT_TRUE(expectEigenNear(segment_idxs, segment_idxs_expected, 1e-15));

  VectorXd ctrls{n_ctrls};
  ctrls << 0, 1, 1, -1, 0;
  using Spline1d = Spline<double, 1>;
  Spline1d spline_test{knots_expected, ctrls};
  MatrixXd weights = msd_mpc.getSplineWeights();

  for (int k{0}; k < T; ++k) {
    // test weights
    const double t = k;
    const RowVectorXd weights_expected =
        Spline1d::BasisFunctions(t, deg, knots_expected);
    const RowVectorXd weights_i = weights.col(k);
    ASSERT_TRUE(expectEigenNear(weights_i, weights_expected, 1e-15));

    // test spline evaluation
    const int span = Spline1d::Span(t, deg, knots_expected);
    Ref<const VectorXd> active_ctrls = ctrls.segment(segment_idxs(k), deg + 1);
    double eval = weights_i * active_ctrls;
    double eval_expected = spline_test(t)(0);
    ASSERT_DOUBLE_EQ(eval, eval_expected);
  }
}


TEST(MPCBaseTester, givenContinuousLinearSystem_DiscretizesCorrectly)
{
  const int n{2}, m{1}, T{5}, nc{3}, deg{1};
  MPCBaseTester base{n, m, {T, nc, deg}};

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
  const int n{9}, m{4}, T{5}, nc{3}, deg{1};
  MPCBaseTester base{n, m, {T, nc, deg}};

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
  const int n{2}, m{2}, T{3}, nc{3}, deg{1};
  MPCBaseTester base{n, m, {T, nc, deg}, {.use_input_cost = true}};
  Eigen::Vector2d Q{1, 2}, R{3, 4};
  base.setWeights(Q, R);

  Eigen::DiagonalMatrix<double, n * T> Qbig_expected;
  Qbig_expected.diagonal() << Q, Q, Q;
  Eigen::DiagonalMatrix<double, m * nc> Rbig_expected;
  Rbig_expected.diagonal() << R, R, R;


  ASSERT_TRUE(expectEigenNear(Qbig_expected.diagonal(),
                              base.getQbig().diagonal(), 1e-6));
  ASSERT_TRUE(expectEigenNear(Rbig_expected.diagonal(),
                              base.getRbig().diagonal(), 1e-6));
}

TEST(MPCBaseTester, askedToUpdateTrajectories_updatesCorrectly)
{
  const int n{2}, m{2}, T{3}, nc{3}, deg{1};
  MPCBaseTester base{n, m, {T, nc, deg}, {.use_input_cost = true}};

  Eigen::Vector2d x_des{1, 2}, u_des{3, 4};
  base.setReferenceState(x_des);
  base.setReferenceInput(u_des);

  Eigen::Matrix<double, n * T, 1> x_traj_expected;
  x_traj_expected << x_des, x_des, x_des;
  Eigen::Matrix<double, m * nc, 1> u_traj_expected;
  u_traj_expected << u_des, u_des, u_des;

  ASSERT_TRUE(
      expectEigenNear(x_traj_expected, base.getStateTrajectory(), 1e-6));
  ASSERT_TRUE(
      expectEigenNear(u_traj_expected, base.getInputTrajectory(), 1e-6));
}

TEST(MPCBaseTester, givenStateWeights_FormsQbigWithTerminalCostCorrectly)
{
  const int n{2}, m{1}, T{4}, nc{3}, deg{1};
  MPCBaseTester base{n, m, {T, nc, deg}};

  Eigen::Vector2d Q{1.0, 2.0}, Qf{10.0, 20.0};

  base.setStateWeights(Q);

  // Qf = Q when not specified, so all steps should use Q
  Eigen::Matrix<double, n * T, 1> Q_big_diag_expected;
  Q_big_diag_expected << Q, Q, Q, Q;
  ASSERT_TRUE(
      expectEigenNear(Q_big_diag_expected, base.getQbig().diagonal(), 1e-15));

  // Intermediate steps use Q, final step uses Qf
  base.setStateWeights(Q, Qf);
  Q_big_diag_expected << Q, Q, Q, Qf;

  ASSERT_TRUE(
      expectEigenNear(Q_big_diag_expected, base.getQbig().diagonal(), 1e-15));
}

TEST(MPCBaseTester, givenInputWeights_FormsQbigWithTerminalCostCorrectly)
{
  const int n{2}, m{1}, T{4}, nc{3}, deg{1};
  MPCBaseTester base{n, m, {T, nc, deg}, {.use_input_cost = true}};

  Eigen::Vector<double, 1> R{3.0};

  Eigen::Matrix<double, 1 * nc, 1> R_big_diag_expected;

  base.setInputWeights(R);
  R_big_diag_expected << R, R, R;

  ASSERT_TRUE(
      expectEigenNear(R_big_diag_expected, base.getRbig().diagonal(), 1e-15));
}

TEST(MPCBaseTester, givenAllWeights_FormsQbigWithTerminalCostCorrectly)
{
  const int n{2}, m{1}, T{4}, nc{3}, deg{1};
  MPCBaseTester base{n, m, {T, nc, deg}, {.use_input_cost = true}};

  Eigen::Vector2d Q{1.0, 2.0}, Qf{10.0, 20.0};
  Eigen::Vector<double, 1> R{3.0};

  Eigen::Matrix<double, n * T, 1> Q_big_diag_expected;
  Eigen::Matrix<double, 1 * nc, 1> R_big_diag_expected;

  // Qf = Q when not specified, so all steps should use Q
  base.setWeights(Q, R);
  Q_big_diag_expected << Q, Q, Q, Q;
  R_big_diag_expected << R, R, R;

  ASSERT_TRUE(
      expectEigenNear(Q_big_diag_expected, base.getQbig().diagonal(), 1e-15));
  ASSERT_TRUE(
      expectEigenNear(R_big_diag_expected, base.getRbig().diagonal(), 1e-15));

  // Specify Qf
  base.setWeights(Q, Qf, R);
  Q_big_diag_expected << Q, Q, Q, Qf;
  R_big_diag_expected << R, R, R;

  ASSERT_TRUE(
      expectEigenNear(Q_big_diag_expected, base.getQbig().diagonal(), 1e-15));
  ASSERT_TRUE(
      expectEigenNear(R_big_diag_expected, base.getRbig().diagonal(), 1e-15));
}
