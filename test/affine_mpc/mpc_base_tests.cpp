#include <Eigen/Core>
#include <gtest/gtest.h>
#include <unsupported/Eigen/Splines>

#include "affine_mpc/mpc_base.hpp"
#include "affine_mpc/options.hpp"
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
  const auto getStateTrajectory() { return x_ref_; }
  const auto getInputTrajectory() { return u_ref_; }
  const auto getASlew() { return A_.middleRows(slew0_idx_, totalSlewDim()); }
  const auto getLSlew() { return l_.segment(slew0_idx_, totalSlewDim()); }
  const auto getUSlew() { return u_.segment(slew0_idx_, totalSlewDim()); }
  const auto getASatU() { return A_.middleRows(u_sat_idx_, u_sat_dim_); }
  const auto getLSatU() { return l_.segment(u_sat_idx_, u_sat_dim_); }
  const auto getUSatU() { return u_.segment(u_sat_idx_, u_sat_dim_); }

protected:
  void qpUpdateX0(const Eigen::Ref<const Eigen::VectorXd>& x0) override final {}
  bool qpUpdateModel() override final { return true; }
  bool qpUpdateReferences() override final { return true; }
  bool qpUpdateInputLimits() override final { return true; }
  bool qpUpdateStateLimits() override final { return true; }
  bool qpUpdateSlewRate() override final { return true; }

  int totalSlewDim() const
  {
    return slew_dim_ + input_dim_ * opts_.slew_initial_input;
  }
};

TEST(MPCBaseTester, givenParams_FormsSplineCorrectly)
{
  const int n{2}, m{1};                // not important for this test
  const int T{10}, n_ctrls{5}, deg{2}; // define expected behavior
  const ampc::Parameterization param{
      ampc::Parameterization::bspline(T, deg, n_ctrls)};
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
  MPCBaseTester base{n, m, {T, deg, nc}};

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
  MPCBaseTester base{n, m, {T, deg, nc}};

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
  MPCBaseTester base{n, m, {T, deg, nc}, {.use_input_cost = true}};
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
  MPCBaseTester base{n, m, {T, deg, nc}, {.use_input_cost = true}};

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
  MPCBaseTester base{n, m, {T, deg, nc}};

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
  MPCBaseTester base{n, m, {T, deg, nc}, {.use_input_cost = true}};

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
  MPCBaseTester base{n, m, {T, deg, nc}, {.use_input_cost = true}};

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

TEST(MPCBaseTester, givenInvalidStateDim_Throws)
{
  auto testInvalidStateDim = [](int invalid_n) {
    const int m{1}, T{4}, nc{3}, deg{1};
    expectInvalidArgumentWithMessage(
        [&]() { MPCBaseTester base(invalid_n, m, {T, deg, nc}); },
        "state_dim must be positive");
  };
  testInvalidStateDim(0);
  testInvalidStateDim(-1);
}

TEST(MPCBaseTester, givenInvalidInputDim_Throws)
{
  auto testInvalidInputDim = [](int invalid_m) {
    const int n{2}, T{4}, nc{3}, deg{1};
    expectInvalidArgumentWithMessage(
        [&]() { MPCBaseTester base(n, invalid_m, {T, deg, nc}); },
        "input_dim must be positive");
  };
  testInvalidInputDim(0);
  testInvalidInputDim(-1);
}

TEST(MPCBaseTester, inputCostDisabledButUsed_Throws)
{
  const int n{2}, m{1}, T{4}, nc{1}, deg{0};
  MPCBaseTester base{n, m, {T, deg, nc}};

  Eigen::Vector<double, 1> R{1.0};
  expectLogicErrorWithMessage([&base, &R]() { base.setInputWeights(R); },
                              "Input cost is not enabled");
  expectLogicErrorWithMessage([&base, &R]() { base.setReferenceInput(R); },
                              "Input cost is not enabled");
  expectLogicErrorWithMessage(
      [&base, &R]() { base.setReferenceParameterizedInputTrajectory(R); },
      "Input cost is not enabled");
}

TEST(MPCBaseTester, initialSlewRateDisabledButUsed_Throws)
{
  const int n{2}, m{1}, T{4}, nc{3}, deg{1};
  MPCBaseTester base{n, m, {T, deg, nc}};

  Eigen::Vector<double, 1> u_slew{0.5};
  expectLogicErrorWithMessage(
      [&base, &u_slew]() { base.setSlewRateInitial(u_slew); },
      "Initial slew rate is not enabled");
  expectLogicErrorWithMessage(
      [&base, &u_slew]() { base.setPreviousInput(u_slew); },
      "Initial slew rate is not enabled");
}

TEST(MPCBaseTester, slewRateDisabledButUsed_Throws)
{
  const int n{2}, m{1}, T{4}, nc{3}, deg{1};
  MPCBaseTester base{n, m, {T, deg, nc}};

  Eigen::Vector<double, 1> u_slew{0.5};
  expectLogicErrorWithMessage([&base, &u_slew]() { base.setSlewRate(u_slew); },
                              "Slew rate is not enabled");
}

TEST(MPCBaseTester, stateSaturationDisabledButUsed_Throws)
{
  const int n{2}, m{1}, T{4}, nc{3}, deg{1};
  MPCBaseTester base{n, m, {T, deg, nc}};

  Eigen::Vector<double, 2> x_min{-1, -1}, x_max{1, 1};
  expectLogicErrorWithMessage(
      [&base, &x_min, &x_max]() { base.setStateLimits(x_min, x_max); },
      "State saturation is not enabled");
}

TEST(MPCBaseTester, givenNegativeSlewRate_Throws)
{
  const int n{2}, m{1}, T{4}, nc{3}, deg{1};
  MPCBaseTester base{n, m, {T, deg, nc}, {.slew_control_points = true}};

  Eigen::Vector<double, 1> u_slew{-0.5};
  expectInvalidArgumentWithMessage(
      [&base, &u_slew]() { base.setSlewRate(u_slew); },
      "Slew rate must be non-negative");
}

TEST(MPCBaseTester, givenNegativeInitialSlewRate_Throws)
{
  const int n{2}, m{1}, T{4}, nc{3}, deg{1};
  MPCBaseTester base{n, m, {T, deg, nc}, {.slew_initial_input = true}};

  Eigen::Vector<double, 1> u_slew{-0.5};
  expectInvalidArgumentWithMessage(
      [&base, &u_slew]() { base.setSlewRateInitial(u_slew); },
      "Slew rate must be non-negative");
}

TEST(MPCBaseTester, givenInvalidInputLimits_Throws)
{
  const int n{2}, m{1}, T{4}, nc{3}, deg{1};
  MPCBaseTester base{n, m, {T, deg, nc}, {.saturate_states = true}};

  Eigen::Vector<double, 1> u_min{1}, u_max{0};
  expectInvalidArgumentWithMessage(
      [&base, &u_min, &u_max]() { base.setInputLimits(u_min, u_max); },
      "u_min cannot be greater than u_max");
}

TEST(MPCBaseTester, givenInvalidStateLimits_Throws)
{
  const int n{2}, m{1}, T{4}, nc{3}, deg{1};
  MPCBaseTester base{n, m, {T, deg, nc}, {.saturate_states = true}};

  Eigen::Vector<double, 2> x_min{1, 1}, x_max{-1, -1};
  expectInvalidArgumentWithMessage(
      [&base, &x_min, &x_max]() { base.setStateLimits(x_min, x_max); },
      "x_min cannot be greater than x_max");
}

TEST(MPCBaseTester, givenInvalidStateWeights_Throws)
{
  const int n{2}, m{1}, T{4}, nc{3}, deg{1};
  MPCBaseTester base{n, m, {T, deg, nc}};

  Eigen::Vector2d Q{-1, 1}, Q2{1, 1};
  expectInvalidArgumentWithMessage([&base, &Q]() { base.setStateWeights(Q); },
                                   "State weights must be non-negative");
  // same test but on Qf
  expectInvalidArgumentWithMessage(
      [&base, &Q, &Q2]() { base.setStateWeights(Q2, Q); },
      "State weights must be non-negative");
}

TEST(MPCBaseTester, givenInvalidInputWeights_Throws)
{
  const int n{2}, m{1}, T{4}, nc{3}, deg{1};
  MPCBaseTester base{n, m, {T, deg, nc}, {.use_input_cost = true}};

  Eigen::Vector<double, 1> R{-1};
  expectInvalidArgumentWithMessage([&base, &R]() { base.setInputWeights(R); },
                                   "Input weights must be non-negative");
}

TEST(MPCBaseTester,
     givenControlPointSatOption_FormsInputSaturationConstraintsCorrectly)
{

  auto test = [&](bool sat_u_traj) {
    const int n{2}, m{1}, T{4}, nc{3}, deg{1};
    const ampc::Options opts{.saturate_input_trajectory = sat_u_traj};
    MPCBaseTester base{n, m, {T, deg, nc}, opts};

    Eigen::Vector<double, 1> u_min{-1}, u_max{2};
    base.setInputLimits(u_min, u_max);

    const MatrixXd A = base.getASatU();
    const VectorXd l = base.getLSatU();
    const VectorXd u = base.getUSatU();

    ASSERT_EQ(A.rows(), m * nc);
    ASSERT_EQ(l.size(), m * nc);
    ASSERT_EQ(u.size(), m * nc);

    ASSERT_TRUE(A.isIdentity());

    VectorXd l_expected = u_min.replicate(nc, 1);
    VectorXd u_expected = u_max.replicate(nc, 1);
    ASSERT_TRUE(expectEigenNear(l, l_expected, 1e-15));
    ASSERT_TRUE(expectEigenNear(u, u_expected, 1e-15));
  };

  test(false);
  // when asked to saturate input traj, but deg < 2, then it should still only
  // saturate control points, not the whole trajectory
  test(true);
}

TEST(MPCBaseTester,
     givenInputTrajSatOptionSimpleCase_FormsInputSaturationConstraintsCorrectly)
{
  const int n{2}, m{1}, T{5}, nc{3}, deg{2};
  MPCBaseTester base{n, m, {T, deg, nc}, {.saturate_input_trajectory = true}};

  Eigen::Vector<double, 1> u_min{-1}, u_max{2};
  base.setInputLimits(u_min, u_max);

  const MatrixXd A = base.getASatU();
  const VectorXd l = base.getLSatU();
  const VectorXd u = base.getUSatU();

  ASSERT_EQ(A.rows(), m * T);
  ASSERT_EQ(l.size(), m * T);
  ASSERT_EQ(u.size(), m * T);

  // this only works for m = 1 and nc = deg+1, otherwise each weight is
  // multiplied by I(m,m) or 0(m,m)
  const MatrixXd A_expected = base.getSplineWeights().transpose();
  ASSERT_TRUE(expectEigenNear(A, A_expected, 1e-15));

  VectorXd l_expected = u_min.replicate(T, 1);
  VectorXd u_expected = u_max.replicate(T, 1);
  ASSERT_TRUE(expectEigenNear(l, l_expected, 1e-15));
  ASSERT_TRUE(expectEigenNear(u, u_expected, 1e-15));
}

TEST(MPCBaseTester,
     givenInputTrajSatOptionMIMO_FormsInputSaturationConstraintsCorrectly)
{
  const int n{4}, m{2}, T{10}, nc{5}, deg{2};
  MPCBaseTester base{n, m, {T, deg, nc}, {.saturate_input_trajectory = true}};

  Eigen::Vector<double, m> u_min{-1, -0.1}, u_max{2, 1};
  base.setInputLimits(u_min, u_max);

  const MatrixXd A = base.getASatU();
  const VectorXd l = base.getLSatU();
  const VectorXd u = base.getUSatU();

  ASSERT_EQ(A.rows(), m * T);
  ASSERT_EQ(l.size(), m * T);
  ASSERT_EQ(u.size(), m * T);

  const MatrixXd c = base.getSplineWeights();
  const MatrixXd I = MatrixXd::Identity(m, m);

  MatrixXd A_expected = MatrixXd::Zero(A.rows(), A.cols());
  // clang-format off
  A_expected << c(0,0)*I, c(1,0)*I, c(2,0)*I,      0*I,      0*I,
                c(0,1)*I, c(1,1)*I, c(2,1)*I,      0*I,      0*I,
                c(0,2)*I, c(1,2)*I, c(2,2)*I,      0*I,      0*I,
                     0*I, c(0,3)*I, c(1,3)*I, c(2,3)*I,      0*I,
                     0*I, c(0,4)*I, c(1,4)*I, c(2,4)*I,      0*I,
                     0*I, c(0,5)*I, c(1,5)*I, c(2,5)*I,      0*I,
                     0*I,      0*I, c(0,6)*I, c(1,6)*I, c(2,6)*I,
                     0*I,      0*I, c(0,7)*I, c(1,7)*I, c(2,7)*I,
                     0*I,      0*I, c(0,8)*I, c(1,8)*I, c(2,8)*I,
                     0*I,      0*I, c(0,9)*I, c(1,9)*I, c(2,9)*I;
  // clang-format on
  ASSERT_TRUE(expectEigenNear(A, A_expected, 1e-15));

  VectorXd l_expected = u_min.replicate(T, 1);
  VectorXd u_expected = u_max.replicate(T, 1);
  ASSERT_TRUE(expectEigenNear(l, l_expected, 1e-15));
  ASSERT_TRUE(expectEigenNear(u, u_expected, 1e-15));
}

TEST(MPCBaseTester, givenInitialSlewRate_FormsSlewRateConstraintsCorrectly)
{
  const int n{2}, m{1}, T{4}, nc{3}, deg{1};
  MPCBaseTester base{n, m, {T, deg, nc}, {.slew_initial_input = true}};

  Eigen::Vector<double, 1> u_slew{0.5}, u_prev{0.1};
  base.setSlewRateInitial(u_slew);
  base.setPreviousInput(u_prev);

  const MatrixXd A = base.getASlew();
  const VectorXd l = base.getLSlew();
  const VectorXd u = base.getUSlew();

  ASSERT_EQ(A.rows(), m);
  ASSERT_EQ(l.size(), m);
  ASSERT_EQ(u.size(), m);

  ASSERT_TRUE(A.isIdentity());

  VectorXd l_expected = u_prev - u_slew;
  VectorXd u_expected = u_prev + u_slew;

  ASSERT_TRUE(expectEigenNear(l, l_expected, 1e-15));
  ASSERT_TRUE(expectEigenNear(u, u_expected, 1e-15));
}

TEST(MPCBaseTester, givenSlewRate_FormsSlewRateConstraintsCorrectly)
{
  const int n{2}, m{1}, T{4}, nc{3}, deg{1};
  MPCBaseTester base{n, m, {T, deg, nc}, {.slew_control_points = true}};

  Eigen::Vector<double, 1> u_slew{0.5};
  base.setSlewRate(u_slew);

  const MatrixXd A = base.getASlew();
  const VectorXd l = base.getLSlew();
  const VectorXd u = base.getUSlew();

  ASSERT_EQ(A.rows(), m * (nc - 1));
  ASSERT_EQ(l.size(), m * (nc - 1));
  ASSERT_EQ(u.size(), m * (nc - 1));

  MatrixXd A_expected = MatrixXd::Zero(A.rows(), A.cols());
  A_expected.diagonal().setConstant(-1.0);
  A_expected.diagonal(m).setConstant(1.0);
  ASSERT_TRUE(expectEigenNear(A, A_expected, 1e-15));

  VectorXd u_expected = u_slew.replicate(m * (nc - 1), 1);
  VectorXd l_expected = -u_expected;

  ASSERT_TRUE(expectEigenNear(l, l_expected, 1e-15));
  ASSERT_TRUE(expectEigenNear(u, u_expected, 1e-15));
}

TEST(MPCBaseTester, givenBothSlewRates_FormsSlewRateConstraintsCorrectly)
{
  const int n{2}, m{1}, T{4}, nc{3}, deg{1};
  affine_mpc::Options opts{.slew_initial_input = true,
                           .slew_control_points = true};
  MPCBaseTester base{n, m, {T, deg, nc}, opts};

  Eigen::Vector<double, 1> u_slew{0.5}, u_prev{0.1};
  base.setSlewRate(u_slew);
  base.setSlewRateInitial(u_slew);
  base.setPreviousInput(u_prev);

  const MatrixXd A = base.getASlew();
  const VectorXd l = base.getLSlew();
  const VectorXd u = base.getUSlew();

  ASSERT_EQ(A.rows(), m * nc);
  ASSERT_EQ(l.size(), m * nc);
  ASSERT_EQ(u.size(), m * nc);

  MatrixXd A_expected = MatrixXd::Zero(A.rows(), A.cols());
  A_expected.diagonal().setConstant(1.0);
  A_expected.diagonal(-m).setConstant(-1.0);
  ASSERT_TRUE(expectEigenNear(A, A_expected, 1e-15));

  VectorXd u_expected{m * nc}, l_expected{m * nc};
  u_expected << u_prev + u_slew, u_slew, u_slew;
  l_expected << u_prev - u_slew, -u_slew, -u_slew;

  ASSERT_TRUE(expectEigenNear(l, l_expected, 1e-15));
  ASSERT_TRUE(expectEigenNear(u, u_expected, 1e-15));
}

TEST(MPCBaseTester, givenModel_PropagatesModelCorrectly)
{
  const int n{2}, m{1}, T{4}, nc{3}, deg{1};
  MPCBaseTester base{n, m, {T, deg, nc}};

  Eigen::Matrix2d A;
  Eigen::Vector2d B, w;
  A << 0, 1, -0.6, -0.1;
  B << 0, 0.2;
  w << 0.01, 0.02;
  base.setModelContinuous2Discrete(A, B, w, 0.1);

  const Eigen::Vector2d x0{1.0, -0.5};
  const Eigen::Vector<double, 1> u{2.0};
  Eigen::Vector2d x_next;
  base.propagateModel(x0, u, x_next);

  const Eigen::Vector2d x_next_expected =
      base.getAd() * x0 + base.getBd() * u + base.getWd();
  ASSERT_TRUE(expectEigenNear(x_next, x_next_expected, 1e-12));
}

TEST(MPCBaseTester, givenModel_PropagateModelAllowsAliasing)
{
  // x_next = x0 is a common pattern in simulation loops
  const int n{2}, m{1}, T{4}, nc{3}, deg{1};
  MPCBaseTester base{n, m, {T, deg, nc}};

  Eigen::Matrix2d A;
  Eigen::Vector2d B, w;
  A << 0, 1, -0.6, -0.1;
  B << 0, 0.2;
  w.setZero();
  base.setModelContinuous2Discrete(A, B, w, 0.1);

  Eigen::Vector2d x{1.0, -0.5};
  const Eigen::Vector<double, 1> u{2.0};
  const Eigen::Vector2d x_expected =
      base.getAd() * x + base.getBd() * u + base.getWd();

  base.propagateModel(x, u, x); // aliased: x_next == x0
  ASSERT_TRUE(expectEigenNear(x, x_expected, 1e-12));
}

TEST(MPCBaseTester, givenParameterizedInputTrajectory_SetsUgoalCorrectly)
{
  const int n{2}, m{2}, T{3}, nc{3}, deg{1};
  MPCBaseTester base{n, m, {T, deg, nc}, {.use_input_cost = true}};

  VectorXd u_ctrl_pts{m * nc};
  u_ctrl_pts << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0;
  base.setReferenceParameterizedInputTrajectory(u_ctrl_pts);

  // u_ref_ should be set verbatim (no replication, unlike setReferenceInput)
  ASSERT_TRUE(expectEigenNear(base.getInputTrajectory(), u_ctrl_pts, 1e-15));
}

TEST(MPCBaseTester,
     setReferenceInputAndSetReferenceParameterizedInput_ProduceDifferentUref)
{
  // setReferenceInput replicates a single step; setReferenceParameterizedInput
  // sets the full control-point vector directly. Verify they differ for a
  // non-trivial target.
  const int n{2}, m{1}, T{4}, nc{3}, deg{1};
  MPCBaseTester base{n, m, {T, deg, nc}, {.use_input_cost = true}};

  VectorXd u_step{m};
  u_step << 2.0;
  base.setReferenceInput(u_step);
  VectorXd u_ref_replicated = base.getInputTrajectory(); // [2, 2, 2]

  VectorXd u_ctrl_pts{m * nc};
  u_ctrl_pts << 1.0, 2.0, 3.0; // non-uniform
  base.setReferenceParameterizedInputTrajectory(u_ctrl_pts);
  VectorXd u_ref_direct = base.getInputTrajectory(); // [1, 2, 3]

  // They should differ
  ASSERT_FALSE(expectEigenNear(u_ref_replicated, u_ref_direct, 1e-10));
  ASSERT_TRUE(expectEigenNear(u_ref_direct, u_ctrl_pts, 1e-15));
}
