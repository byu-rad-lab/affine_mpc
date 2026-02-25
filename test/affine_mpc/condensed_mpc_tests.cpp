#include "affine_mpc/condensed_mpc.hpp"

#include <gtest/gtest.h>
#include <unsupported/Eigen/Splines>

#include "affine_mpc/mpc_logger.hpp"
#include "affine_mpc/parameterization.hpp"
#include "utils.hpp"

using namespace Eigen;
namespace ampc = affine_mpc;


// Tests are done with a Mass-Spring-Damper system (2 states, 1 input)
class CondensedMPCProtectedTester : public ampc::CondensedMPC
{
public:
  using CondensedMPC::CondensedMPC;
  virtual ~CondensedMPCProtectedTester() = default;

  void setModel()
  {
    Eigen::Matrix2d A;
    Eigen::Vector2d B, w;
    A << 0, 1, -0.6, -0.1;
    B << 0, 0.2;
    w.setZero();
    const double ts{0.1};
    setModelContinuous2Discrete(A, B, w, ts);
    // Matrix2d Ad;
    // Vector2d Bd, wd;
    // Ad << 0.99701147, 0.09940219, -0.05964131, 0.98707125;
    // Bd << 0.00099618, 0.01988044;
    // wd << 0.01193257, 0.138468;
    // setModelDiscrete(Ad, Bd, wd);
  }

  void findSAndV(const Eigen::Ref<const Eigen::VectorXd>& x0)
  {
    updateS();
    updateV(x0);
  }

  void findPAndQ(const Eigen::Ref<const Eigen::VectorXd>& x0)
  {
    qpUpdateX0(x0);
  }

  const auto getSplineSegmentIdxs() const { return this->spline_segment_idxs_; }
  const auto getSplineKnots() const { return this->spline_knots_; }
  const auto getSplineWeights() const { return this->spline_weights_; }
  const auto getAd() const { return this->Ad_; }
  const auto getBd() const { return this->Bd_; }
  const auto getWd() const { return this->wd_; }
  const auto getQbig() const { return this->Q_big_; }
  const auto getRbig() const { return this->R_big_; }
  const auto getS() const { return this->S_; }
  const auto getV() const { return this->v_; }
  const auto getP() const { return this->P_; }
  const auto getQ() const { return this->q_; }
};

TEST(CondensedMPCProtectedTester, givenParams_FormsSplineCorrectly)
{
  const int n{2}, m{1};                // not important for this test
  const int T{10}, n_ctrls{5}, deg{2}; // define expected behavior
  const ampc::Parameterization param{
      ampc::Parameterization::bspline(T, n_ctrls, deg)};
  CondensedMPCProtectedTester msd_mpc{n, m, param};

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

TEST(CondensedMPCProtectedTester, givenModel_FormsSandVcorrectly)
{
  const int n{2}, m{1}, T{5}, nc{3};
  CondensedMPCProtectedTester msd_mpc{
      n, m, ampc::Parameterization::linearInterp(T, nc)};
  msd_mpc.setModel();

  Eigen::Vector2d x0{0, 0.1};
  msd_mpc.findSAndV(x0);

  Eigen::Matrix2d A{msd_mpc.getAd()};
  Eigen::Vector2d B{msd_mpc.getBd()};
  Eigen::Vector2d w{msd_mpc.getWd()};
  Eigen::Matrix2d I{Eigen::Matrix2d::Identity()};
  Eigen::Vector2d zero{Eigen::Vector2d::Zero()};
  Eigen::Matrix<double, n * T, m * nc> S_test;
  // clang-format off
  S_test << B,                     zero,                    zero,
           (A+0.5*I)*B,            0.5*B,                   zero,
           (A*A+0.5*A)*B,         (0.5*A+I)*B,              zero,
           (A*A*A+0.5*A*A)*B,     (0.5*A*A+A+0.5*I)*B,      0.5*B,
           (A*A*A*A+0.5*A*A*A)*B, (0.5*A*A*A+A*A+0.5*A)*B, (0.5*A+I)*B;
  // clang-format on

  Eigen::Matrix<double, n * T, 1> v_test;
  // clang-format off
  v_test << A*x0 + w,
            A*A*x0 + A*w + w,
            A*A*A*x0 + A*A*w + A*w + w,
            A*A*A*A*x0 + A*A*A*w + A*A*w + A*w + w,
            A*A*A*A*A*x0 + A*A*A*A*w + A*A*A*w + A*A*w + A*w + w;
  // clang-format on

  ASSERT_TRUE(expectEigenNear(msd_mpc.getS(), S_test, 1e-6));
  ASSERT_TRUE(expectEigenNear(msd_mpc.getV(), v_test, 1e-6));
}

TEST(CondensedMPCProtectedTester, givenModel_FormsPandQcorrectly)
{
  const int n{2}, m{1}, T{5}, nc{3};
  CondensedMPCProtectedTester msd_mpc{
      n, m, ampc::Parameterization::linearInterp(T, nc),
      ampc::Options{.use_input_cost = true}};
  msd_mpc.setModel();

  Eigen::Matrix<double, n, 1> Q{1, 1};
  Eigen::Matrix<double, m, 1> R{1}, u_goal{1};
  Eigen::Vector2d x0{0, 0.1};
  Eigen::Matrix<double, n * T, 1> x_goal;
  x_goal.setOnes();

  msd_mpc.setWeights(Q, R);
  msd_mpc.setReferenceStateTrajectory(x_goal);
  msd_mpc.setReferenceInput(u_goal);
  msd_mpc.findPAndQ(x0);

  Eigen::Matrix2d A{msd_mpc.getAd()};
  Eigen::Vector2d B{msd_mpc.getBd()};
  Eigen::Vector2d w{msd_mpc.getWd()};
  Eigen::Matrix2d I{Eigen::Matrix2d::Identity()};
  Eigen::Vector2d zero{Eigen::Vector2d::Zero()};
  Eigen::Matrix<double, n * T, m * nc> S;
  // clang-format off
  S << B,                     zero,                    zero,
      (A+0.5*I)*B,            0.5*B,                   zero,
      (A*A+0.5*A)*B,         (0.5*A+I)*B,              zero,
      (A*A*A+0.5*A*A)*B,     (0.5*A*A+A+0.5*I)*B,      0.5*B,
      (A*A*A*A+0.5*A*A*A)*B, (0.5*A*A*A+A*A+0.5*A)*B, (0.5*A+I)*B;
  // clang-format on

  Eigen::Matrix<double, n * T, 1> v;
  // clang-format off
  v << A*x0 + w,
       A*A*x0 + A*w + w,
       A*A*A*x0 + A*A*w + A*w + w,
       A*A*A*A*x0 + A*A*A*w + A*A*w + A*w + w,
       A*A*A*A*A*x0 + A*A*A*A*w + A*A*A*w + A*A*w + A*w + w;
  // clang-format on

  Eigen::Matrix<double, m * nc, m * nc> P_test;
  P_test = msd_mpc.getRbig();
  P_test += S.transpose() * S;

  Eigen::Matrix<double, m * nc, 1> q_test;
  q_test.setConstant(-1);
  q_test += S.transpose() * (v - x_goal);

  ASSERT_TRUE(expectEigenNear(msd_mpc.getP(), P_test, 1e-6));
  ASSERT_TRUE(expectEigenNear(msd_mpc.getQ(), q_test, 1e-6));
}

TEST(CondensedMPCProtectedTester, initializedAndAskedToSolve_SolvesCorrecly)
{
  const int n{2}, m{1}, T{10}, nc{10};
  CondensedMPCProtectedTester msd_mpc{
      n, m, ampc::Parameterization::linearInterp(T, nc),
      ampc::Options{.use_input_cost = true, .slew_control_points = true}};
  ampc::MPCLogger logger{&msd_mpc, "~/tmp/mpc_data"};
  msd_mpc.setModel();

  Eigen::Matrix<double, n, 1> Q{1, 0.11};
  Eigen::Matrix<double, m, 1> R{.0001};
  msd_mpc.setWeights(Q, R);
  Eigen::Vector2d x0{1, 1}, x_goal{1, 0};
  msd_mpc.setReferenceState(x_goal);
  Eigen::Matrix<double, m, 1> u_goal{0}, u_min{0}, u_max{3}, slew{1};
  ASSERT_TRUE(msd_mpc.setInputLimits(u_min, u_max));
  msd_mpc.setReferenceInput(u_goal);
  ASSERT_TRUE(msd_mpc.setSlewRate(slew));

  // These are the recommended settings, but explicitly set here for clarity
  auto settings{ampc::OSQPSolver::getDefaultSettings()};
  settings.alpha = 1.0;
  settings.verbose = false;
  settings.eps_abs = 1e-6;
  settings.eps_rel = 1e-6;
  ASSERT_TRUE(msd_mpc.initializeSolver(settings));

  x0.setZero();
  Eigen::Matrix<double, m, 1> u_star, u_star_expected;
  bool solved;
  solved = msd_mpc.solve(x0);
  msd_mpc.getNextInput(u_star);
  logger.logPreviousSolve(0, 0.1, x0);

  u_star_expected << 3.0;

  ASSERT_TRUE(expectEigenNear(u_star, u_star_expected, 1e-5));

  Eigen::Matrix<double, m * nc, 1> u_traj;
  solved = msd_mpc.solve(x0);
  msd_mpc.getParameterizedInputTrajectory(u_traj);
  logger.logPreviousSolve(0, 0.1, x0);
  int slew_errors{0};
  for (int i{0}; i < nc - 1; ++i) {
    slew_errors += abs(u_traj(i + 1) - u_traj(i)) > slew(0) + 1e-6;
  }

  ASSERT_EQ(slew_errors, 0);
}
