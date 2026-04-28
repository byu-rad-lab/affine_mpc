#include <Eigen/Core>
#include <gtest/gtest.h>
#include <unsupported/Eigen/Splines>

#include "affine_mpc/condensed_mpc.hpp"
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

  auto settings{ampc::OSQPSolver::getDefaultSettings()};
  settings.alpha = 1.0;
  settings.verbose = false;
  settings.eps_abs = 1e-6;
  settings.eps_rel = 1e-6;
  ASSERT_TRUE(msd_mpc.initializeSolver(settings));

  x0.setZero();
  Eigen::Matrix<double, m, 1> u_star, u_star_expected;
  ampc::SolveStatus status;
  status = msd_mpc.solve(x0);
  ASSERT_EQ(status, ampc::SolveStatus::Success);

  msd_mpc.getNextInput(u_star);

  u_star_expected << 3.0;
  ASSERT_TRUE(expectEigenNear(u_star, u_star_expected, 1e-5));

  Eigen::Matrix<double, m * nc, 1> u_traj;
  status = msd_mpc.solve(x0);
  ASSERT_EQ(status, ampc::SolveStatus::Success);

  msd_mpc.getInputControlPoints(u_traj);
  int slew_errors{0};
  for (int i{0}; i < nc - 1; ++i) {
    slew_errors += abs(u_traj(i + 1) - u_traj(i)) > slew(0) + 1e-6;
  }

  ASSERT_EQ(slew_errors, 0);
}

TEST(CondensedMPCProtectedTester,
     givenStateSaturation_RespectsStateBoundsInSolve)
{
  const int n{2}, m{1}, T{100}, nc{10};
  CondensedMPCProtectedTester msd_mpc{
      n, m, ampc::Parameterization::linearInterp(T, nc),
      ampc::Options{.use_input_cost = true, .saturate_states = true}};
  msd_mpc.setModel();

  msd_mpc.setWeights(Vector2d{1.0, 0.1}, VectorXd::Constant(m, 1e-4));
  msd_mpc.setReferenceState(Vector2d{1.0, 0.0});
  msd_mpc.setInputLimits(VectorXd::Constant(m, -3.0),
                         VectorXd::Constant(m, 3.0));

  // Tight state bounds that will be active
  const Vector2d x_min{-0.2, -2.0}, x_max{0.5, 2.0};
  msd_mpc.setStateLimits(x_min, x_max);

  auto settings{ampc::OSQPSolver::getRecommendedSettings(true)};
  // loosened tolerances to speed up test (else hits max iters)
  settings.eps_abs = 1e-4;
  settings.eps_rel = 1e-4;
  settings.adaptive_rho = true;
  ASSERT_TRUE(msd_mpc.initializeSolver(settings));
  ASSERT_EQ(msd_mpc.solve(Vector2d{0.0, 0.0}), ampc::SolveStatus::Success);

  Eigen::Matrix<double, n * T, 1> x_traj;
  msd_mpc.getPredictedStateTrajectory(x_traj);

  int bound_violations{0};
  for (int k{0}; k < T; ++k) {
    const Vector2d xk = x_traj.segment(n * k, n);
    bound_violations += (xk.array() > x_max.array() + 1e-3).any();
    bound_violations += (xk.array() < x_min.array() - 1e-3).any();
  }
  ASSERT_EQ(bound_violations, 0);
}

TEST(CondensedMPCProtectedTester,
     givenWeightUpdateBetweenSolves_ProducesCorrectSolution)
{
  const int n{2}, m{1}, T{10}, nc{10};
  CondensedMPCProtectedTester msd_mpc{
      n, m, ampc::Parameterization::linearInterp(T, nc),
      ampc::Options{.use_input_cost = true}};
  msd_mpc.setModel();

  msd_mpc.setWeights(Vector2d{1.0, 0.1}, VectorXd::Constant(m, 1e-4));
  msd_mpc.setReferenceState(Vector2d{1.0, 0.0});
  msd_mpc.setInputLimits(VectorXd::Constant(m, 0.0),
                         VectorXd::Constant(m, 3.0));
  msd_mpc.setReferenceInput(VectorXd::Zero(m));

  auto settings{ampc::OSQPSolver::getRecommendedSettings(true)};
  ASSERT_TRUE(msd_mpc.initializeSolver(settings));

  const Vector2d x0{0.0, 0.0};
  ASSERT_EQ(msd_mpc.solve(x0), ampc::SolveStatus::Success);
  Eigen::Matrix<double, m, 1> u_first;
  msd_mpc.getNextInput(u_first);

  // Heavy input penalty: solution should move away from saturation
  msd_mpc.setWeights(Vector2d{1.0, 0.1}, VectorXd::Constant(m, 1.0));
  ASSERT_EQ(msd_mpc.solve(x0), ampc::SolveStatus::Success);
  Eigen::Matrix<double, m, 1> u_second;
  msd_mpc.getNextInput(u_second);

  // Higher R must reduce the optimal input magnitude
  ASSERT_LT(u_second(0), u_first(0) - 1e-3);
}

TEST(CondensedMPCProtectedTester,
     givenReferenceUpdateBetweenSolves_ProducesCorrectSolution)
{
  const int n{2}, m{1}, T{10}, nc{10};
  CondensedMPCProtectedTester msd_mpc{
      n, m, ampc::Parameterization::linearInterp(T, nc),
      ampc::Options{.use_input_cost = true}};
  msd_mpc.setModel();

  msd_mpc.setWeights(Vector2d{1.0, 0.1}, VectorXd::Constant(m, 1e-4));
  msd_mpc.setReferenceState(Vector2d{1.0, 0.0});
  msd_mpc.setInputLimits(VectorXd::Constant(m, -3.0),
                         VectorXd::Constant(m, 3.0));
  msd_mpc.setReferenceInput(VectorXd::Zero(m));

  auto settings{ampc::OSQPSolver::getRecommendedSettings(true)};
  ASSERT_TRUE(msd_mpc.initializeSolver(settings));

  const Vector2d x0{0.0, 0.0};
  ASSERT_EQ(msd_mpc.solve(x0), ampc::SolveStatus::Success);
  Eigen::Matrix<double, m, 1> u_pos;
  msd_mpc.getNextInput(u_pos);

  // Flip the goal to the other side: optimal input should flip sign
  msd_mpc.setReferenceState(Vector2d{-1.0, 0.0});
  ASSERT_EQ(msd_mpc.solve(x0), ampc::SolveStatus::Success);
  Eigen::Matrix<double, m, 1> u_neg;
  msd_mpc.getNextInput(u_neg);

  ASSERT_GT(u_pos(0), 0.0);
  ASSERT_LT(u_neg(0), 0.0);

  // symmetric problem, so magnitudes should be similar
  ASSERT_NEAR(u_pos(0), -u_neg(0), 1e-3);
}

TEST(CondensedMPCProtectedTester,
     givenNonUniformReferenceTrajectory_SolvesCorrectly)
{
  const int n{2}, m{1}, T{10}, nc{10};
  CondensedMPCProtectedTester mpc{n, m,
                                  ampc::Parameterization::linearInterp(T, nc),
                                  ampc::Options{.use_input_cost = true}};
  mpc.setModel();

  mpc.setWeights(Vector2d{1.0, 0.1}, VectorXd::Constant(m, 1e-4));
  mpc.setInputLimits(VectorXd::Constant(m, -3.0), VectorXd::Constant(m, 3.0));
  mpc.setReferenceInput(VectorXd::Zero(m));

  // slowly ramping reference trajectory so u0 < u_max
  Vector2d tmp{0.05, 0.0};
  VectorXd x_traj_ref{n * T};
  for (int k{0}; k < T; ++k) {
    const double s = static_cast<double>(k + 1) / T;
    x_traj_ref.segment(k * n, n) = s * tmp;
  }
  mpc.setReferenceStateTrajectory(x_traj_ref);

  ASSERT_TRUE(mpc.initializeSolver());
  ASSERT_EQ(mpc.solve(Vector2d::Zero()), ampc::SolveStatus::Success);

  // Solve with uniform reference at the final value for comparison
  CondensedMPCProtectedTester mpc2{n, m,
                                   ampc::Parameterization::linearInterp(T, nc),
                                   ampc::Options{.use_input_cost = true}};
  mpc2.setModel();
  mpc2.setWeights(Vector2d{1.0, 0.1}, VectorXd::Constant(m, 1e-4));
  mpc2.setInputLimits(VectorXd::Constant(m, -3.0), VectorXd::Constant(m, 3.0));
  mpc2.setReferenceInput(VectorXd::Zero(m));
  mpc2.setReferenceState(Vector2d{1.0, 0.0}); // uniform at final value
  ASSERT_TRUE(mpc2.initializeSolver());
  ASSERT_EQ(mpc2.solve(Vector2d::Zero()), ampc::SolveStatus::Success);

  VectorXd u_ramp{m}, u_uniform{m};
  mpc.getNextInput(u_ramp);
  mpc2.getNextInput(u_uniform);

  // Ramp reference starts lower, so the optimal first input should be smaller
  // than for a uniform reference at the endpoint
  ASSERT_LT(u_ramp(0), u_uniform(0));
}
