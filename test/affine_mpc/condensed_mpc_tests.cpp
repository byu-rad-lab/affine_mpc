#include <Eigen/Core>
#include <algorithm>
#include <cstddef>
#include <gtest/gtest.h>
#include <numeric>
#include <optional>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "affine_mpc/mpc.hpp"
#include "affine_mpc/parameterization.hpp"
#include "formulation.hpp" // internal (src/): the condensed state map
#include "qp_builder.hpp"
#include "utils.hpp"

using namespace Eigen;
namespace ampc = affine_mpc;


// Tests are done with a Mass-Spring-Damper system (2 states, 1 input)
class CondensedMPCProtectedTester : public ampc::MPC
{
public:
  using MPC::MPC;

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

  // assembling the QP computes the condensed state map x = S c + v
  void findSAndV(const Eigen::Ref<const Eigen::VectorXd>& x0)
  {
    (void)qpUpdateX0(x0);
  }

  void findPAndQ(const Eigen::Ref<const Eigen::VectorXd>& x0)
  {
    qpUpdateX0(x0);
  }

  auto getAd() const { return readModel(*this).Ad; }
  auto getBd() const { return readModel(*this).Bd; }
  auto getWd() const { return readModel(*this).wd; }
  auto getS() const { return formulation().S(); }
  auto getV() const { return formulation().v(); }
  auto getP() const { return getQPCostMatrix(); }
  auto stateCostProduct() const { return formulation().stateCostProduct(); }
  auto getQ() const { return this->qp().q; }

private:
  const ampc::detail::CondensedFormulation& formulation() const
  {
    return static_cast<const ampc::detail::CondensedFormulation&>(
        qpBuilder().formulation());
  }
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
      ampc::Options{.input_cost = ampc::InputCostMode::ControlPoints}};
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
  P_test = R.replicate(nc, 1).asDiagonal();
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
      ampc::Options{.input_cost = ampc::InputCostMode::ControlPoints,
                    .slew_control_points = true}};
  msd_mpc.setModel();

  Eigen::Matrix<double, n, 1> Q{1, 0.11};
  Eigen::Matrix<double, m, 1> R{.0001};
  msd_mpc.setWeights(Q, R);
  Eigen::Vector2d x0{1, 1}, x_goal{1, 0};
  msd_mpc.setReferenceState(x_goal);
  Eigen::Matrix<double, m, 1> u_goal{0}, u_min{0}, u_max{3}, slew{1};
  msd_mpc.setInputLimits(u_min, u_max);
  msd_mpc.setReferenceInput(u_goal);
  msd_mpc.setSlewRate(slew);

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
      ampc::Options{.input_cost = ampc::InputCostMode::ControlPoints,
                    .state_bounds = ampc::StateBoundMode::Hard}};
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
      ampc::Options{.input_cost = ampc::InputCostMode::ControlPoints}};
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
      ampc::Options{.input_cost = ampc::InputCostMode::ControlPoints}};
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
  CondensedMPCProtectedTester mpc{
      n, m, ampc::Parameterization::linearInterp(T, nc),
      ampc::Options{.input_cost = ampc::InputCostMode::ControlPoints}};
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
  CondensedMPCProtectedTester mpc2{
      n, m, ampc::Parameterization::linearInterp(T, nc),
      ampc::Options{.input_cost = ampc::InputCostMode::ControlPoints}};
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

namespace {

// A random parameterization of the given kind: 0 move-blocking with random
// change points, 1 uniform B-spline with a random knot ratio, 2 clamped
// B-spline with random interior knots, 3 linear interpolation, 4 custom
// dense basis. Empty if the random knots were rejected.
std::optional<ampc::Parameterization> randomParameterization(const int kind,
                                                             const int T,
                                                             const int degree,
                                                             const int ncp,
                                                             std::mt19937& gen)
{
  std::uniform_real_distribution<double> unit{0.0, 1.0};
  try {
    switch (kind) {
    case 0: {
      std::vector<int> steps(static_cast<std::size_t>(T - 1));
      std::iota(steps.begin(), steps.end(), 1);
      std::shuffle(steps.begin(), steps.end(), gen);
      steps.resize(static_cast<std::size_t>(ncp - 1));
      std::sort(steps.begin(), steps.end());
      VectorXd change_points{ncp};
      change_points(0) = 0.0;
      for (int i{1}; i < ncp; ++i)
        change_points(i) = steps[static_cast<std::size_t>(i - 1)];
      return ampc::Parameterization::moveBlocking(T, change_points);
    }
    case 1: // uniform knots with a control point per step (else too short)
      return ampc::Parameterization::bspline(
          T, degree, ncp, ncp == T ? 1.0 : 0.7 + 0.7 * unit(gen));
    case 2: {
      VectorXd knots{ncp + degree + 1};
      knots.head(degree + 1).setZero();
      knots.tail(degree + 1).setConstant(T - 1);
      for (int i{degree + 1}; i < ncp; ++i)
        knots(i) = (T - 1) * unit(gen);
      std::sort(knots.data() + degree + 1, knots.data() + ncp);
      return ampc::Parameterization{T, degree, knots};
    }
    case 3:
      return ampc::Parameterization::linearInterp(T, ncp);
    default: {
      std::normal_distribution<double> normal;
      return ampc::Parameterization::fromBasis(
          T, MatrixXd::NullaryExpr(T, ncp, [&] { return normal(gen); }));
    }
    }
  } catch (const std::invalid_argument&) {
    return std::nullopt;
  }
}

} // namespace

// The state cost computed on the band (the upper triangle from each column's
// first nonzero row, then mirrored) agrees with the plain product
// S^T diag(Q) S to round-off, on both sides of the size threshold that
// chooses between the dot products and the matrix-vector products, and the
// dense basis keeps the plain product
TEST(CondensedMPCProtectedTester, givenBandedBasis_StateCostMatchesPlainProduct)
{
  struct Size
  {
    int n, m, T;
  };
  const std::vector<Size> sizes{
      {2, 1, 10}, {3, 2, 16}, {6, 2, 30}, {8, 3, 40}, {12, 4, 50}};
  std::mt19937 gen{7};
  std::normal_distribution<double> normal;
  std::uniform_real_distribution<double> unit{0.0, 1.0};
  using Product = ampc::detail::CondensedFormulation::StateCostProduct;
  int num_dense{0}, num_dots{0}, num_gemv{0};
  double max_rel{0.0};
  for (const Size& s : sizes)
    for (int kind{0}; kind < 5; ++kind)
      for (int degree{kind == 2 ? 1 : 0}; degree <= 3; ++degree) {
        if ((kind == 0 || kind >= 3) && degree > 0)
          continue; // degree only varies for B-splines
        for (const int ncp : {std::max(degree + 2, s.T / 4), s.T / 2, s.T}) {
          if (kind == 2 && ncp == s.T)
            continue; // too many random knots to keep the basis full rank
          std::optional<ampc::Parameterization> param;
          for (int attempt{0}; attempt < 20 && !param; ++attempt)
            if (auto p{randomParameterization(kind, s.T, degree, ncp, gen)})
              param.emplace(*p);
          ASSERT_TRUE(param.has_value())
              << "T " << s.T << " kind " << kind << " degree " << degree
              << " ncp " << ncp;

          CondensedMPCProtectedTester mpc{s.n, s.m, *param};
          const MatrixXd Ad{MatrixXd::Identity(s.n, s.n)
                            + 0.1 * MatrixXd::NullaryExpr(s.n, s.n, [&] {
                                return normal(gen);
                              })};
          const MatrixXd Bd{
              MatrixXd::NullaryExpr(s.n, s.m, [&] { return normal(gen); })};
          mpc.setModelDiscrete(Ad, Bd, VectorXd::Zero(s.n));
          const VectorXd Q{
              VectorXd::NullaryExpr(s.n, [&] { return 2.0 * unit(gen); })};
          const VectorXd Qf{
              VectorXd::NullaryExpr(s.n, [&] { return 5.0 * unit(gen); })};
          mpc.setStateWeights(Q, Qf);
          mpc.findPAndQ(VectorXd::Zero(s.n));

          VectorXd Q_big{s.n * s.T};
          Q_big << Q.replicate(s.T - 1, 1), Qf;
          const MatrixXd S{mpc.getS()};
          const MatrixXd P_plain{S.transpose() * Q_big.asDiagonal() * S};
          const MatrixXd P{mpc.getP()};
          const double rel{(P - P_plain).cwiseAbs().maxCoeff()
                           / P_plain.cwiseAbs().maxCoeff()};
          max_rel = std::max(max_rel, rel);
          EXPECT_LE(rel, 1e-14)
              << "n " << s.n << " m " << s.m << " T " << s.T << " kind " << kind
              << " degree " << degree << " ncp " << ncp;
          if (kind == 4) { // a custom basis is dense
            EXPECT_EQ(mpc.stateCostProduct(), Product::Dense);
          }
          switch (mpc.stateCostProduct()) {
          case Product::Dense:
            ++num_dense;
            break;
          case Product::BandDots:
            ++num_dots;
            break;
          case Product::BandGemv:
            ++num_gemv;
            break;
          }
          if (mpc.stateCostProduct() != Product::Dense) {
            EXPECT_TRUE(P == P.transpose()); // mirrored exactly
          }
        }
      }
  // every product is covered
  EXPECT_GT(num_dense, 0);
  EXPECT_GT(num_dots, 0);
  EXPECT_GT(num_gemv, 0);
  std::ostringstream max_rel_text;
  max_rel_text << max_rel;
  RecordProperty("max_relative_difference", max_rel_text.str());
}
