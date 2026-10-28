// Refined input bounds (InputBoundMode::Refined): the knot refinement that
// gives the bound rows, and the rows and solutions of the MPC that uses them.

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <gtest/gtest.h>
#include <random>
#include <string>
#include <unsupported/Eigen/Splines>
#include <vector>

#include "affine_mpc/detail/vector_parameterization.hpp"
#include "affine_mpc/mpc.hpp"
#include "affine_mpc/options.hpp"
#include "affine_mpc/parameterization.hpp"
#include "affine_mpc/quadratic_program.hpp"
#include "utils.hpp"

namespace ampc = affine_mpc;
using namespace Eigen;
using Refinement = ampc::detail::VectorParameterization::KnotRefinement;

namespace {

Refinement refine(const ampc::Parameterization& param, int max_interval)
{
  return ampc::detail::VectorParameterization{1, param}.refineKnots(
      max_interval);
}

// The refined spline sampled at k = 0, ..., T-1, from its knots and control
// points (independent of Parameterization)
VectorXd evaluateRefined(const VectorXd& knots,
                         int degree,
                         int T,
                         const VectorXd& refined_ctrls)
{
  using Spline1d = Spline<double, 1>;
  const Spline1d::KnotVectorType tau{knots.transpose()};
  VectorXd u{T};
  for (int k{0}; k < T; ++k) {
    const double t = k;
    const Index span{Spline1d::Span(t, degree, tau)};
    const auto basis{Spline1d::BasisFunctions(t, degree, tau)};
    u(k) = 0.0;
    for (int i{0}; i <= degree; ++i)
      u(k) += basis(i) * refined_ctrls(span - degree + i);
  }
  return u;
}

// R c for the banded map R
VectorXd applyR(const Refinement& r, const VectorXd& ctrls)
{
  VectorXd refined{r.start.size()};
  for (Index j{0}; j < r.start.size(); ++j)
    refined(j) =
        r.weights.col(j).dot(ctrls.segment(r.start(j), r.weights.rows()));
  return refined;
}

// Parameterizations covering uniform, geometric, fractional, repeated, and
// unclamped knots
std::vector<std::pair<std::string, ampc::Parameterization>> splines()
{
  using P = ampc::Parameterization;
  VectorXd repeated{11}; // degree 2, a double knot at 12 and a triple at 25
  repeated << 0, 0, 0, 12, 12, 25, 25, 25, 39, 39, 39;
  VectorXd fractional{10}; // degree 3, non-integer knots
  fractional << 0, 0, 0, 0, 7.3, 22.9, 29, 29, 29, 29;
  VectorXd unclamped{9}; // degree 2, knots outside [0, T-1]
  unclamped << -6, -3, 0, 5.5, 11, 17, 23, 26, 29;
  return {
      {"uniform deg2", P::bspline(40, 2, 6)},
      {"uniform deg3", P::bspline(60, 3, 8)},
      {"uniform deg4", P::bspline(50, 4, 7)},
      {"geometric 1.4", P::bspline(80, 2, 9, 1.4)},
      {"geometric 0.7", P::bspline(80, 3, 9, 0.7)},
      {"repeated", P{40, 2, repeated}},
      {"fractional", P{30, 3, fractional}},
      {"unclamped", P{24, 2, unclamped}},
      {"short horizon", P::bspline(5, 2, 4)},
      {"long horizon", P::bspline(500, 3, 10)},
  };
}

// The active knot intervals [t_i, t_{i+1}], i = degree, ..., size-degree-2
std::vector<double> activeIntervals(const VectorXd& knots, int degree)
{
  std::vector<double> lengths;
  for (Index i{degree}; i < knots.size() - degree - 1; ++i)
    lengths.push_back(knots(i + 1) - knots(i));
  return lengths;
}

// Mass-spring-damper MPC with the input limits and every setup the solve
// needs
void setupMSD(ampc::MPC& mpc,
              int m,
              const VectorXd& u_min,
              const VectorXd& u_max,
              const Vector2d& x_ref)
{
  Matrix2d A;
  A << 0, 1, -0.6, -0.1;
  MatrixXd B{MatrixXd::Zero(2, m)};
  for (int j{0}; j < m; ++j)
    B(1, j) = 0.2 / (j + 1);
  mpc.setModelContinuous2Discrete(A, B, Vector2d::Zero(), 0.1);
  mpc.setInputLimits(u_min, u_max);
  mpc.setStateWeights(Vector2d{1.0, 0.1});
  mpc.setReferenceState(x_ref);
}

int inputBoundRows(const ampc::MPC& mpc)
{
  for (const ampc::QPBlock& b : mpc.getQPConstraintBlocks())
    if (b.name == "input_bounds")
      return b.size;
  return -1;
}

// The input bound rows of A as a dense matrix
MatrixXd inputBoundMatrix(ampc::MPC& mpc, int n)
{
  ampc::QuadraticProgram qp;
  mpc.assembleQP(VectorXd::Zero(n), qp);
  for (const ampc::QPBlock& b : mpc.getQPConstraintBlocks())
    if (b.name == "input_bounds")
      return MatrixXd{qp.A}
          .middleRows(b.offset, b.size)
          .leftCols(mpc.getNumControlPoints() * mpc.getInputDim());
  return {};
}

} // namespace

// ---- Knot refinement --------------------------------------------------------

TEST(KnotRefinementTest, givenSplines_RefinedControlPointsReproduceSpline)
{
  std::mt19937 gen{7};
  std::uniform_real_distribution<double> dist{-2.0, 2.0};
  for (const auto& [name, param] : splines()) {
    for (const int L : {1, 2, 3, 5, 8, 16, 1000}) {
      SCOPED_TRACE(name + " max_interval=" + std::to_string(L));
      const Refinement r{refine(param, L)};
      const int p{param.degree}, T{param.horizon_steps};
      ASSERT_EQ(r.weights.rows(), p + 1);
      ASSERT_EQ(r.start.size(), r.knots.size() - p - 1);
      for (int trial{0}; trial < 5; ++trial) {
        VectorXd c{param.num_control_points};
        for (Index i{0}; i < c.size(); ++i)
          c(i) = dist(gen);
        const VectorXd u{param.evaluate(1, c)};
        const VectorXd u_ref{evaluateRefined(r.knots, p, T, applyR(r, c))};
        ASSERT_TRUE(expectEigenNear(u_ref, u, 1e-12));
      }
    }
  }
}

TEST(KnotRefinementTest, givenSplines_RIsBandedConvexCombination)
{
  for (const auto& [name, param] : splines()) {
    for (const int L : {1, 4, 16}) {
      SCOPED_TRACE(name + " max_interval=" + std::to_string(L));
      const Refinement r{refine(param, L)};
      for (Index j{0}; j < r.start.size(); ++j) {
        EXPECT_GE(r.start(j), 0);
        EXPECT_LE(r.start(j) + r.weights.rows(), param.num_control_points);
        if (j > 0) {
          EXPECT_GE(r.start(j), r.start(j - 1));
        }
        EXPECT_GE(r.weights.col(j).minCoeff(), -1e-14);
        EXPECT_NEAR(r.weights.col(j).sum(), 1.0, 1e-12);
      }
    }
  }
}

TEST(KnotRefinementTest, givenMaxInterval_SplitsEveryIntervalIntoEqualParts)
{
  for (const auto& [name, param] : splines()) {
    for (const int L : {1, 2, 3, 7, 16, 1000}) {
      SCOPED_TRACE(name + " max_interval=" + std::to_string(L));
      const int p{param.degree};
      const Refinement r{refine(param, L)};

      // the end knots are unchanged and the original knots are kept
      ASSERT_TRUE(
          expectEigenNear(r.knots.head(p + 1), param.knots.head(p + 1), 0.0));
      ASSERT_TRUE(
          expectEigenNear(r.knots.tail(p + 1), param.knots.tail(p + 1), 0.0));
      for (Index i{0}; i < param.knots.size(); ++i)
        EXPECT_NE(std::find(r.knots.data(), r.knots.data() + r.knots.size(),
                            param.knots(i)),
                  r.knots.data() + r.knots.size());

      // each nonempty interval is split into max(2, ceil(length / L)) equal
      // parts, so every interval is split and none is longer than L
      int expected_inserted{0};
      for (const double length : activeIntervals(param.knots, p))
        if (length > 0.0)
          expected_inserted +=
              std::max(2, static_cast<int>(std::ceil(length / L))) - 1;
      EXPECT_EQ(r.knots.size() - param.knots.size(), expected_inserted);
      for (const double length : activeIntervals(r.knots, p))
        EXPECT_LE(length, L * (1.0 + 1e-12));
    }
  }
}

TEST(KnotRefinementTest, givenIntervalsAtMostMaxInterval_SplitsEachInTwo)
{
  // uniform knots with intervals of exactly 6 samples (4 intervals)
  const auto param{ampc::Parameterization::bspline(25, 2, 6)};
  ASSERT_EQ(activeIntervals(param.knots, 2).front(), 6.0);
  EXPECT_EQ(refine(param, 6).knots.size(), param.knots.size() + 4);
  EXPECT_EQ(refine(param, 100).knots.size(), param.knots.size() + 4);
  EXPECT_EQ(refine(param, 3).knots.size(), param.knots.size() + 4);
  EXPECT_EQ(refine(param, 2).knots.size(), param.knots.size() + 8);
  // the inserted knots are the midpoints
  const Refinement r{refine(param, 6)};
  for (Index i{0}; i < 4; ++i)
    EXPECT_DOUBLE_EQ(r.knots(3 + 2 * i), 3.0 + 6.0 * i);
}

TEST(KnotRefinementTest, givenRefinement_SampledPointsAreThoseSamplesUse)
{
  // a refined basis function that is zero at every sample is left out, and
  // every sampled input is a convex combination of the kept ones
  using Spline1d = Spline<double, 1>;
  for (const auto& [name, param] : splines()) {
    for (const int L : {1, 2, 5}) {
      SCOPED_TRACE(name + " max_interval=" + std::to_string(L));
      const int p{param.degree};
      const Refinement r{refine(param, L)};
      const Spline1d::KnotVectorType tau{r.knots.transpose()};
      std::vector<int> used;
      for (int k{0}; k < param.horizon_steps; ++k) {
        const double t = k;
        const Index span{Spline1d::Span(t, p, tau)};
        const auto basis{Spline1d::BasisFunctions(t, p, tau)};
        for (int i{0}; i <= p; ++i)
          if (basis(i) != 0.0)
            used.push_back(static_cast<int>(span) - p + i);
      }
      std::sort(used.begin(), used.end());
      used.erase(std::unique(used.begin(), used.end()), used.end());
      EXPECT_EQ(r.sampled, used);
    }
  }
}

// ---- MPC bound rows
// ----------------------------------------------------------

TEST(RefinedInputBoundsTest, givenDegree2OrMore_BoundsSampledRefinedPoints)
{
  for (const int m : {1, 2}) {
    for (const auto& [name, param] : splines()) {
      for (const int L : {1, 4, 16}) {
        SCOPED_TRACE(name + " m=" + std::to_string(m)
                     + " max_interval=" + std::to_string(L));
        const int n{2};
        ampc::MPC mpc{n,
                      m,
                      param,
                      {.input_bounds = ampc::InputBoundMode::Refined,
                       .refined_bound_interval = L}};
        setupMSD(mpc, m, VectorXd::Constant(m, -1.0),
                 VectorXd::Constant(m, 2.0), Vector2d{1.0, 0.0});
        const Refinement r{refine(param, L)};
        const int num_rows{static_cast<int>(r.sampled.size())};
        if (num_rows >= param.horizon_steps) {
          // no fewer rows than the samples: the sampled rows are used
          ampc::MPC exact{
              n, m, param, {.input_bounds = ampc::InputBoundMode::Exact}};
          setupMSD(exact, m, VectorXd::Constant(m, -1.0),
                   VectorXd::Constant(m, 2.0), Vector2d{1.0, 0.0});
          EXPECT_TRUE(expectEigenNear(inputBoundMatrix(mpc, n),
                                      inputBoundMatrix(exact, n), 0.0));
          continue;
        }
        ASSERT_EQ(inputBoundRows(mpc), m * num_rows);

        // the rows are R c on the sampled refined points, channel by channel
        const MatrixXd A{inputBoundMatrix(mpc, n)};
        std::mt19937 gen{3};
        std::uniform_real_distribution<double> dist{-1.0, 1.0};
        VectorXd ctrls{m * param.num_control_points};
        for (Index i{0}; i < ctrls.size(); ++i)
          ctrls(i) = dist(gen);
        const VectorXd Ac{A * ctrls};
        const Map<const MatrixXd> ctrl_mat{ctrls.data(), m,
                                           param.num_control_points};
        for (int ch{0}; ch < m; ++ch) {
          const VectorXd refined{applyR(r, VectorXd{ctrl_mat.row(ch)})};
          for (int row{0}; row < num_rows; ++row)
            EXPECT_NEAR(Ac(m * row + ch),
                        refined(r.sampled[static_cast<std::size_t>(row)]),
                        1e-12);
        }
      }
    }
  }
}

TEST(RefinedInputBoundsTest, givenDegree0Or1_UsesTheExactCheapRows)
{
  const int n{2}, m{2};
  VectorXd knots{6};
  knots << 0, 0, 2.5, 6.5, 9, 9;
  for (const auto& param : {ampc::Parameterization::moveBlocking(30, 6),
                            ampc::Parameterization::linearInterp(30, 6),
                            ampc::Parameterization{10, 1, knots}}) {
    SCOPED_TRACE(::testing::Message() << param);
    const ampc::InputBoundMode same{param.degree == 0
                                        ? ampc::InputBoundMode::ControlPoints
                                        : ampc::InputBoundMode::Exact};
    ampc::MPC refined{n,
                      m,
                      param,
                      {.input_bounds = ampc::InputBoundMode::Refined,
                       .refined_bound_interval = 1}};
    ampc::MPC expected{n, m, param, {.input_bounds = same}};
    for (ampc::MPC* mpc : {&refined, &expected})
      setupMSD(*mpc, m, VectorXd::Constant(m, -1.0), VectorXd::Constant(m, 2.0),
               Vector2d{1.0, 0.0});
    EXPECT_TRUE(expectEigenNear(inputBoundMatrix(refined, n),
                                inputBoundMatrix(expected, n), 0.0));
  }
}

TEST(RefinedInputBoundsTest, givenCustomBasis_BoundsEverySample)
{
  const int n{2}, m{1}, T{12};
  const auto param{
      ampc::Parameterization::fromBasis(T, laguerreBasis(T, 4, 0.6))};
  ampc::MPC mpc{n, m, param, {.input_bounds = ampc::InputBoundMode::Refined}};
  setupMSD(mpc, m, VectorXd::Constant(m, -1.0), VectorXd::Constant(m, 2.0),
           Vector2d{1.0, 0.0});
  EXPECT_EQ(inputBoundRows(mpc), m * T);
  EXPECT_TRUE(
      expectEigenNear(inputBoundMatrix(mpc, n), denseBasis(param), 0.0));
}

TEST(RefinedInputBoundsTest, givenLongHorizon_RowCountsLieBetweenModes)
{
  // T = 500, 10 control points: intervals of 62.4 samples, each split into
  // max(2, ceil(62.4 / L)) parts, plus degree rows
  const int n{2}, m{1}, T{500}, nc{10};
  for (const int deg : {2, 3}) {
    const auto param{ampc::Parameterization::bspline(T, deg, nc)};
    auto rows = [&](ampc::InputBoundMode mode, int L) {
      ampc::MPC mpc{
          n, m, param, {.input_bounds = mode, .refined_bound_interval = L}};
      return inputBoundRows(mpc);
    };
    EXPECT_EQ(rows(ampc::InputBoundMode::ControlPoints, 8), nc);
    EXPECT_EQ(rows(ampc::InputBoundMode::Exact, 8), T);
    for (const int L : {4, 8, 16, 32, 64, 1000}) {
      SCOPED_TRACE("degree " + std::to_string(deg) + " L=" + std::to_string(L));
      const int got{rows(ampc::InputBoundMode::Refined, L)};
      const int intervals{nc - deg};
      const double length{(T - 1.0) / intervals};
      const int refined_intervals{
          intervals * std::max(2, static_cast<int>(std::ceil(length / L)))};
      EXPECT_EQ(got, refined_intervals + deg);
      EXPECT_LT(got, T);
    }
  }
}

TEST(RefinedInputBoundsTest, givenTooManyRefinedRows_UsesExactRows)
{
  // T = 10, 5 cubic control points: 2 intervals of 4.5 samples; L = 2 splits
  // each into 3, giving 6 + 3 = 9 refined rows (fewer than T), and L = 1
  // splits each into 5, giving 13 (not fewer), so the sampled rows are used
  const int n{2}, m{2}, T{10};
  const auto param{ampc::Parameterization::bspline(T, 3, 5)};
  auto make = [&](ampc::InputBoundMode mode, int L) {
    ampc::MPC mpc{
        n, m, param, {.input_bounds = mode, .refined_bound_interval = L}};
    setupMSD(mpc, m, VectorXd::Constant(m, -1.0), VectorXd::Constant(m, 2.0),
             Vector2d{1.0, 0.0});
    return inputBoundMatrix(mpc, n);
  };
  ASSERT_EQ(refine(param, 2).sampled.size(), 9u);
  ASSERT_EQ(refine(param, 1).sampled.size(), 13u);
  EXPECT_EQ(make(ampc::InputBoundMode::Refined, 2).rows(), m * 9);
  EXPECT_TRUE(expectEigenNear(make(ampc::InputBoundMode::Refined, 1),
                              make(ampc::InputBoundMode::Exact, 1), 0.0));
}

TEST(RefinedInputBoundsTest, givenShortHorizon_IsTighterThanControlPoints)
{
  // T = 40, 6 quadratic control points: every interval (9.75 samples) is
  // shorter than the default L = 16 but is still split in two, so the optimal
  // cost with the limit active inside the horizon is strictly lower than with
  // the control point bounds (and no lower than with exact bounds)
  const int n{2}, m{1}, T{40};
  const auto param{ampc::Parameterization::bspline(T, 2, 6)};
  auto cost = [&](ampc::InputBoundMode mode, int expected_rows) {
    ampc::MPC mpc{
        n, m, param, {.input_bounds = mode, .slew_initial_input = true}};
    setupMSD(mpc, m, VectorXd::Zero(1), VectorXd::Constant(1, 3.0),
             Vector2d{1.0, 0.0});
    mpc.setSlewRateInitial(VectorXd::Constant(1, 0.5));
    EXPECT_EQ(inputBoundRows(mpc), expected_rows);
    EXPECT_TRUE(mpc.initializeSolver());
    EXPECT_EQ(mpc.solve(Vector2d::Zero()), ampc::SolveStatus::Success);
    VectorXd u{m * T};
    mpc.getInputTrajectory(u);
    EXPECT_GE(u.minCoeff(), -1e-4);
    EXPECT_LE(u.maxCoeff(), 3.0 + 1e-4);
    return mpc.getSolveInfo().objective;
  };
  const double control_points{cost(ampc::InputBoundMode::ControlPoints, 6)};
  const double refined{cost(ampc::InputBoundMode::Refined, 8 + 2)};
  const double exact{cost(ampc::InputBoundMode::Exact, T)};
  const double tol{1e-6 * std::abs(control_points)};
  EXPECT_LT(refined, control_points - 100 * tol);
  EXPECT_LE(exact, refined + tol);
}

TEST(RefinedInputBoundsTest, givenDefaultOptions_KeepsControlPointBounds)
{
  const ampc::Options opts{};
  EXPECT_EQ(opts.input_bounds, ampc::InputBoundMode::ControlPoints);
  EXPECT_EQ(opts.refined_bound_interval, 16);
}

TEST(RefinedInputBoundsTest, givenNonPositiveInterval_Throws)
{
  const auto param{ampc::Parameterization::bspline(20, 2, 5)};
  for (const int L : {0, -3}) {
    for (const auto mode :
         {ampc::InputBoundMode::Refined, ampc::InputBoundMode::ControlPoints}) {
      expectInvalidArgumentWithMessage(
          [&] {
            ampc::MPC mpc{2,
                          1,
                          param,
                          {.input_bounds = mode, .refined_bound_interval = L}};
          },
          "refined_bound_interval must be positive");
    }
  }
}

// ---- Solutions
// ---------------------------------------------------------------

TEST(RefinedInputBoundsTest, givenRandomSolves_SampledInputsStayInsideLimits)
{
  std::mt19937 gen{11};
  std::uniform_real_distribution<double> lim{0.2, 2.0}, ref{-6.0, 6.0};
  int active{0};
  for (const int m : {1, 2}) {
    for (const auto& [name, param] : splines()) {
      if (param.horizon_steps > 100)
        continue; // keep the test fast
      for (const int L : {1, 3, 8}) {
        const int n{2}, T{param.horizon_steps};
        const ampc::Options opts{.input_bounds = ampc::InputBoundMode::Refined,
                                 .refined_bound_interval = L};
        ampc::MPC condensed{n, m, param, opts};
        ampc::MPC sparse{n, m, param, opts, ampc::Formulation::Sparse};
        for (int trial{0}; trial < 3; ++trial) {
          SCOPED_TRACE(name + " m=" + std::to_string(m) + " max_interval="
                       + std::to_string(L) + " trial " + std::to_string(trial));
          VectorXd u_min{m}, u_max{m};
          for (int j{0}; j < m; ++j) {
            u_min(j) = -lim(gen);
            u_max(j) = lim(gen);
          }
          const Vector2d x_ref{ref(gen), 0.0};
          for (ampc::MPC* mpc : {&condensed, &sparse})
            setupMSD(*mpc, m, u_min, u_max, x_ref);
          if (trial == 0) {
            ASSERT_TRUE(condensed.initializeSolver());
            ASSERT_TRUE(sparse.initializeSolver());
          }
          const Vector2d x0{0.0, 0.0};
          ASSERT_EQ(condensed.solve(x0), ampc::SolveStatus::Success);
          ASSERT_EQ(sparse.solve(x0), ampc::SolveStatus::Success);

          // (with two inputs the optimal inputs need not be unique, so the
          // formulations are compared by their cost)
          EXPECT_NEAR(condensed.getSolveInfo().objective,
                      sparse.getSolveInfo().objective,
                      1e-4 * (1.0 + std::abs(sparse.getSolveInfo().objective)));
          for (const ampc::MPC* mpc : {&condensed, &sparse}) {
            VectorXd u{m * T};
            mpc->getInputTrajectory(u);
            const Map<const MatrixXd> u_mat{u.data(), m, T};
            const double tol{1e-4};
            for (int j{0}; j < m; ++j) {
              EXPECT_GE(u_mat.row(j).minCoeff(), u_min(j) - tol);
              EXPECT_LE(u_mat.row(j).maxCoeff(), u_max(j) + tol);
              if (u_mat.row(j).maxCoeff() > u_max(j) - 0.1
                  || u_mat.row(j).minCoeff() < u_min(j) + 0.1)
                ++active;
            }
          }
        }
      }
    }
  }
  EXPECT_GT(active, 0) << "no solve came near the limits";
}

TEST(RefinedInputBoundsTest, givenActiveLimit_FeasibleSetsAreNested)
{
  // Reaching x_ref quickly needs u > u_max, and the initial slew limit keeps
  // u_0 low, so the upper limit is active inside the horizon. The
  // bounds are nested: control points within the limits give refined control
  // points within them, a finer nested refinement keeps them within, and
  // refined control points within the limits give sampled inputs within them.
  // So the optimal cost can only decrease from ControlPoints to Refined with
  // L = 8, 4, 2 (nested knots here: intervals of 14.75 samples) to Exact.
  const int n{2}, m{1}, T{60};
  const auto param{ampc::Parameterization::bspline(T, 2, 6)};
  const VectorXd u_min{VectorXd::Zero(1)}, u_max{VectorXd::Constant(1, 3.0)};
  auto cost = [&](ampc::InputBoundMode mode, int L) {
    double objective[2]{};
    int i{0};
    for (const auto formulation :
         {ampc::Formulation::Condensed, ampc::Formulation::Sparse}) {
      ampc::MPC mpc{n,
                    m,
                    param,
                    {.input_bounds = mode,
                     .refined_bound_interval = L,
                     .slew_initial_input = true},
                    formulation};
      setupMSD(mpc, m, u_min, u_max, Vector2d{1.0, 0.0});
      mpc.setSlewRateInitial(VectorXd::Constant(1, 0.5));
      EXPECT_TRUE(mpc.initializeSolver());
      EXPECT_EQ(mpc.solve(Vector2d::Zero()), ampc::SolveStatus::Success);
      VectorXd u{m * T};
      mpc.getInputTrajectory(u);
      EXPECT_GE(u.minCoeff(), -1e-4);
      EXPECT_LE(u.maxCoeff(), 3.0 + 1e-4);
      objective[i++] = mpc.getSolveInfo().objective;
    }
    EXPECT_NEAR(objective[0], objective[1], 1e-6 * std::abs(objective[0]));
    return objective[0];
  };
  const double control_points{cost(ampc::InputBoundMode::ControlPoints, 8)};
  const double coarse{cost(ampc::InputBoundMode::Refined, 8)};
  const double medium{cost(ampc::InputBoundMode::Refined, 4)};
  const double fine{cost(ampc::InputBoundMode::Refined, 2)};
  const double exact{cost(ampc::InputBoundMode::Exact, 8)};
  const double tol{1e-6 * std::abs(control_points)};
  EXPECT_LT(coarse, control_points - 100 * tol);
  EXPECT_LE(medium, coarse + tol);
  EXPECT_LE(fine, medium + tol);
  EXPECT_LE(exact, fine + tol);
}
