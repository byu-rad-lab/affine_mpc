#include <Eigen/Core>
#include <Eigen/SVD>
#include <gtest/gtest.h>

#include <algorithm>
#include <iostream>
#include <limits>
#include <random>
#include <sstream>
#include <string>
#include <vector>

#include "affine_mpc/parameterization.hpp"
#include "utils.hpp"

#include <unsupported/Eigen/Splines>

using namespace Eigen;
namespace ampc = affine_mpc;
using ampc::operator<<;


// ---- static unform clamped knots -------------------------------------------

TEST(ParameterizationUniformClampedKnots, givenHorizon_Throws)
{
  expectInvalidArgumentWithMessage(
      [&]() { ampc::Parameterization::makeUniformClampedKnots(0, 0, 1); },
      "horizon_steps must be positive");
}

TEST(ParameterizationUniformClampedKnots, givenInvalidDegree_Throws)
{
  expectInvalidArgumentWithMessage(
      [&]() { ampc::Parameterization::makeUniformClampedKnots(5, -1, 1); },
      "degree can not be negative");
}

TEST(ParameterizationUniformClampedKnots, givenInvalidNumControls_Throws)
{
  expectInvalidArgumentWithMessage(
      [&]() { ampc::Parameterization::bspline(5, 0, 0); },
      "must have at least degree+1 control points");
  expectInvalidArgumentWithMessage(
      [&]() { ampc::Parameterization::bspline(5, 1, 1); },
      "must have at least degree+1 control points");
  expectInvalidArgumentWithMessage(
      [&]() { ampc::Parameterization::bspline(5, 2, 2); },
      "must have at least degree+1 control points");
  expectInvalidArgumentWithMessage(
      [&]() { ampc::Parameterization::bspline(5, 0, 6); },
      "num_control_points can not be greater than horizon_steps");
}

// ---- Direct constructor (uniform knots) ------------------------------------

TEST(ParameterizationDirectConstructor, givenInvalidHorizonSteps_Throws)
{
  expectInvalidArgumentWithMessage(
      [&]() {
        const Vector4d knots{0, 1, 2, 3};
        ampc::Parameterization{-1, 0, knots};
      },
      "horizon_steps must be positive");
}

TEST(ParameterizationDirectConstructor, givenNegativeDegree_Throws)
{
  const Vector4d knots{0, 1, 2, 3};
  expectInvalidArgumentWithMessage(
      [&]() { ampc::Parameterization{5, -1, knots}; },
      "degree can not be negative");
}

TEST(ParameterizationDirectConstructor,
     givenDegreeGreaterThanHorizonSteps_Throws)
{
  expectInvalidArgumentWithMessage([&]() { ampc::Parameterization{3, 4, 5}; },
                                   "degree must be less than horizon_steps");
}

TEST(ParameterizationDirectConstructor, givenKnotsTooSmallForDegree_Throws)
{
  expectInvalidArgumentWithMessage(
      [&]() {
        const Vector3d knots{0, 1, 4};
        ampc::Parameterization{5, 1, knots};
      },
      "Size of knots must be at least 2*(degree+1)");
}

TEST(ParameterizationDirectConstructor, givenKnotsTooLargeForHorizon_Throws)
{
  expectInvalidArgumentWithMessage(
      [&]() {
        VectorXd knots{6};
        knots << 0, 1, 2, 3, 4, 4;
        ampc::Parameterization{4, 0, knots};
      },
      "Size of knots can not exceed horizon_steps+degree+1");
}

TEST(ParameterizationDirectConstructor, givenFirstActiveKnotNotZero_Throws)
{
  expectInvalidArgumentWithMessage(
      [&]() {
        VectorXd knots{5};
        knots << 0, 1, 2, 3, 3;
        ampc::Parameterization{4, 1, knots};
      },
      "First active knot must equal zero");
}

TEST(ParameterizationDirectConstructor,
     givenLastActiveKnotNotHorizonMinus1_Throws)
{
  expectInvalidArgumentWithMessage(
      [&]() {
        VectorXd knots{5};
        knots << 0, 0, 1, 2, 2;
        ampc::Parameterization{4, 1, knots};
      },
      "Last active knot must equal horizon_steps-1");
}

TEST(ParameterizationDirectConstructor, givenKnotsDecreasing_Throws)
{
  expectInvalidArgumentWithMessage(
      [&]() {
        VectorXd knots{5};
        knots << 0, 0, 1, 3, 2;
        ampc::Parameterization{4, 1, knots};
      },
      "knots must be non-decreasing");
}

TEST(ParameterizationDirectConstructor,
     givenRepeatedFirstActiveKnotUnclamped_Throws)
{
  // basis function 0 is nonzero only on [-1, 0), which contains no sample
  expectInvalidArgumentWithMessage(
      [&]() {
        VectorXd knots{5};
        knots << -1, 0, 0, 3, 4;
        ampc::Parameterization{4, 1, knots};
      },
      "control point 0 has no sample of its own");
}

TEST(ParameterizationDirectConstructor,
     givenMoveBlockingParams_FormsKnotsCorrectly)
{
  // degree=0: no clamped head/tail, all knots are active, LinSpaced over [0,
  // T-1]
  const int T{6}, nc{3}, deg{0};
  const ampc::Parameterization p{T, deg, nc};

  ASSERT_EQ(p.horizon_steps, T);
  ASSERT_EQ(p.num_control_points, nc);
  ASSERT_EQ(p.degree, deg);
  ASSERT_EQ(p.knots.size(), nc + deg + 1);

  VectorXd knots_expected{nc + deg + 1};
  knots_expected = ArrayXd::LinSpaced(nc + deg + 1, 0, T - 1);
  ASSERT_TRUE(expectEigenNear(p.knots, knots_expected, 1e-15));
}

TEST(ParameterizationDirectConstructor,
     givenLinearInterpParams_FormsKnotsCorrectly)
{
  // degree=1: head(1)=0, tail(1)=T-1, 3 active knots LinSpaced over [0, T-1]
  const int T{6}, nc{3}, deg{1};
  const ampc::Parameterization p{T, deg, nc};

  ASSERT_EQ(p.horizon_steps, T);
  ASSERT_EQ(p.num_control_points, nc);
  ASSERT_EQ(p.degree, deg);
  ASSERT_EQ(p.knots.size(), nc + deg + 1);

  VectorXd knots_expected{5};
  knots_expected << 0, 0, 2.5, 5, 5;
  ASSERT_TRUE(expectEigenNear(p.knots, knots_expected, 1e-15));
}

TEST(ParameterizationDirectConstructor, givenBsplineParams_FormsKnotsCorrectly)
{
  // degree=2: head(2)=0, tail(2)=T-1, 4 active knots LinSpaced over [0, T-1]
  // This mirrors the mpc_base_tests.cpp spline test
  const int T{10}, nc{5}, deg{2};
  const ampc::Parameterization p{T, deg, nc};

  ASSERT_EQ(p.knots.size(), nc + deg + 1);

  VectorXd knots_expected{8};
  knots_expected << 0, 0, 0, 3, 6, 9, 9, 9;
  ASSERT_TRUE(expectEigenNear(p.knots, knots_expected, 1e-15));
}

TEST(ParameterizationDirectConstructor,
     givenMinimalParams_nc1_deg0_FormsKnotsCorrectly)
{
  // Minimal case: single control point, degree 0 — constant input over horizon
  const int T{5}, nc{1}, deg{0};
  const ampc::Parameterization p{T, deg, nc};

  ASSERT_EQ(p.knots.size(), 2);
  VectorXd knots_expected{2};
  knots_expected << 0, T - 1;
  ASSERT_TRUE(expectEigenNear(p.knots, knots_expected, 1e-15));
}

TEST(ParameterizationDirectConstructor, givenNcEqualsT_deg1_FormsKnotsCorrectly)
{
  // Maximum control points for linearInterp: one node per step (no reduction)
  const int T{5}, nc{5}, deg{1};
  const ampc::Parameterization p{T, deg, nc};

  ASSERT_EQ(p.knots.size(), nc + deg + 1);

  VectorXd knots_expected{7};
  knots_expected << 0, 0, 1, 2, 3, 4, 4;
  ASSERT_TRUE(expectEigenNear(p.knots, knots_expected, 1e-15));
}

TEST(ParameterizationStreamOperator, givenParameterization_FormatsSummary)
{
  const ampc::Parameterization p{5, 1, 3};
  std::ostringstream oss;
  oss << p;

  EXPECT_EQ(oss.str(),
            "Parameterization(horizon_steps=5, degree=1, num_control_points=3, "
            "knots=[0, 0, 2, 4, 4])");
}

// ---- Methods ---------------------------------------------------------------

TEST(Parameterization, givenControlPoints_EvaluatesInputTrajCorrectly)
{
  const int T{10}, m{1}, nc{4};
  const Vector4d ctrls{0, 1, -1, 0};

  const affine_mpc::Parameterization p0{T, 0, nc};
  const affine_mpc::Parameterization p1{T, 1, nc};
  const affine_mpc::Parameterization p2{T, 2, nc};

  const auto u_traj0{p0.evaluate(m, ctrls)};
  const auto u_traj1{p1.evaluate(m, ctrls)};
  const auto u_traj2{p2.evaluate(m, ctrls)};

  std::cout << u_traj0.transpose() << std::endl;

  VectorXd u_traj0_expected{m * T}, u_traj1_expected{m * T},
      u_traj2_expected{m * T};
  u_traj0_expected << 0, 0, 0, 1, 1, -1, -1, 0, 0, 0;
  u_traj1_expected << 0, 0.33333333, 0.66666666, 1, 0.33333333, -0.33333333, -1,
      -0.66666666, -0.33333333, 0;
  u_traj2_expected << 0, 0.34567901, 0.49382716, 0.44444444, 0.19753086,
      -0.19753086, -0.44444444, -0.49382716, -0.34567901, 0;

  expectEigenNear(u_traj0, u_traj0_expected, 1e-8);
  expectEigenNear(u_traj1, u_traj1_expected, 1e-8);
  expectEigenNear(u_traj2, u_traj2_expected, 1e-8);
}

// ---- Direct constructor (custom knots) — validation paths ------------------

TEST(ParameterizationCustomKnotsConstructor,
     givenValidKnots_StoresKnotsCorrectly)
{
  const int T{10}, nc{5}, deg{2};
  VectorXd knots{8};
  knots << 0, 0, 0, 3, 6, 9, 9, 9;
  const ampc::Parameterization p{T, deg, knots};

  ASSERT_TRUE(expectEigenNear(p.knots, knots, 1e-15));
  ASSERT_EQ(p.horizon_steps, T);
  ASSERT_EQ(p.num_control_points, nc);
  ASSERT_EQ(p.degree, deg);
}

TEST(ParameterizationCustomKnotsConstructor, givenWrongKnotsSize_Throws)
{
  const int T{6}, deg{2};
  VectorXd knots{10}; // expected 8 = nc + deg + 1
  knots << 0, 0, 0, 1, 2, 3, 4, 5, 5, 5;
  expectInvalidArgumentWithMessage(
      [&]() { ampc::Parameterization{T, deg, knots}; },
      "Size of knots can not exceed horizon_steps+degree+1");
}

TEST(ParameterizationCustomKnotsConstructor, givenDecreasingKnots_Throws)
{
  const int T{10}, deg{2};
  VectorXd knots{8};
  knots << 0, 0, 0, 6, 3, 9, 9, 9; // 6 > 3: non-decreasing violated
  expectInvalidArgumentWithMessage(
      [&]() { ampc::Parameterization{T, deg, knots}; },
      "knots must be non-decreasing");
}

TEST(ParameterizationCustomKnotsConstructor, givenFirstActiveKnotNonZero_Throws)
{
  // knots[degree] must equal 0
  const int T{10}, deg{2};
  VectorXd knots{8};
  knots << 0, 0, 1, 3, 6, 9, 9, 9; // knots[2] != 0
  expectInvalidArgumentWithMessage(
      [&]() { ampc::Parameterization{T, deg, knots}; },
      "First active knot must equal zero");
}

TEST(ParameterizationCustomKnotsConstructor,
     givenLastActiveKnotNotHorizonMinus1_Throws)
{
  // knots[nc] must equal T-1
  const int T{10}, deg{2};
  VectorXd knots{8};
  knots << 0, 0, 0, 3, 6, 8, 9, 9; // knots[5]=8 != 9
  expectInvalidArgumentWithMessage(
      [&]() { ampc::Parameterization{T, deg, knots}; },
      "Last active knot must equal horizon_steps-1");
}

// ---- Factory: moveBlocking -------------------------------------------------

TEST(ParameterizationMoveBlocking, givenUniformParams_FormsKnotsCorrectly)
{
  const int T{6}, nc{3};
  const auto p{ampc::Parameterization::moveBlocking(T, nc)};

  ASSERT_EQ(p.horizon_steps, T);
  ASSERT_EQ(p.num_control_points, nc);
  ASSERT_EQ(p.degree, 0);
  ASSERT_EQ(p.knots.size(), nc + 1);

  // degree=0: all knots are active, LinSpaced over [0, T-1]
  // [0, 5/3, 10/3, 5]
  VectorXd knots_expected = ArrayXd::LinSpaced(nc + 1, 0, T - 1);
  ASSERT_TRUE(expectEigenNear(p.knots, knots_expected, 1e-15));
}

TEST(ParameterizationMoveBlocking, givenCustomChangePoints_FormsKnotsCorrectly)
{
  // change_points is the FULL knot vector for degree=0 (must include 0)
  const int T{6}, nc{3};
  VectorXd change_points{nc};
  change_points << 0, 2, 5;
  const auto p{ampc::Parameterization::moveBlocking(T, change_points)};

  ASSERT_EQ(p.horizon_steps, T);
  ASSERT_EQ(p.num_control_points, nc);
  ASSERT_EQ(p.degree, 0);

  VectorXd knots_expected{nc + 1};
  knots_expected << change_points, T - 1;
  ASSERT_TRUE(expectEigenNear(p.knots, knots_expected, 1e-15));
}

TEST(ParameterizationEvaluate, givenWrongControlPointsSize_Throws)
{
  const auto p{ampc::Parameterization::linearInterp(10, 4)};
  const VectorXd ctrls{VectorXd::Zero(10)}; // input_dim * horizon_steps
  expectInvalidArgumentWithMessage(
      [&]() { (void)p.evaluate(1, ctrls); },
      "Size of control_points must be input_dim*num_control_points");
}

TEST(ParameterizationMoveBlocking, givenEmptyChangePoints_Throws)
{
  const VectorXd change_points{0};
  expectInvalidArgumentWithMessage(
      [&]() { ampc::Parameterization::moveBlocking(6, change_points); },
      "change_points can not be empty");
}

TEST(ParameterizationMoveBlocking, givenChangePointsFirstNotZero_Throws)
{
  const int T{6};
  VectorXd change_points{4};
  change_points << 1, 2, 4, 5; // first must be 0
  expectInvalidArgumentWithMessage(
      [&]() { ampc::Parameterization::moveBlocking(T, change_points); },
      "First active knot must equal zero");
}

TEST(ParameterizationMoveBlocking,
     givenChangePointsLastAboveHorizonMinus1_Throws)
{
  const int T{6};
  VectorXd change_points{4};
  change_points << 0, 2, 4, 6; // last must be T-1=5
  expectInvalidArgumentWithMessage(
      [&]() { ampc::Parameterization::moveBlocking(T, change_points); },
      "change_points must be less than or equal to horizon_steps - 1");
}

TEST(ParameterizationMoveBlocking,
     givenCustomChangePoints_SplineEvaluatesProperly)
{
  using Spline1d = Spline<double, 1>;
  const int T{10};
  const Vector3d controls{0, 1, -1}, change_pts{0, 2, 5};

  VectorXd vals{T}, vals_expected{T};
  // change at idxs 0, 2, & 5
  // mostly verifying that final knot at T-1 doesn't change last value
  vals_expected(seq(0, 2)).setConstant(controls(0));
  vals_expected(seq(2, 5)).setConstant(controls(1));
  vals_expected(seq(5, T - 1)).setConstant(controls(2));

  const auto p{ampc::Parameterization::moveBlocking(T, change_pts)};
  const Spline1d spline{p.knots, controls};
  for (int k{0}; k < T; ++k) {
    const double t = k;
    vals(k) = spline(t)(0);
  }

  ASSERT_TRUE(expectEigenNear(vals, vals_expected, 1e-15));
}

TEST(ParameterizationMoveBlocking,
     givenLenHorizonCustomChangePoints_SplineEvaluatesProperly)
{
  using Spline1d = Spline<double, 1>;
  const int T{3};
  const Vector3d controls{0, 1, -1}, change_pts{0, 1, 2};

  VectorXd vals{T}, vals_expected{T};
  vals_expected << controls;

  const auto p{ampc::Parameterization::moveBlocking(T, change_pts)};
  const Spline1d spline{p.knots, controls};
  for (int k{0}; k < T; ++k) {
    const double t = k;
    vals(k) = spline(t)(0);
  }

  ASSERT_TRUE(expectEigenNear(vals, vals_expected, 1e-15));
}

TEST(ParameterizationMoveBlocking, givenEveryStepChangePoints_OneInputPerStep)
{
  // change points 0, 1, ..., T-1 give every sample its own control point
  const int T{5};
  const VectorXd change_pts{VectorXd::LinSpaced(T, 0, T - 1)};
  const auto p{ampc::Parameterization::moveBlocking(T, change_pts)};
  ASSERT_EQ(p.num_control_points, T);

  VectorXd controls{T};
  controls << 10, 20, 30, 40, 50;
  EXPECT_TRUE(expectEigenNear(p.evaluate(1, controls), controls, 0.0));
}

TEST(ParameterizationMoveBlocking, givenFreeStepsThenTail_EvaluatesProperly)
{
  // one free input for each of the first 3 steps, then constant to the end
  const int T{10};
  const Vector3d change_pts{0, 1, 2}, controls{10, 20, 30};
  const auto p{ampc::Parameterization::moveBlocking(T, change_pts)};
  ASSERT_EQ(p.num_control_points, 3);

  VectorXd u_expected{T};
  u_expected << 10, 20, 30, 30, 30, 30, 30, 30, 30, 30;
  EXPECT_TRUE(expectEigenNear(p.evaluate(1, controls), u_expected, 0.0));
}

TEST(ParameterizationMoveBlocking, givenSingleStepHorizon_HasOneControlPoint)
{
  // the first and last active knots are both 0, and there is no interior knot
  const auto p{ampc::Parameterization::moveBlocking(1, 1)};
  ASSERT_EQ(p.num_control_points, 1);
  EXPECT_TRUE(expectEigenNear(p.knots, Vector2d{0.0, 0.0}, 0.0));
  EXPECT_TRUE(expectEigenNear(p.evaluate(1, Matrix<double, 1, 1>{3.0}),
                              Matrix<double, 1, 1>{3.0}, 0.0));
}

TEST(ParameterizationMoveBlocking, givenRepeatedChangePoint_Throws)
{
  // the block [2, 2) is empty
  const int T{5};
  const Vector3d change_points{0, 2, 2};
  expectInvalidArgumentWithMessage(
      [&]() { ampc::Parameterization::moveBlocking(T, change_points); },
      "control point 1 has no sample of its own");
}

// ---- Factory: linearInterp -------------------------------------------------

TEST(ParameterizationLinearInterp, givenUniformParams_FormsKnotsCorrectly)
{
  const int T{6}, nc{3};
  const ampc::Parameterization p{ampc::Parameterization::linearInterp(T, nc)};

  ASSERT_EQ(p.horizon_steps, T);
  ASSERT_EQ(p.num_control_points, nc);
  ASSERT_EQ(p.degree, 1);
  ASSERT_EQ(p.knots.size(), nc + 2);

  // knots = [0, 0, 2.5, 5, 5]
  VectorXd knots_expected{5};
  knots_expected << 0, 0, 2.5, 5, 5;
  ASSERT_TRUE(expectEigenNear(p.knots, knots_expected, 1e-15));
}

TEST(ParameterizationLinearInterp, givenCustomChangePoints_FormsKnotsCorrectly)
{
  // change_points are the interior nodes; fist must be 0, last must be T-1.
  const int T{10};
  VectorXd change_points{3};
  change_points << 0, 2, 9;
  const ampc::Parameterization p{
      ampc::Parameterization::linearInterp(T, change_points)};

  // nc = change_points.size() = 3, knots = [0, 0, 2, 9, 9]
  ASSERT_EQ(p.num_control_points, 3);
  ASSERT_EQ(p.degree, 1);
  ASSERT_EQ(p.knots.size(), 5);

  VectorXd knots_expected{5};
  knots_expected << 0, 0, 2, 9, 9;
  ASSERT_TRUE(expectEigenNear(p.knots, knots_expected, 1e-15));
}

TEST(ParameterizationLinearInterp, givenTooManyChangePoints_Throws)
{
  const int T{5};
  VectorXd change_points{6};
  change_points << 0, 1, 2, 3, 3.5, 4;
  expectInvalidArgumentWithMessage(
      [&]() { ampc::Parameterization::linearInterp(T, change_points); },
      "Size of endpoints can not exceed horizon_steps");
}

TEST(ParameterizationLinearInterp, givenChangePointsFirstNotZero_Throws)
{
  const int T{10};
  VectorXd change_points{3};
  change_points << 1, 4, 9; // first must be 0
  expectInvalidArgumentWithMessage(
      [&]() { ampc::Parameterization::linearInterp(T, change_points); },
      "First active knot must equal zero");
}

TEST(ParameterizationLinearInterp, givenChangePointsLastNotHorizonMinus1_Throws)
{
  const int T{10};
  VectorXd change_points{3};
  change_points << 0, 4, 7; // last must be T-1=9 (becomes knots[nc])
  expectInvalidArgumentWithMessage(
      [&]() { ampc::Parameterization::linearInterp(T, change_points); },
      "Last active knot must equal horizon_steps-1");
}

TEST(ParameterizationLinearInterp, givenChangePointsSizeEqualsT_Succeeds)
{
  // Boundary of the throw condition: size == T is valid (no reduction)
  const int T{5};
  VectorXd change_points{T};
  change_points << 0, 1, 2, 3, 4;
  ASSERT_NO_THROW(ampc::Parameterization::linearInterp(T, change_points));
}

TEST(ParameterizationLinearInterp, givenRepeatedEndpoint_JumpsAtKnot)
{
  // a double knot at 2 makes the piecewise linear input discontinuous there:
  // c1 at the end of the first segment (reached only in the limit) and c2 at
  // the start of the second
  const int T{5};
  const Vector4d endpoints{0, 2, 2, 4};
  const auto p{ampc::Parameterization::linearInterp(T, endpoints)};
  ASSERT_EQ(p.num_control_points, 4);
  const Vector4d ctrls{0, 2, 10, 6};
  VectorXd u_expected{T};
  u_expected << 0, 1, 10, 8, 6;
  EXPECT_TRUE(expectEigenNear(p.evaluate(1, ctrls), u_expected, 1e-15));
}

// ---- Factory: bspline ------------------------------------------------------

TEST(ParameterizationBspline, givenUniformParams_FormsKnotsCorrectly)
{
  const int T{10}, nc{5}, deg{2};
  const ampc::Parameterization p{ampc::Parameterization::bspline(T, deg, nc)};

  ASSERT_EQ(p.horizon_steps, T);
  ASSERT_EQ(p.num_control_points, nc);
  ASSERT_EQ(p.degree, deg);
  ASSERT_EQ(p.knots.size(), nc + deg + 1);

  VectorXd knots_expected{8};
  knots_expected << 0, 0, 0, 3, 6, 9, 9, 9;
  ASSERT_TRUE(expectEigenNear(p.knots, knots_expected, 1e-15));
}

TEST(ParameterizationBspline, givenCustomActiveKnots_FormsKnotsCorrectly)
{
  // nc=5, deg=2: num_active = nc - deg + 1 = 4 internal knots expected
  const int T{10}, deg{2};
  VectorXd active_knots{4};
  active_knots << 0, 2, 5, 9; // first must be 0, last must be T-1 = 9
  const ampc::Parameterization p{
      ampc::Parameterization::bspline(T, deg, active_knots)};

  VectorXd knots_expected{8};
  knots_expected << 0, 0, 0, 2, 5, 9, 9, 9;
  ASSERT_TRUE(expectEigenNear(p.knots, knots_expected, 1e-15));
}

TEST(ParameterizationBspline, givenWrongActiveKnotsCount_Throws)
{
  const int T{5}, deg{2};
  // expected nc - deg + 1 = 4 active knots, providing 3
  VectorXd active_knots{9};
  active_knots << 0, 0, 0, 1, 2, 3, 4, 4, 4;
  expectInvalidArgumentWithMessage(
      [&]() { ampc::Parameterization::bspline(T, deg, active_knots); },
      "Size of knots can not exceed horizon_steps+degree+1");
}

TEST(ParameterizationBspline, givenActiveKnotsOutOfRange_Throws)
{
  const int T{10}, deg{2};
  // nc - deg + 1 = 4 active knots required
  VectorXd active_knots{4};

  // first active knot must be 0
  active_knots << 1, 2, 5, 9;
  expectInvalidArgumentWithMessage(
      [&]() { ampc::Parameterization::bspline(T, deg, active_knots); },
      "First active knot must equal zero");

  // last active knot must be T-1
  active_knots << 0, 2, 5, 7;
  expectInvalidArgumentWithMessage(
      [&]() { ampc::Parameterization::bspline(T, deg, active_knots); },
      "Last active knot must equal horizon_steps-1");
}

// ---- zero-influence control points -----------------------------------------

TEST(ParameterizationBspline, givenControlPointWithoutInfluence_Throws)
{
  // the hat function of control point 2 is nonzero only on (3.2, 3.22), which
  // contains no sample
  VectorXd active_knots{5};
  active_knots << 0, 3.2, 3.21, 3.22, 9;
  expectInvalidArgumentWithMessage(
      [&]() { ampc::Parameterization::bspline(10, 1, active_knots); },
      "control point 2 has no sample of its own. Its basis function, with "
      "support [3.2, 3.22], is nonzero at no sample");
}

TEST(ParameterizationMoveBlocking, givenBlockWithoutSample_Throws)
{
  // block [3.2, 3.5) contains no integer sample
  const Vector3d change_points{0, 3.2, 3.5};
  expectInvalidArgumentWithMessage(
      [&]() { ampc::Parameterization::moveBlocking(10, change_points); },
      "control point 1 has no sample of its own");
}

TEST(ParameterizationBspline, givenSubSampleKnotsWithInfluence_Succeeds)
{
  // knots closer than one sample are fine as long as every control point
  // influences some sample
  VectorXd active_knots{4};
  active_knots << 0, 0.5, 1.2, 9;
  EXPECT_NO_THROW(ampc::Parameterization::bspline(10, 1, active_knots));
  EXPECT_NO_THROW(ampc::Parameterization::bspline(10, 2, active_knots));

  const Vector3d change_points{0, 0.5, 1.2};
  EXPECT_NO_THROW(ampc::Parameterization::moveBlocking(10, change_points));
}

// ---- end-knot tolerance ----------------------------------------------------

TEST(ParameterizationBspline, givenEndKnotsWithRoundOff_SnapsToBoundaries)
{
  // e.g. knots computed by scaling fractions to [0, T-1]
  const int T{10}, deg{2};
  const Vector3d active_knots{1e-13, 4.0, T - 1 - 1e-13};
  const auto p{ampc::Parameterization::bspline(T, deg, active_knots)};

  VectorXd knots_expected{7};
  knots_expected << 0, 0, 0, 4, 9, 9, 9;
  EXPECT_TRUE(expectEigenNear(p.knots, knots_expected, 0.0));
}

TEST(ParameterizationDirectConstructor,
     givenEndKnotsWithRoundOff_SnapsToBoundaries)
{
  const int T{10}, deg{1};
  VectorXd knots{5};
  knots << -1e-13, 1e-13, 4.0, T - 1 - 1e-13, T - 1 + 1e-13;
  const ampc::Parameterization p{T, deg, knots};

  VectorXd knots_expected{5};
  knots_expected << 0, 0, 4, 9, 9;
  EXPECT_TRUE(expectEigenNear(p.knots, knots_expected, 0.0));
}

TEST(ParameterizationBspline, givenEndKnotOutsideTolerance_PrintsFullPrecision)
{
  const int T{10}, deg{2};
  const Vector3d active_knots{0.0, 4.0, T - 1 - 1e-6};
  // the default stream precision would print "Got 9"
  expectInvalidArgumentWithMessage(
      [&]() { ampc::Parameterization::bspline(T, deg, active_knots); },
      "Got 8.999999");
}

// ---- banded basis ----------------------------------------------------------

TEST(ParameterizationBand, givenBspline_StoresBandOfBasisFunctions)
{
  const int T{10}, nc{5}, deg{2};
  const auto p{ampc::Parameterization::bspline(T, deg, nc)};

  ASSERT_EQ(p.band_width, deg + 1);
  VectorXi start_expected{T};
  start_expected << 0, 0, 0, 1, 1, 1, 2, 2, 2, 2;
  EXPECT_TRUE(expectEigenNear(p.band_start, start_expected, 0.0));
  ASSERT_EQ(p.band_weights.rows(), deg + 1);
  ASSERT_EQ(p.band_weights.cols(), T);

  using Spline1d = Spline<double, 1>;
  const VectorXd ctrls{{0, 1, 1, -1, 0}};
  const Spline1d spline{p.knots, ctrls.transpose()};
  for (int k{0}; k < T; ++k) {
    const double t = k;
    const RowVectorXd weights_expected{
        Spline1d::BasisFunctions(t, deg, p.knots)};
    EXPECT_TRUE(expectEigenNear(p.band_weights.col(k).transpose(),
                                weights_expected, 0.0));
    const double u{p.band_weights.col(k).dot(
        ctrls.segment(p.band_start(k), p.band_width))};
    EXPECT_DOUBLE_EQ(u, spline(t)(0));
  }
}

TEST(ParameterizationBand, givenAnyParameterization_BandReproducesEvaluate)
{
  const int m{2};
  for (const auto& p : {ampc::Parameterization::moveBlocking(12, 4),
                        ampc::Parameterization::linearInterp(12, 5),
                        ampc::Parameterization::bspline(12, 3, 7),
                        ampc::Parameterization{12, 2, 12}}) {
    ASSERT_EQ(p.band_width, p.degree + 1);
    const VectorXd ctrls{VectorXd::LinSpaced(m * p.num_control_points, -1, 2)};
    const VectorXd u_traj{p.evaluate(m, ctrls)};
    const Map<const MatrixXd> c{ctrls.data(), m, p.num_control_points};
    for (int k{0}; k < p.horizon_steps; ++k) {
      ASSERT_LE(p.band_start(k) + p.band_width, p.num_control_points);
      const VectorXd uk{c.middleCols(p.band_start(k), p.band_width)
                        * p.band_weights.col(k)};
      EXPECT_TRUE(expectEigenNear(u_traj.segment(m * k, m), uk, 0.0));
    }
  }
}

// ---- Factory option: knot_ratio --------------------------------------------

TEST(ParameterizationKnotRatio, givenRatio_SpacesActiveKnotsGeometrically)
{
  // interval lengths 1, g, g^2, g^3 scaled to span [0, T-1]
  const int T{20}, deg{2}, nc{6};
  const double g{1.5};
  const auto p{ampc::Parameterization::bspline(T, deg, nc, g)};
  ASSERT_EQ(p.degree, deg);
  ASSERT_EQ(p.num_control_points, nc);

  const double scale{(T - 1) / (1 + g + g * g + g * g * g)};
  VectorXd knots_expected{nc + deg + 1};
  knots_expected << 0, 0, 0, scale, scale * (1 + g), scale * (1 + g + g * g),
      T - 1, T - 1, T - 1;
  EXPECT_TRUE(expectEigenNear(p.knots, knots_expected, 1e-12));
  // the last active knot is exact, not a round-off away from T-1
  EXPECT_EQ(p.knots(nc), T - 1);
}

TEST(ParameterizationKnotRatio, givenRatioBelowOne_MoveBlockingShrinksBlocks)
{
  const int T{20}, nc{4};
  const auto p{ampc::Parameterization::moveBlocking(T, nc, 0.5)};
  ASSERT_EQ(p.degree, 0);
  // blocks 8/15, 4/15, 2/15, 1/15 of 19
  VectorXd knots_expected{nc + 1};
  knots_expected << 0, 19 * 8.0 / 15, 19 * 12.0 / 15, 19 * 14.0 / 15, 19;
  EXPECT_TRUE(expectEigenNear(p.knots, knots_expected, 1e-12));
}

TEST(ParameterizationKnotRatio, givenRatio_LinearInterpSpacesEndpoints)
{
  const int T{15}, nc{4};
  const double g{2.0};
  const auto p{ampc::Parameterization::linearInterp(T, nc, g)};
  ASSERT_EQ(p.degree, 1);
  // segments 1, 2, 4 sevenths of 14
  VectorXd knots_expected{nc + 2};
  knots_expected << 0, 0, 2, 6, 14, 14;
  EXPECT_TRUE(expectEigenNear(p.knots, knots_expected, 1e-12));
}

TEST(ParameterizationKnotRatio, givenRatioOne_MatchesUniformKnotsExactly)
{
  const int T{15};
  using P = ampc::Parameterization;
  EXPECT_EQ(P::bspline(T, 3, 8, 1.0).knots,
            P::makeUniformClampedKnots(T, 3, 8));
  EXPECT_EQ(P::moveBlocking(T, 4, 1.0).knots,
            P::makeUniformClampedKnots(T, 0, 4));
  EXPECT_EQ(P::linearInterp(T, 5, 1.0).knots,
            P::makeUniformClampedKnots(T, 1, 5));
}

TEST(ParameterizationKnotRatio, givenRatio_MakeClampedKnotsMatchesFactories)
{
  const int T{20};
  const double g{1.4};
  using P = ampc::Parameterization;
  EXPECT_EQ(P::makeClampedKnots(T, 2, 6, g), P::bspline(T, 2, 6, g).knots);
  EXPECT_EQ(P::makeClampedKnots(T, 0, 5, g), P::moveBlocking(T, 5, g).knots);
  EXPECT_EQ(P::makeClampedKnots(T, 1, 5, g), P::linearInterp(T, 5, g).knots);
  EXPECT_EQ(P::makeClampedKnots(T, 3, 8), P::makeUniformClampedKnots(T, 3, 8));
  expectInvalidArgumentWithMessage(
      [] { P::makeClampedKnots(20, 1, 5, 10.0); },
      "[Parameterization::makeClampedKnots] knot_ratio = 10 gives knot "
      "intervals too short for the samples");
}

TEST(ParameterizationKnotRatio, givenInvalidArguments_Throws)
{
  using P = ampc::Parameterization;
  expectInvalidArgumentWithMessage([] { P::bspline(20, 1, 5, 0.0); },
                                   "knot_ratio must be positive and finite");
  expectInvalidArgumentWithMessage(
      [] { P::moveBlocking(20, 5, std::numeric_limits<double>::infinity()); },
      "knot_ratio must be positive and finite");
  // knots 0, 0.0171, 0.188, 1.90, 19: the hat of control point 1 is nonzero
  // only on (0, 0.188), which contains no sample
  expectInvalidArgumentWithMessage([] { P::bspline(20, 1, 5, 10.0); },
                                   "control point 1 has no sample of its own");
  expectInvalidArgumentWithMessage(
      [] { P::bspline(20, 1, 5, 10.0); },
      "Use a knot_ratio closer to 1 or fewer control points");
  // ratio below 1: the last intervals are the shortest
  expectInvalidArgumentWithMessage([] { P::linearInterp(20, 5, 0.1); },
                                   "control point 3 has no sample of its own");
  expectInvalidArgumentWithMessage(
      [] { P::bspline(20, 2, 2, 1.5); },
      "must have at least degree+1 control points");
}

// ---- Unclamped knots -------------------------------------------------------

TEST(ParameterizationUnclamped, givenUnclampedKnots_EvaluatesLikeSpline)
{
  // uniform quadratic with knots extending beyond [0, T-1] on both sides
  using Spline1d = Spline<double, 1>;
  const int T{10}, deg{2};
  VectorXd knots{8};
  knots << -6, -3, 0, 3, 6, 9, 12, 15;
  const ampc::Parameterization p{T, deg, knots};
  ASSERT_EQ(p.num_control_points, 5);
  EXPECT_EQ(p.knots, knots);

  VectorXd controls{5};
  controls << 1, -2, 0.5, 3, -1;
  const Spline1d spline{p.knots, controls.transpose()};
  const VectorXd u{p.evaluate(1, controls)};
  for (int k{0}; k < T; ++k)
    EXPECT_NEAR(u(k), spline(k)(0), 1e-14) << "k = " << k;

  // unlike a clamped spline, the first input is not the first control point
  EXPECT_NEAR(u(0), 0.5 * (controls(0) + controls(1)), 1e-14);

  // partition of unity still holds on [0, T-1]
  EXPECT_TRUE(expectEigenNear(p.evaluate(1, VectorXd::Constant(5, 2.5)),
                              VectorXd::Constant(T, 2.5), 1e-14));
}

// ---- Schoenberg-Whitney rank check -----------------------------------------

namespace {

// Independent Cox-de Boor evaluation of basis function i of degree p at t,
// with zero-denominator terms dropped. Degree 0 is right-continuous, except
// with left_limit, which takes the limit from the left (the spline is
// evaluated at the last active knot horizon_steps-1 from the left).
double coxDeBoor(const VectorXd& knots, int i, int p, double t, bool left_limit)
{
  if (p == 0) {
    if (left_limit)
      return (knots(i) < t && t <= knots(i + 1)) ? 1.0 : 0.0;
    return (knots(i) <= t && t < knots(i + 1)) ? 1.0 : 0.0;
  }
  double value{0.0};
  const double d1{knots(i + p) - knots(i)};
  const double d2{knots(i + p + 1) - knots(i + 1)};
  if (d1 > 0.0)
    value += (t - knots(i)) / d1 * coxDeBoor(knots, i, p - 1, t, left_limit);
  if (d2 > 0.0)
    value += (knots(i + p + 1) - t) / d2
             * coxDeBoor(knots, i + 1, p - 1, t, left_limit);
  return value;
}

// Dense T x num_control_points basis sampled at k = 0, ..., T-1 by Cox-de
// Boor. Degree 0 follows the library convention that the last control point
// owns the last sample (so a change point at T-1 is a one-sample block).
MatrixXd coxDeBoorBasis(int T, int p, const VectorXd& knots)
{
  const int n{static_cast<int>(knots.size()) - p - 1};
  MatrixXd basis{MatrixXd::Zero(T, n)};
  for (int k{0}; k < T; ++k) {
    if (p == 0 && k == T - 1) {
      basis(k, n - 1) = 1.0;
      continue;
    }
    for (int i{0}; i < n; ++i)
      basis(k, i) = coxDeBoor(knots, i, p, k, p > 0 && k == T - 1);
  }
  return basis;
}

// Dense basis from the stored band
MatrixXd bandBasis(const ampc::Parameterization& p)
{
  MatrixXd basis{MatrixXd::Zero(p.horizon_steps, p.num_control_points)};
  for (int k{0}; k < p.horizon_steps; ++k)
    basis.row(k).segment(p.band_start(k), p.band_width) =
        p.band_weights.col(k).transpose();
  return basis;
}

// Smallest over largest singular value (0 when there are more columns than
// rows, which can not have full column rank)
double inverseCondition(const MatrixXd& basis)
{
  if (basis.cols() > basis.rows())
    return 0.0;
  const VectorXd sv{JacobiSVD<MatrixXd>{basis}.singularValues()};
  return sv(sv.size() - 1) / sv(0);
}

// Clamped knot vector from active knots
VectorXd clampedKnots(int T, int p, const VectorXd& active)
{
  VectorXd knots{active.size() + 2 * p};
  knots << VectorXd::Zero(p), active, VectorXd::Constant(p, T - 1.0);
  return knots;
}

} // namespace

TEST(ParameterizationRank, givenRepeatedInteriorKnots_EigenMatchesCoxDeBoor)
{
  // every multiplicity that keeps the spline defined (2 to degree+1), at and
  // between samples, next to simple knots
  const int T{14};
  int num_accepted{0};
  for (int p{1}; p <= 3; ++p) {
    for (int mult{2}; mult <= p + 1; ++mult) {
      for (const double t : {5.0, 5.5, 5.25, 4.0 + 1e-9}) {
        VectorXd active{4 + mult};
        active << 0.0, 2.5, VectorXd::Constant(mult, t), 9.0, T - 1.0;
        const auto param{ampc::Parameterization::bspline(T, p, active)};
        ++num_accepted;
        const MatrixXd expected{coxDeBoorBasis(T, p, param.knots)};
        EXPECT_TRUE(expectEigenNear(bandBasis(param), expected, 1e-14))
            << "p = " << p << ", mult = " << mult << ", t = " << t;
        EXPECT_GT(inverseCondition(expected), 1e-6);

        const VectorXd ctrls{
            VectorXd::LinSpaced(param.num_control_points, -1, 3).array().sin()};
        EXPECT_TRUE(
            expectEigenNear(param.evaluate(1, ctrls), expected * ctrls, 1e-14));
      }
    }
  }
  EXPECT_EQ(num_accepted, 6 * 4);
}

TEST(ParameterizationRank, givenKnotMultiplicityAboveDegreePlusOne_Throws)
{
  // a basis function whose support is a single point is zero everywhere
  for (int p{0}; p <= 3; ++p) {
    VectorXd active{5 + p};
    active << 0.0, 3.0, VectorXd::Constant(p + 2, 5.5), 9.0;
    expectInvalidArgumentWithMessage(
        [&] { ampc::Parameterization::bspline(10, p, active); },
        "has no sample of its own");
  }
}

TEST(ParameterizationRank, givenInteriorKnotAtLastActiveKnot_Throws)
{
  // the spline is evaluated at T-1 from the left, so the last basis function
  // (support [9, 9]) is zero at every sample
  VectorXd active{4};
  active << 0, 4, 9, 9;
  expectInvalidArgumentWithMessage(
      [&] { ampc::Parameterization::bspline(10, 1, active); },
      "control point 3 has no sample of its own");
  expectInvalidArgumentWithMessage(
      [&] { ampc::Parameterization::bspline(10, 2, active); },
      "control point 4 has no sample of its own");
  // degree 0: a change point at T-1 is a one-sample block (see moveBlocking)
  EXPECT_NO_THROW(ampc::Parameterization::bspline(10, 0, active));
}

TEST(ParameterizationRank, givenInteriorKnotAtFirstActiveKnot_Throws)
{
  // the spline is evaluated from the right, so the first basis function
  // (support [0, 0], or [-1, 0] unclamped) is zero at every sample
  const Vector3d change_points{0, 0, 3};
  expectInvalidArgumentWithMessage(
      [&] { ampc::Parameterization::moveBlocking(10, change_points); },
      "control point 0 has no sample of its own");
  VectorXd active{4};
  active << 0, 0, 4, 9;
  for (int p{1}; p <= 3; ++p)
    expectInvalidArgumentWithMessage(
        [&] { ampc::Parameterization::bspline(10, p, active); },
        "control point 0 has no sample of its own");
  VectorXd knots{6};
  knots << -1, 0, 0, 4, 9, 10;
  expectInvalidArgumentWithMessage(
      [&] { ampc::Parameterization{10, 1, knots}; },
      "interior knot equals the first active knot");
}

TEST(ParameterizationRank, givenTwoColumnsSharingOneSample_Throws)
{
  // every basis function is nonzero at some sample (accepted before the rank
  // check), but the hats of control points 3 and 4 are both nonzero only at
  // k = 4, so they are linearly dependent on the samples
  const int T{10}, p{1};
  VectorXd active{7};
  active << 0, 2, 3.5, 3.9, 4.2, 4.6, 9;
  const MatrixXd basis{coxDeBoorBasis(T, p, clampedKnots(T, p, active))};
  for (int j{0}; j < basis.cols(); ++j)
    EXPECT_GT(basis.col(j).cwiseAbs().maxCoeff(), 0.0) << "column " << j;
  EXPECT_LT(inverseCondition(basis), 1e-14);

  expectInvalidArgumentWithMessage(
      [&] { ampc::Parameterization::bspline(T, p, active); },
      "control point 4 has no sample of its own. Its basis function, with "
      "support [3.9, 4.6], is nonzero at sample k = 4 only, but it needs a "
      "sample k > 4 (control points 0 to 3 need samples up to k = 4)");
}

TEST(ParameterizationRank, givenRandomKnots_AcceptedIffBasisHasFullRank)
{
  // random degrees, horizons, and active knots, on a quarter-sample grid (with
  // repeats) and continuous; the check accepts exactly the knot vectors whose
  // sampled basis has full column rank
  std::mt19937 gen{2024};
  int num_accepted{0}, num_rejected{0}, num_repeated{0};
  double min_accepted{1.0}, max_rejected{0.0};
  for (int trial{0}; trial < 4000; ++trial) {
    std::uniform_int_distribution<int> T_dist{2, 25}, p_dist{0, 3};
    const int T{T_dist(gen)}, p{p_dist(gen)};
    std::uniform_int_distribution<int> num_dist{0, T};
    const int num_interior{num_dist(gen)};
    std::uniform_real_distribution<double> uniform{0.0, T - 1.0};
    std::uniform_int_distribution<int> grid{0, 4 * (T - 1)};
    std::vector<double> interior(num_interior);
    for (double& t : interior)
      t = trial % 2 == 0 ? 0.25 * grid(gen) : uniform(gen);
    std::sort(interior.begin(), interior.end());
    VectorXd active{num_interior + 2};
    active << 0.0, Map<const VectorXd>{interior.data(), num_interior}, T - 1.0;
    const VectorXd knots{clampedKnots(T, p, active)};
    if (knots.size() - p - 1 > T)
      continue; // more control points than samples: rejected by the counts
    const MatrixXd expected{coxDeBoorBasis(T, p, knots)};
    const double inv_cond{inverseCondition(expected)};

    try {
      const auto param{ampc::Parameterization::bspline(T, p, active)};
      ++num_accepted;
      for (Index i{p + 1}; i + 1 < knots.size() - p; ++i)
        num_repeated += knots(i) == knots(i - 1);
      ASSERT_TRUE(expectEigenNear(bandBasis(param), expected, 1e-13))
          << "trial " << trial;
      min_accepted = std::min(min_accepted, inv_cond);
      ASSERT_GT(inv_cond, 1e-12) << "trial " << trial;
    } catch (const std::invalid_argument& e) {
      ASSERT_NE(std::string{e.what()}.find("full column rank"),
                std::string::npos)
          << e.what();
      ++num_rejected;
      max_rejected = std::max(max_rejected, inv_cond);
      ASSERT_LT(inv_cond, 1e-14) << "trial " << trial;
    }
  }
  std::cout << "accepted " << num_accepted << " (with " << num_repeated
            << " repeated knots), rejected " << num_rejected
            << ", smallest accepted inverse condition " << min_accepted
            << ", largest rejected " << max_rejected << "\n";
  EXPECT_GT(num_accepted, 1000);
  EXPECT_GT(num_rejected, 500);
  EXPECT_GT(num_repeated, 50);
}

TEST(ParameterizationRank, givenSubSampleGeometricInterval_Accepts)
{
  // first interval 19 / (1 + 3 + 9 + 27) = 0.475 samples, but every control
  // point still has its own sample
  const auto p{ampc::Parameterization::bspline(20, 1, 5, 3.0)};
  EXPECT_LT(p.knots(2) - p.knots(1), 1.0);
  EXPECT_GT(inverseCondition(bandBasis(p)), 1e-3);
  EXPECT_EQ(ampc::Parameterization::makeClampedKnots(20, 1, 5, 3.0), p.knots);
  EXPECT_NO_THROW(ampc::Parameterization::moveBlocking(20, 4, 3.0));
  EXPECT_NO_THROW(ampc::Parameterization::linearInterp(20, 5, 1.0 / 3.0));
}

// ---- custom basis ----------------------------------------------------------

TEST(ParameterizationFromBasis, givenBasis_StoresDenseBand)
{
  const int T{10}, m{2};
  const auto bspline{ampc::Parameterization::bspline(T, 2, 5)};
  const MatrixXd basis{bandBasis(bspline)};
  const auto param{ampc::Parameterization::fromBasis(T, basis)};

  EXPECT_EQ(param.type, ampc::Parameterization::Type::Custom);
  EXPECT_EQ(bspline.type, ampc::Parameterization::Type::BSpline);
  EXPECT_EQ(param.horizon_steps, T);
  EXPECT_EQ(param.num_control_points, 5);
  EXPECT_EQ(param.degree, -1);
  EXPECT_EQ(param.knots.size(), 0);
  EXPECT_EQ(param.band_width, 5);
  EXPECT_TRUE(param.band_start.isZero());
  EXPECT_TRUE(expectEigenNear(param.band_weights, basis.transpose(), 0.0));
  EXPECT_TRUE(expectEigenNear(bandBasis(param), basis, 0.0));

  const VectorXd ctrls{VectorXd::LinSpaced(m * 5, -1, 2)};
  EXPECT_TRUE(expectEigenNear(param.evaluate(m, ctrls),
                              bspline.evaluate(m, ctrls), 1e-15));
}

TEST(ParameterizationFromBasis, givenInvalidShape_Throws)
{
  using P = ampc::Parameterization;
  expectInvalidArgumentWithMessage([] { P::fromBasis(0, MatrixXd{0, 1}); },
                                   "horizon_steps must be positive");
  expectInvalidArgumentWithMessage(
      [] { P::fromBasis(5, MatrixXd::Identity(4, 2)); },
      "basis must have horizon_steps = 5 rows (one per sample), got 4");
  expectInvalidArgumentWithMessage([] { P::fromBasis(4, MatrixXd{4, 0}); },
                                   "basis must have at least one column");
  expectInvalidArgumentWithMessage(
      [] { P::fromBasis(2, MatrixXd::Identity(2, 3)); },
      "basis can not have more columns than rows");
}

TEST(ParameterizationFromBasis, givenNonFiniteOrRankDeficientBasis_Throws)
{
  using P = ampc::Parameterization;
  MatrixXd basis{MatrixXd::Identity(4, 2)};
  basis(1, 1) = std::numeric_limits<double>::quiet_NaN();
  expectInvalidArgumentWithMessage([&] { P::fromBasis(4, basis); },
                                   "basis entries must be finite");

  basis.setOnes(); // repeated columns
  expectInvalidArgumentWithMessage([&] { P::fromBasis(4, basis); },
                                   "basis must have full column rank");
  expectInvalidArgumentWithMessage(
      [] { P::fromBasis(4, MatrixXd::Zero(4, 2)); },
      "basis must have full column rank");

  // nearly dependent columns: singular value ratio about 1e-10 < 1e-9
  basis << 1, 1, 1, 1 + 4e-10, 1, 1, 1, 1;
  expectInvalidArgumentWithMessage([&] { P::fromBasis(4, basis); },
                                   "basis must have full column rank");
  // ratio about 1e-8 is accepted
  basis(1, 1) = 1 + 4e-8;
  EXPECT_NO_THROW(P::fromBasis(4, basis));
}

TEST(ParameterizationStreamOperator, givenCustomBasis_FormatsSummary)
{
  std::ostringstream oss;
  oss << ampc::Parameterization::fromBasis(4, MatrixXd::Identity(4, 2));
  EXPECT_EQ(oss.str(), "Parameterization(type=custom, horizon_steps=4, "
                       "num_control_points=2)");
}
