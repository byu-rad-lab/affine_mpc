#include <Eigen/Core>
#include <gtest/gtest.h>

#include "affine_mpc/parameterization.hpp"
#include "utils.hpp"

#include <unsupported/Eigen/Splines>

using namespace Eigen;
namespace ampc = affine_mpc;

// ---- Direct constructor (uniform knots) ------------------------------------

TEST(ParameterizationDirectConstructor, givenInvalidHorizonSteps_Throws)
{
  expectInvalidArgumentWithMessage([&]() { ampc::Parameterization{-1, 3, 0}; },
                                   "horizon_steps must be at least 1");
}

TEST(ParameterizationDirectConstructor, givenInvalidNumControlPoints_Throws)
{
  expectInvalidArgumentWithMessage(
      [&]() { ampc::Parameterization{5, 0, 0}; },
      "num_control_points must be in the range [1, horizon_steps]");
  expectInvalidArgumentWithMessage(
      [&]() { ampc::Parameterization{5, 6, 0}; },
      "num_control_points must be in the range [1, horizon_steps]");
}

TEST(ParameterizationDirectConstructor, givenInvalidDegree_Throws)
{
  expectInvalidArgumentWithMessage(
      [&]() { ampc::Parameterization{5, 3, -1}; },
      "degree must be in the range [0, num_control_points - 1]");
  expectInvalidArgumentWithMessage(
      [&]() { ampc::Parameterization{5, 3, 3}; },
      "degree must be in the range [0, num_control_points - 1]");
}

TEST(ParameterizationDirectConstructor,
     givenMoveBlockingParams_FormsKnotsCorrectly)
{
  // degree=0: no clamped head/tail, all knots are active, LinSpaced over [0,
  // T-1]
  const int T{6}, nc{3}, deg{0};
  const ampc::Parameterization p{T, nc, deg};

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
  const ampc::Parameterization p{T, nc, deg};

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
  const ampc::Parameterization p{T, nc, deg};

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
  const ampc::Parameterization p{T, nc, deg};

  ASSERT_EQ(p.knots.size(), 2);
  VectorXd knots_expected{2};
  knots_expected << 0, T - 1;
  ASSERT_TRUE(expectEigenNear(p.knots, knots_expected, 1e-15));
}

TEST(ParameterizationDirectConstructor, givenNcEqualsT_deg1_FormsKnotsCorrectly)
{
  // Maximum control points for linearInterp: one node per step (no reduction)
  const int T{5}, nc{5}, deg{1};
  const ampc::Parameterization p{T, nc, deg};

  ASSERT_EQ(p.knots.size(), nc + deg + 1);

  VectorXd knots_expected{7};
  knots_expected << 0, 0, 1, 2, 3, 4, 4;
  ASSERT_TRUE(expectEigenNear(p.knots, knots_expected, 1e-15));
}

// ---- Direct constructor (custom knots) — validation paths ------------------

TEST(ParameterizationCustomKnotsConstructor,
     givenValidKnots_StoresKnotsCorrectly)
{
  const int T{10}, nc{5}, deg{2};
  VectorXd knots{8};
  knots << 0, 0, 0, 3, 6, 9, 9, 9;
  const ampc::Parameterization p{T, nc, deg, knots};

  ASSERT_TRUE(expectEigenNear(p.knots, knots, 1e-15));
  ASSERT_EQ(p.horizon_steps, T);
  ASSERT_EQ(p.num_control_points, nc);
  ASSERT_EQ(p.degree, deg);
}

TEST(ParameterizationCustomKnotsConstructor, givenWrongKnotsSize_Throws)
{
  const int T{10}, nc{5}, deg{2};
  VectorXd knots{7}; // expected 8 = nc + deg + 1
  knots << 0, 0, 0, 3, 6, 9, 9;
  expectInvalidArgumentWithMessage(
      [&]() { ampc::Parameterization{T, nc, deg, knots}; },
      "Invalid knots size");
}

TEST(ParameterizationCustomKnotsConstructor, givenDecreasingKnots_Throws)
{
  const int T{10}, nc{5}, deg{2};
  VectorXd knots{8};
  knots << 0, 0, 0, 6, 3, 9, 9, 9; // 6 > 3: non-decreasing violated
  expectInvalidArgumentWithMessage(
      [&]() { ampc::Parameterization{T, nc, deg, knots}; },
      "knots must be non-decreasing");
}

TEST(ParameterizationCustomKnotsConstructor, givenFirstActiveKnotNonZero_Throws)
{
  // knots[degree] must equal 0
  const int T{10}, nc{5}, deg{2};
  VectorXd knots{8};
  knots << 0, 0, 1, 3, 6, 9, 9, 9; // knots[2] != 0
  expectInvalidArgumentWithMessage(
      [&]() { ampc::Parameterization{T, nc, deg, knots}; },
      "First active knot must equal zero");
}

TEST(ParameterizationCustomKnotsConstructor,
     givenLastActiveKnotNotHorizonMinus1_Throws)
{
  // knots[nc] must equal T-1
  const int T{10}, nc{5}, deg{2};
  VectorXd knots{8};
  knots << 0, 0, 0, 3, 6, 8, 9, 9; // knots[5]=8 != 9
  expectInvalidArgumentWithMessage(
      [&]() { ampc::Parameterization{T, nc, deg, knots}; },
      "Last active knot must equal horizon_steps - 1");
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
  const int T{10}, nc{3};
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
  const int T{3}, nc{3};
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

TEST(ParameterizationMoveBlocking, givenRepeatedChangePoint_Throws)
{
  const int T{5};
  const Vector3d change_points{0, 2, 2};
  expectInvalidArgumentWithMessage(
      [&]() { ampc::Parameterization::moveBlocking(T, change_points); },
      "active knots can not be repeated");
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
      "change_points size must be less than or equal to horizon_steps");
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
      "Last active knot must equal horizon_steps - 1");
}

TEST(ParameterizationLinearInterp, givenChangePointsSizeEqualsT_Succeeds)
{
  // Boundary of the throw condition: size == T is valid (no reduction)
  const int T{5};
  VectorXd change_points{T};
  change_points << 0, 1, 2, 3, 4;
  ASSERT_NO_THROW(ampc::Parameterization::linearInterp(T, change_points));
}

TEST(ParameterizationLinearInterp, givenRepeatedChangePoint_Throws)
{
  const int T{5};
  const Vector4d endpoints{0, 2, 2, 4};
  expectInvalidArgumentWithMessage(
      [&]() { ampc::Parameterization::linearInterp(T, endpoints); },
      "active knots can not be repeated");
}

// ---- Factory: bspline ------------------------------------------------------

TEST(ParameterizationBspline, givenUniformParams_FormsKnotsCorrectly)
{
  const int T{10}, nc{5}, deg{2};
  const ampc::Parameterization p{ampc::Parameterization::bspline(T, nc, deg)};

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
  const int T{10}, nc{5}, deg{2};
  VectorXd active_knots{4};
  active_knots << 0, 2, 5, 9; // first must be 0, last must be T-1 = 9
  const ampc::Parameterization p{
      ampc::Parameterization::bspline(T, nc, deg, active_knots)};

  VectorXd knots_expected{8};
  knots_expected << 0, 0, 0, 2, 5, 9, 9, 9;
  ASSERT_TRUE(expectEigenNear(p.knots, knots_expected, 1e-15));
}

TEST(ParameterizationBspline, givenWrongActiveKnotsCount_Throws)
{
  const int T{10}, nc{5}, deg{2};
  // expected nc - deg + 1 = 4 active knots, providing 3
  VectorXd active_knots{3};
  active_knots << 0, 5, 9;
  expectInvalidArgumentWithMessage(
      [&]() { ampc::Parameterization::bspline(T, nc, deg, active_knots); },
      "Invalid active_knots size");
}

TEST(ParameterizationBspline, givenActiveKnotsOutOfRange_Throws)
{
  const int T{10}, nc{5}, deg{2};
  // nc - deg + 1 = 4 active knots required
  VectorXd active_knots{4};

  // first active knot must be 0
  active_knots << 1, 2, 5, 9;
  expectInvalidArgumentWithMessage(
      [&]() { ampc::Parameterization::bspline(T, nc, deg, active_knots); },
      "First active knot must equal zero");

  // last active knot must be T-1
  active_knots << 0, 2, 5, 7;
  expectInvalidArgumentWithMessage(
      [&]() { ampc::Parameterization::bspline(T, nc, deg, active_knots); },
      "Last active knot must equal horizon_steps - 1");
}
