#include <Eigen/Core>
#include <gtest/gtest-spi.h>
#include <gtest/gtest.h>
#include <limits>

#include "utils.hpp"

using namespace Eigen;

namespace {
constexpr double inf{std::numeric_limits<double>::infinity()};
constexpr double nan_value{std::numeric_limits<double>::quiet_NaN()};
} // namespace

TEST(ExpectEigenNearTest, givenEqualInfinities_Passes)
{
  const Vector3d a{-inf, 1.0, inf};
  EXPECT_TRUE(expectEigenNear(a, a, 0.0));
}

TEST(ExpectEigenNearTest, givenNaN_FailsOnEveryPlatform)
{
  const Vector3d a{0.0, 1.0, 2.0}, b{0.0, nan_value, 2.0};
  // maxCoeff() would drop the NaN with SSE, so check each case explicitly
  EXPECT_NONFATAL_FAILURE(expectEigenNear(a, b, 1.0), "nan");
  EXPECT_NONFATAL_FAILURE(expectEigenNear(b, b, 1.0), "nan");
}

TEST(ExpectEigenNearTest, givenDifferentInfinities_Fails)
{
  const Vector2d a{inf, 0.0}, b{-inf, 0.0}, c{1e300, 0.0};
  EXPECT_NONFATAL_FAILURE(expectEigenNear(a, b, 1.0), "inf");
  EXPECT_NONFATAL_FAILURE(expectEigenNear(a, c, 1.0), "inf");
}

TEST(ExpectEigenNearTest, givenDifferenceAboveDelta_FailsAndNamesTheElement)
{
  const Matrix2d a{Matrix2d::Zero()};
  Matrix2d b{a};
  b(1, 0) = 0.5;
  EXPECT_TRUE(expectEigenNear(a, b, 0.5));
  EXPECT_NONFATAL_FAILURE(expectEigenNear(a, b, 0.4), "at (1, 0)");
}
