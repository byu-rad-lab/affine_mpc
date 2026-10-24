#include "affine_mpc/discretizer.hpp"

#include <Eigen/Core>
#include <cmath>
#include <gtest/gtest.h>
#include <limits>

#include "utils.hpp"

using namespace Eigen;
namespace ampc = affine_mpc;

TEST(DiscretizerTester, givenDoubleIntegrator_MatchesClosedForm)
{
  ampc::Discretizer disc{2, 1};
  Matrix2d Ac;
  Ac << 0, 1, 0, 0;
  const Vector2d Bc{0, 1}, wc{0, -9.81};
  const double dt{0.05};
  MatrixXd Ad{2, 2}, Bd{2, 1};
  VectorXd wd{2};
  disc.discretize(Ac, Bc, wc, dt, Ad, Bd, wd);

  Matrix2d Ad_expected, G_expected;
  Ad_expected << 1, dt, 0, 1;
  G_expected << dt, dt * dt / 2, 0, dt;
  EXPECT_TRUE(expectEigenNear(Ad, Ad_expected, 1e-15));
  EXPECT_TRUE(
      expectEigenNear(disc.getExponentialIntegral(), G_expected, 1e-15));
  EXPECT_TRUE(expectEigenNear(Bd, G_expected * Bc, 1e-15));
  EXPECT_TRUE(expectEigenNear(wd, G_expected * wc, 1e-15));
}

// ||Ac*dt|| = 10, so the scaling and squaring path is used
TEST(DiscretizerTester, givenStiffDiagonalSystem_MatchesClosedForm)
{
  ampc::Discretizer disc{2, 2};
  const Vector2d a{-100, -0.5};
  const Matrix2d Ac{a.asDiagonal()};
  const Matrix2d Bc{Matrix2d::Identity()};
  const Vector2d wc{2, 1};
  const double dt{0.1};
  MatrixXd Ad{2, 2}, Bd{2, 2};
  VectorXd wd{2};
  disc.discretize(Ac, Bc, wc, dt, Ad, Bd, wd, 1e-12);

  const Vector2d e{(a * dt).array().exp()};
  const Vector2d g{(e.array() - 1) / a.array()};
  EXPECT_TRUE(expectEigenNear(Ad, Matrix2d{e.asDiagonal()}, 1e-12));
  EXPECT_TRUE(expectEigenNear(Bd, Matrix2d{g.asDiagonal()}, 1e-12));
  EXPECT_TRUE(expectEigenNear(wd, Vector2d{g.cwiseProduct(wc)}, 1e-12));
}

// Undamped oscillator with ||Ac*dt|| = 20
TEST(DiscretizerTester, givenFastOscillator_MatchesClosedForm)
{
  ampc::Discretizer disc{2, 1};
  const double omega{200}, dt{0.1};
  Matrix2d Ac;
  Ac << 0, omega, -omega, 0;
  MatrixXd Ad{2, 2}, Bd{2, 1};
  VectorXd wd{2};
  disc.discretize(Ac, Vector2d{0, 1}, Vector2d{1, 0}, dt, Ad, Bd, wd, 1e-12);

  const double c{std::cos(omega * dt)}, s{std::sin(omega * dt)};
  Matrix2d Ad_expected, G_expected;
  Ad_expected << c, s, -s, c;
  G_expected << s / omega, (1 - c) / omega, -(1 - c) / omega, s / omega;
  EXPECT_TRUE(expectEigenNear(Ad, Ad_expected, 1e-10));
  EXPECT_TRUE(
      expectEigenNear(disc.getExponentialIntegral(), G_expected, 1e-12));
}

TEST(DiscretizerTester, givenOutputsAliasingInputs_DiscretizesCorrectly)
{
  ampc::Discretizer disc{2, 2};
  Matrix2d Ac;
  Ac << 0, 1, -0.6, -0.1;
  const Matrix2d Bc{{0.0, 1.0}, {0.2, 0.0}};
  const Vector2d wc{0.3, -0.1};
  const double dt{0.1};

  MatrixXd Ad{2, 2}, Bd{2, 2};
  VectorXd wd{2};
  disc.discretize(Ac, Bc, wc, dt, Ad, Bd, wd);

  MatrixXd A{Ac}, B{Bc};
  VectorXd w{wc};
  disc.discretize(A, B, w, dt, A, B, w);
  EXPECT_TRUE(expectEigenNear(A, Ad, 0.0));
  EXPECT_TRUE(expectEigenNear(B, Bd, 0.0));
  EXPECT_TRUE(expectEigenNear(w, wd, 0.0));
}

TEST(DiscretizerTester, givenNewOffset_DiscretizesWithLastExponentialIntegral)
{
  ampc::Discretizer disc{2, 1};
  Matrix2d Ac;
  Ac << 0, 1, -0.6, -0.1;
  const Vector2d Bc{0, 0.2};
  MatrixXd Ad{2, 2}, Bd{2, 1};
  VectorXd wd{2};

  VectorXd wd_offset{2};
  EXPECT_FALSE(disc.hasDiscretized());
  expectLogicErrorWithMessage(
      [&] { disc.discretizeOffset(Vector2d{1, 2}, wd_offset); },
      "discretize() must be called");
  expectLogicErrorWithMessage([&] { (void)disc.getExponentialIntegral(); },
                              "discretize() must be called");

  disc.discretize(Ac, Bc, Vector2d::Zero(), 0.1, Ad, Bd, wd);
  EXPECT_TRUE(disc.hasDiscretized());
  const Vector2d wc{0.5, -1.0};
  disc.discretizeOffset(wc, wd_offset);

  // same as discretizing the whole model with the new offset
  disc.discretize(Ac, Bc, wc, 0.1, Ad, Bd, wd);
  EXPECT_TRUE(expectEigenNear(wd_offset, wd, 0.0));
}

TEST(DiscretizerTester, givenInvalidArguments_Throws)
{
  expectInvalidArgumentWithMessage([] { ampc::Discretizer(0, 1); },
                                   "state_dim must be positive");
  expectInvalidArgumentWithMessage([] { ampc::Discretizer(2, 0); },
                                   "input_dim must be positive");

  ampc::Discretizer disc{2, 1};
  Matrix2d Ac;
  Ac << 0, 1, -0.6, -0.1;
  const Vector2d Bc{0, 0.2}, wc{0, 0};
  MatrixXd Ad{2, 2}, Bd{2, 1};
  VectorXd wd{2};

  expectInvalidArgumentWithMessage(
      [&] { disc.discretize(Ac, Bc, wc, 0.0, Ad, Bd, wd); },
      "dt must be positive");
  expectInvalidArgumentWithMessage(
      [&] { disc.discretize(Ac, Bc, wc, 0.1, Ad, Bd, wd, 0.0); },
      "tol must be positive");
  expectInvalidArgumentWithMessage(
      [&] { disc.discretize(Matrix3d::Zero(), Bc, wc, 0.1, Ad, Bd, wd); },
      "Ac must be 2x2");
  MatrixXd Bd_wrong{2, 2};
  expectInvalidArgumentWithMessage(
      [&] { disc.discretize(Ac, Bc, wc, 0.1, Ad, Bd_wrong, wd); },
      "Bd must be 2x1");
  VectorXd wd_wrong{3};
  expectInvalidArgumentWithMessage([&] { disc.discretizeOffset(wc, wd_wrong); },
                                   "wd must have size 2");

  Ac(0, 0) = std::numeric_limits<double>::quiet_NaN();
  expectInvalidArgumentWithMessage(
      [&] { disc.discretize(Ac, Bc, wc, 0.1, Ad, Bd, wd); },
      "Ac*dt must be finite");
}
