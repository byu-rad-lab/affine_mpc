// Slew-rate bounds (Options::slew_bounds): the rows and bounds of each
// SlewBoundMode, and the solutions of the MPC that uses them.

#include <Eigen/Core>
#include <Eigen/SparseCore>
#include <algorithm>
#include <cmath>
#include <gtest/gtest.h>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "affine_mpc/mpc.hpp"
#include "affine_mpc/options.hpp"
#include "affine_mpc/parameterization.hpp"
#include "affine_mpc/quadratic_program.hpp"
#include "utils.hpp"

namespace ampc = affine_mpc;
using namespace Eigen;
using SBM = ampc::SlewBoundMode;
using IC = ampc::InputCostMode;

namespace {

constexpr double inf{std::numeric_limits<double>::infinity()};

const Matrix2d msd_Ac{{0.0, 1.0}, {-0.6, -0.1}};
const Vector2d msd_Bc{0.0, 0.2};

// The rows of the "slew_bounds" block (dense) and their bounds
struct SlewRows
{
  MatrixXd A;
  VectorXd l, u;
};

SlewRows slewRows(const ampc::QuadraticProgram& qp,
                  const std::vector<ampc::QPBlock>& blocks)
{
  for (const ampc::QPBlock& b : blocks) {
    if (b.name != "slew_bounds")
      continue;
    const MatrixXd A{qp.A};
    return {A.middleRows(b.offset, b.size), qp.l.segment(b.offset, b.size),
            qp.u.segment(b.offset, b.size)};
  }
  return {MatrixXd(0, qp.A.cols()), VectorXd(0), VectorXd(0)};
}

// The slew rows of an MPC assembled at x0 = 0 (columns of the control points)
SlewRows assembledSlewRows(ampc::MPC& mpc)
{
  ampc::QuadraticProgram qp;
  mpc.assembleQP(VectorXd::Zero(mpc.getStateDim()), qp);
  SlewRows rows{slewRows(qp, mpc.getQPConstraintBlocks())};
  rows.A =
      rows.A.leftCols(mpc.getInputDim() * mpc.getNumControlPoints()).eval();
  return rows;
}

// Mass-spring-damper (m inputs drive the same mass) from rest at 0 to 1 m,
// with input limits wide enough that the slew limit decides how fast the
// input rises. The inputs act along the same direction, so an input cost
// makes the solution unique.
void configureMsd(ampc::MPC& mpc, const VectorXd& slew_rate)
{
  const int m{mpc.getInputDim()};
  MatrixXd Bc{2, m};
  for (int j{0}; j < m; ++j)
    Bc.col(j) = (1.0 + 0.5 * j) * msd_Bc;
  mpc.setModelContinuous2Discrete(msd_Ac, Bc, Vector2d::Zero(), 0.1);
  mpc.setInputLimits(VectorXd::Constant(m, -10.0), VectorXd::Constant(m, 10.0));
  mpc.setStateWeights(Vector2d{1.0, 0.1});
  mpc.setReferenceState(Vector2d{1.0, 0.0});
  if (mpc.getOptions().input_cost != ampc::InputCostMode::Off) {
    mpc.setInputWeights(VectorXd::Constant(m, 1e-3));
    mpc.setReferenceInput(VectorXd::Constant(m, 3.0));
  }
  if (mpc.getOptions().slew_initial_input) {
    mpc.setSlewRateInitial(slew_rate);
    mpc.setPreviousInput(VectorXd::Zero(m));
  }
  mpc.setSlewRate(slew_rate);
}

ampc::OSQPSettings tightSettings()
{
  ampc::OSQPSettings settings{ampc::OSQPSolver::getRecommendedSettings(true)};
  settings.eps_abs = settings.eps_rel = 1e-9;
  settings.max_iter = 1000000;
  settings.warm_starting = false;
  return settings;
}

// |u_k - u_{k-1}| for k = 1, ..., T-1, one column per step
MatrixXd sampledChanges(const ampc::MPC& mpc)
{
  const int m{mpc.getInputDim()}, T{mpc.getHorizonSteps()};
  MatrixXd u{m, T};
  mpc.getInputTrajectory(u);
  return (u.rightCols(T - 1) - u.leftCols(T - 1)).cwiseAbs();
}

// Solve the MSD problem from rest; returns the control points
VectorXd solveMsd(ampc::MPC& mpc, const VectorXd& slew_rate)
{
  configureMsd(mpc, slew_rate);
  EXPECT_TRUE(mpc.initializeSolver(tightSettings()));
  EXPECT_EQ(mpc.solve(Vector2d::Zero()), ampc::SolveStatus::Success);
  VectorXd c{mpc.getInputDim() * mpc.getNumControlPoints()};
  mpc.getInputControlPoints(c);
  return c;
}

// The sampled changes stay within the limit of each input (to the solver's
// tolerance); with `active`, some change of every input reaches it
void expectChangesWithin(const ampc::MPC& mpc,
                         const VectorXd& slew_rate,
                         bool active)
{
  const MatrixXd du{sampledChanges(mpc)};
  for (int j{0}; j < mpc.getInputDim(); ++j) {
    EXPECT_LE(du.row(j).maxCoeff(), slew_rate(j) + 1e-6) << "input " << j;
    if (active) {
      EXPECT_NEAR(du.row(j).maxCoeff(), slew_rate(j), 1e-5) << "input " << j;
    }
  }
}

const ampc::Formulation formulations[]{ampc::Formulation::Condensed,
                                       ampc::Formulation::Sparse};

const char* name(ampc::Formulation f)
{
  return f == ampc::Formulation::Sparse ? "sparse" : "condensed";
}

} // namespace

// ---- rows and bounds --------------------------------------------------------

// Degree 0 (move blocking, one control point per step or blocked): both modes
// bound the control point differences by the limit, which is exact
TEST(SlewBoundsTester, givenMoveBlocking_BoundsControlPointDifferences)
{
  const int n{2}, m{2};
  const Vector2d slew{0.3, 0.7};
  for (const auto& [label, param] :
       {std::pair{"one per step", ampc::Parameterization::moveBlocking(8, 8)},
        std::pair{"blocked", ampc::Parameterization::moveBlocking(12, 4)},
        std::pair{"geometric",
                  ampc::Parameterization::moveBlocking(15, 5, 1.4)}}) {
    const int nc{param.num_control_points};
    MatrixXd A_expected{MatrixXd::Zero(m * (nc - 1), m * nc)};
    A_expected.diagonal().setConstant(-1.0);
    A_expected.diagonal(m).setConstant(1.0);
    const VectorXd u_expected{slew.replicate(nc - 1, 1)};
    for (const SBM mode : {SBM::ControlPoints, SBM::Trajectory}) {
      SCOPED_TRACE(::testing::Message() << label << " " << mode);
      ampc::MPC mpc{n, m, param, {.slew_bounds = mode}};
      configureMsd(mpc, slew);
      const SlewRows rows{assembledSlewRows(mpc)};
      EXPECT_TRUE(expectEigenNear(rows.A, A_expected, 0.0));
      EXPECT_TRUE(expectEigenNear(rows.u, u_expected, 0.0));
      EXPECT_TRUE(expectEigenNear(rows.l, -u_expected, 0.0));
    }
  }
}

// ControlPoints: |c_i - c_{i-1}| <= (t_{i+p} - t_i) du_max / p per row and
// input, from the knots in samples
TEST(SlewBoundsTester, givenBspline_ControlPointBoundsFollowTheKnots)
{
  const int m{2};
  const Vector2d slew{0.3, 0.7};
  VectorXd knots1{7}; // degree 1, intervals of 2, 3, 4, 3 samples
  knots1 << 0, 0, 2, 5, 9, 12, 12;
  VectorXd knots3{10}; // degree 3, non-integer knots
  knots3 << 0, 0, 0, 0, 7.3, 22.9, 29, 29, 29, 29;
  for (const auto& param : {ampc::Parameterization{13, 1, knots1},
                            ampc::Parameterization::bspline(30, 2, 7, 1.3),
                            ampc::Parameterization{30, 3, knots3}}) {
    SCOPED_TRACE(::testing::Message() << param);
    const int nc{param.num_control_points}, p{param.degree};
    ampc::MPC mpc{2, m, param, {.slew_bounds = SBM::ControlPoints}};
    configureMsd(mpc, slew);
    const SlewRows rows{assembledSlewRows(mpc)};
    ASSERT_EQ(rows.A.rows(), m * (nc - 1));
    for (int i{1}; i < nc; ++i) {
      const double factor{(param.knots(i + p) - param.knots(i)) / p};
      for (int j{0}; j < m; ++j) {
        const int row{(i - 1) * m + j};
        EXPECT_DOUBLE_EQ(rows.u(row), factor * slew(j)) << "row " << row;
        EXPECT_DOUBLE_EQ(rows.l(row), -factor * slew(j)) << "row " << row;
        EXPECT_EQ(rows.A(row, (i - 1) * m + j), -1.0);
        EXPECT_EQ(rows.A(row, i * m + j), 1.0);
        EXPECT_EQ(rows.A.row(row).cwiseAbs().sum(), 2.0);
      }
    }
  }
}

// Trajectory on degree 1 uses the control point rows when every knot interval
// holds a whole sample step, and the sampled differences otherwise (here
// intervals of 9/7 samples, some of which hold no whole step)
TEST(SlewBoundsTester, givenDegree1_TrajectoryUsesControlPointRowsOnlyWhenExact)
{
  const int m{1};
  const VectorXd slew{VectorXd::Constant(m, 0.4)};
  const auto exact_cp{ampc::Parameterization::linearInterp(13, 5)};
  const auto short_intervals{ampc::Parameterization::linearInterp(10, 8)};
  {
    ampc::MPC cp{2, m, exact_cp, {.slew_bounds = SBM::ControlPoints}},
        exact{2, m, exact_cp, {.slew_bounds = SBM::Trajectory}};
    configureMsd(cp, slew);
    configureMsd(exact, slew);
    const SlewRows a{assembledSlewRows(cp)}, b{assembledSlewRows(exact)};
    EXPECT_TRUE(expectEigenNear(a.A, b.A, 0.0));
    EXPECT_TRUE(expectEigenNear(a.u, b.u, 0.0));
    EXPECT_TRUE(expectEigenNear(a.u, VectorXd::Constant(4, 3 * 0.4), 1e-15));
  }
  {
    ampc::MPC exact{2, m, short_intervals, {.slew_bounds = SBM::Trajectory}};
    configureMsd(exact, slew);
    const SlewRows rows{assembledSlewRows(exact)};
    // D Phi, one row per sampled change
    const MatrixXd Phi{denseBasis(short_intervals)};
    const MatrixXd DPhi{Phi.bottomRows(9) - Phi.topRows(9)};
    EXPECT_TRUE(expectEigenNear(rows.A, DPhi, 1e-15));
    EXPECT_TRUE(expectEigenNear(rows.u, VectorXd::Constant(9, 0.4), 0.0));
  }
}

// Trajectory on degree >= 2 and on a custom basis: the sampled differences D
// Phi, every input with its own limit
TEST(SlewBoundsTester, givenTrajectoryMode_RowsAreSampledDifferences)
{
  const int m{2};
  const Vector2d slew{0.3, 0.7};
  for (const auto& param :
       {ampc::Parameterization::bspline(20, 2, 6),
        ampc::Parameterization::bspline(25, 3, 8, 0.8),
        ampc::Parameterization::fromBasis(12, laguerreBasis(12, 4, 0.5))}) {
    SCOPED_TRACE(::testing::Message() << param);
    const int T{param.horizon_steps}, nc{param.num_control_points};
    ampc::MPC mpc{2, m, param, {.slew_bounds = SBM::Trajectory}};
    configureMsd(mpc, slew);
    const SlewRows rows{assembledSlewRows(mpc)};
    const MatrixXd Phi{denseBasis(param)};
    const MatrixXd DPhi{Phi.bottomRows(T - 1) - Phi.topRows(T - 1)};
    ASSERT_EQ(rows.A.rows(), m * (T - 1));
    for (int k{0}; k < T - 1; ++k) {
      for (int j{0}; j < m; ++j) {
        const int row{k * m + j};
        for (int i{0}; i < nc; ++i)
          EXPECT_NEAR(rows.A(row, i * m + j), DPhi(k, i), 1e-15);
        EXPECT_EQ(rows.u(row), slew(j));
        EXPECT_EQ(rows.l(row), -slew(j));
      }
    }
  }
}

// A repeated degree-1 knot (a jump) gets factor 0, which forbids the jump for
// a finite limit, and an infinite limit stays infinite (not NaN)
TEST(SlewBoundsTester, givenZeroLengthInterval_BoundIsZeroOrInfinite)
{
  VectorXd knots{6};
  knots << 0, 0, 4, 4, 8, 8;
  const ampc::Parameterization param{9, 1, knots};
  ampc::MPC mpc{2, 1, param, {.slew_bounds = SBM::ControlPoints}};
  configureMsd(mpc, VectorXd::Constant(1, 0.5));
  SlewRows rows{assembledSlewRows(mpc)};
  EXPECT_TRUE(expectEigenNear(rows.u, Vector3d{2.0, 0.0, 2.0}, 0.0));
  mpc.setSlewRate(VectorXd::Constant(1, inf));
  rows = assembledSlewRows(mpc);
  EXPECT_TRUE(expectEigenNear(rows.u, Vector3d::Constant(inf), 0.0));
  EXPECT_TRUE(expectEigenNear(rows.l, Vector3d::Constant(-inf), 0.0));
}

// The ControlPoints bounds are sufficient for any knots. A sampled change is
// linear in the control point differences d_i = c_i - c_{i-1} (the basis sums
// to one, so c_0 drops out): u_k - u_{k-1} = sum_i S_{k,i} d_i with S_{k,i} =
// sum_{j >= i} (Phi_{k,j} - Phi_{k-1,j}). Over the box |d_i| <= bound_i its
// largest value is sum_i |S_{k,i}| bound_i, which must not exceed the limit.
// Degree 0 and degree 1 with integer knots reach it.
TEST(SlewBoundsTester, givenControlPointsAtTheirBounds_SampledChangesStayWithin)
{
  using P = ampc::Parameterization;
  VectorXd repeated{11}; // degree 2, a double knot at 12 and a triple at 25
  repeated << 0, 0, 0, 12, 12, 25, 25, 25, 39, 39, 39;
  VectorXd fractional{10}; // degree 3, non-integer knots
  fractional << 0, 0, 0, 0, 7.3, 22.9, 29, 29, 29, 29;
  VectorXd unclamped{9}; // degree 2, knots outside [0, T-1]
  unclamped << -6, -3, 0, 5.5, 11, 17, 23, 26, 29;
  const std::vector<std::pair<P, bool>> cases{
      {P::moveBlocking(20, 6, 1.3), true},  {P::linearInterp(21, 6), true},
      {P::linearInterp(30, 6, 1.4), false}, {P::linearInterp(10, 8), false},
      {P::bspline(40, 2, 6), false},        {P::bspline(60, 3, 8), false},
      {P::bspline(50, 4, 7), false},        {P::bspline(80, 3, 9, 0.7), false},
      {P{40, 2, repeated}, false},          {P{30, 3, fractional}, false},
      {P{24, 2, unclamped}, false}};
  const double slew{0.3};
  for (const auto& [param, reaches] : cases) {
    SCOPED_TRACE(::testing::Message() << param);
    ampc::MPC mpc{2, 1, param, {.slew_bounds = SBM::ControlPoints}};
    configureMsd(mpc, VectorXd::Constant(1, slew));
    const SlewRows rows{assembledSlewRows(mpc)};
    const int nc{param.num_control_points};
    ASSERT_EQ(rows.u.size(), nc - 1);
    EXPECT_TRUE(expectEigenNear(rows.l, -rows.u, 0.0));

    const MatrixXd Phi{denseBasis(param)};
    const MatrixXd DPhi{Phi.bottomRows(Phi.rows() - 1)
                        - Phi.topRows(Phi.rows() - 1)};
    double largest{0.0};
    for (Index k{0}; k < DPhi.rows(); ++k) {
      double worst{0.0}, S{0.0};
      for (int i{nc - 1}; i >= 1; --i) {
        S += DPhi(k, i);
        worst += std::abs(S) * rows.u(i - 1);
      }
      largest = std::max(largest, worst);
    }
    EXPECT_LE(largest, slew * (1.0 + 1e-12));
    if (reaches) {
      EXPECT_NEAR(largest, slew, 1e-12);
    }
  }
}

// ---- options and setters ----------------------------------------------------

TEST(SlewBoundsTester, givenCustomBasis_ControlPointsThrowsAndTrajectoryWorks)
{
  const auto param{
      ampc::Parameterization::fromBasis(10, laguerreBasis(10, 3, 0.5))};
  for (const ampc::Formulation f : formulations) {
    SCOPED_TRACE(name(f));
    expectInvalidArgumentWithMessage(
        [&] { ampc::MPC(2, 1, param, {.slew_bounds = SBM::ControlPoints}, f); },
        "SlewBoundMode::ControlPoints requires a B-spline parameterization");
    EXPECT_NO_THROW(
        ampc::MPC(2, 1, param, {.slew_bounds = SBM::Trajectory}, f));
  }
}

TEST(SlewBoundsTester, givenSlewBoundsOff_SetAndGetSlewRateThrow)
{
  ampc::MPC mpc{2, 1, ampc::Parameterization::linearInterp(10, 4)};
  expectLogicErrorWithMessage(
      [&] { mpc.setSlewRate(VectorXd::Constant(1, 0.5)); },
      "Slew rate is not enabled");
  expectLogicErrorWithMessage([&] { (void)mpc.getSlewRate(); },
                              "Slew rate is not enabled");
}

// ---- solutions --------------------------------------------------------------

// Degree 1 with uniform, non-uniform, and geometric knots whose intervals all
// hold a whole sample step: ControlPoints and Trajectory give the solution of
// the sampled rows (the same basis as a custom basis, whose Trajectory rows
// bound every sampled change), and the limit is active
TEST(SlewBoundsTester, givenDegree1_ControlPointsMatchesSampledBounds)
{
  const int n{2}, m{2};
  const Vector2d slew{0.1, 0.15};
  VectorXd endpoints{5};
  endpoints << 0, 1, 3, 7, 19;
  for (const auto& param : {ampc::Parameterization::linearInterp(20, 5),
                            ampc::Parameterization::linearInterp(20, endpoints),
                            ampc::Parameterization::linearInterp(40, 5, 1.5)}) {
    SCOPED_TRACE(::testing::Message() << param);
    const auto sampled{ampc::Parameterization::fromBasis(param.horizon_steps,
                                                         denseBasis(param))};
    for (const ampc::Formulation f : formulations) {
      SCOPED_TRACE(name(f));
      ampc::MPC cp{
          n,
          m,
          param,
          {.input_cost = IC::Trajectory, .slew_bounds = SBM::ControlPoints},
          f},
          exact{n,
                m,
                param,
                {.input_cost = IC::Trajectory, .slew_bounds = SBM::Trajectory},
                f},
          reference{
              n,
              m,
              sampled,
              {.input_cost = IC::Trajectory, .slew_bounds = SBM::Trajectory},
              f};
      const VectorXd c_cp{solveMsd(cp, slew)};
      const VectorXd c_exact{solveMsd(exact, slew)};
      const VectorXd c_ref{solveMsd(reference, slew)};
      EXPECT_TRUE(expectEigenNear(c_cp, c_ref, 1e-5));
      EXPECT_TRUE(expectEigenNear(c_exact, c_ref, 1e-5));
      expectChangesWithin(cp, slew, true);
    }
  }
}

// Degree 2 and 3 (and degree 1 with intervals shorter than a step):
// ControlPoints never exceeds the limit (sufficient), and Trajectory reaches
// it. Condensed and sparse give the same solutions.
TEST(SlewBoundsTester,
     givenHigherDegree_ControlPointsIsSafeAndTrajectoryIsActive)
{
  const int n{2}, m{2};
  const Vector2d slew{0.1, 0.15};
  for (const auto& param : {ampc::Parameterization::bspline(30, 2, 7),
                            ampc::Parameterization::bspline(30, 3, 8, 1.3),
                            ampc::Parameterization::linearInterp(10, 8)}) {
    SCOPED_TRACE(::testing::Message() << param);
    VectorXd c_cp[2], c_exact[2];
    for (int i{0}; i < 2; ++i) {
      const ampc::Formulation f{formulations[i]};
      SCOPED_TRACE(name(f));
      ampc::MPC cp{
          n,
          m,
          param,
          {.input_cost = IC::Trajectory, .slew_bounds = SBM::ControlPoints},
          f},
          exact{n,
                m,
                param,
                {.input_cost = IC::Trajectory, .slew_bounds = SBM::Trajectory},
                f};
      c_cp[i] = solveMsd(cp, slew);
      c_exact[i] = solveMsd(exact, slew);
      expectChangesWithin(cp, slew, false);
      expectChangesWithin(exact, slew, true);
    }
    EXPECT_TRUE(expectEigenNear(c_cp[0], c_cp[1], 1e-5));
    EXPECT_TRUE(expectEigenNear(c_exact[0], c_exact[1], 1e-5));
  }
}

// A custom basis with Trajectory matches the augmented-state workaround: the
// state holds the previous input, x~ = [x; u_{k-1}], and general constraints Hx
// = [0, -I], Hu = I bound u_k - u_{k-1} at every step (k = 0 against the
// previous input, as slew_initial_input does)
TEST(SlewBoundsTester, givenCustomBasis_TrajectoryMatchesAugmentedState)
{
  const int n{2}, m{1}, T{15};
  const auto param{
      ampc::Parameterization::fromBasis(T, laguerreBasis(T, 4, 0.6))};
  const VectorXd slew{VectorXd::Constant(m, 0.12)};
  const VectorXd u_prev{VectorXd::Constant(m, 0.05)};
  const Vector2d x0{0.0, 0.0};

  // the reference, condensed (OSQP does not reach the tight tolerance on the
  // sparse augmented problem, whose state rows converge slowly)
  ampc::MPC augmented{n + m,
                      m,
                      param,
                      {.input_cost = ampc::InputCostMode::Trajectory,
                       .num_general_constraints = m}};
  ampc::MPC plain{n, m, param}; // only for the discrete model
  plain.setModelContinuous2Discrete(msd_Ac, msd_Bc, Vector2d::Zero(), 0.1);
  const DiscreteModel model{readModel(plain)};
  MatrixXd Ad{MatrixXd::Zero(n + m, n + m)}, Bd{n + m, m};
  Ad.topLeftCorner(n, n) = model.Ad;
  Bd << model.Bd, MatrixXd::Identity(m, m);
  augmented.setModelDiscrete(Ad, Bd, VectorXd::Zero(n + m));
  augmented.setInputLimits(VectorXd::Constant(m, -10.0),
                           VectorXd::Constant(m, 10.0));
  augmented.setStateWeights(Vector3d{1.0, 0.1, 0.0});
  augmented.setReferenceState(Vector3d{1.0, 0.0, 0.0});
  augmented.setInputWeights(VectorXd::Constant(m, 1e-3));
  augmented.setReferenceInput(VectorXd::Constant(m, 3.0));
  MatrixXd Hx{MatrixXd::Zero(m, n + m)};
  Hx.rightCols(m) = -MatrixXd::Identity(m, m);
  augmented.setGeneralConstraints(Hx, MatrixXd::Identity(m, m), -slew, slew);
  ASSERT_TRUE(augmented.initializeSolver(tightSettings()));
  ASSERT_EQ(augmented.solve(Vector3d{x0(0), x0(1), u_prev(0)}),
            ampc::SolveStatus::Success);
  VectorXd c_aug{m * param.num_control_points};
  augmented.getInputControlPoints(c_aug);

  for (const ampc::Formulation f : formulations) {
    SCOPED_TRACE(name(f));
    ampc::MPC exact{n,
                    m,
                    param,
                    {.input_cost = ampc::InputCostMode::Trajectory,
                     .slew_initial_input = true,
                     .slew_bounds = SBM::Trajectory},
                    f};
    configureMsd(exact, slew);
    exact.setPreviousInput(u_prev);
    ASSERT_TRUE(exact.initializeSolver(tightSettings()));
    ASSERT_EQ(exact.solve(x0), ampc::SolveStatus::Success);
    VectorXd c_exact{c_aug.size()};
    exact.getInputControlPoints(c_exact);
    EXPECT_TRUE(expectEigenNear(c_exact, c_aug, 1e-5));
    expectChangesWithin(exact, slew, true);
  }
}

// Limit changes between solves only update the bounds (the rows stay), and
// the next solve follows the new limit
TEST(SlewBoundsTester, givenSlewRateUpdate_NextSolveUsesNewLimit)
{
  const int n{2}, m{1};
  for (const auto& [param, mode] :
       {std::pair{ampc::Parameterization::bspline(25, 2, 6), SBM::Trajectory},
        std::pair{ampc::Parameterization::linearInterp(21, 6),
                  SBM::ControlPoints}}) {
    for (const ampc::Formulation f : formulations) {
      SCOPED_TRACE(::testing::Message()
                   << param << " " << mode << " " << name(f));
      ampc::MPC mpc{
          n, m, param, {.input_cost = IC::Trajectory, .slew_bounds = mode}, f};
      (void)solveMsd(mpc, VectorXd::Constant(m, 0.1));
      expectChangesWithin(mpc, VectorXd::Constant(m, 0.1), true);
      const ampc::QuadraticProgram before{mpc.getQP()};

      const VectorXd tighter{VectorXd::Constant(m, 0.05)};
      mpc.setSlewRate(tighter);
      EXPECT_TRUE(mpc.getSlewRate().isApprox(tighter));
      ASSERT_EQ(mpc.solve(Vector2d::Zero()), ampc::SolveStatus::Success);
      expectChangesWithin(mpc, tighter, true);
      const ampc::QuadraticProgram after{mpc.getQP()};
      EXPECT_TRUE(expectEigenNear(MatrixXd{after.A}, MatrixXd{before.A}, 0.0));
      const SlewRows rows_before{slewRows(before, mpc.getQPConstraintBlocks())};
      const SlewRows rows_after{slewRows(after, mpc.getQPConstraintBlocks())};
      EXPECT_TRUE(expectEigenNear(rows_after.u, 0.5 * rows_before.u, 1e-15));
      EXPECT_TRUE(mpc.isWithinSparsityPattern());
    }
  }
}
