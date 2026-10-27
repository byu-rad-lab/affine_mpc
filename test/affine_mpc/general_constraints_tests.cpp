#include <Eigen/Core>
#include <algorithm>
#include <gtest/gtest.h>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <vector>

#include "affine_mpc/mpc.hpp"
#include "affine_mpc/options.hpp"
#include "affine_mpc/osqp_solver.hpp"
#include "affine_mpc/parameterization.hpp"
#include "qp_builder.hpp" // internal (src/): the row layout
#include "utils.hpp"

namespace ampc = affine_mpc;
using namespace Eigen;

namespace {

const double inf{std::numeric_limits<double>::infinity()};

// Exposes the assembled QP, its row layout, and the pending categories
class ConstraintInspector : public ampc::MPC
{
public:
  using ampc::MPC::MPC;
  void assemble(const Ref<const VectorXd>& x0) { (void)this->qpUpdateX0(x0); }
  MatrixXd A() const { return denseA(this->qp()); }
  const VectorXd& l() const { return this->qp().l; }
  const VectorXd& u() const { return this->qp().u; }
  ampc::detail::DataChangeSet pending() const { return this->pendingChanges(); }
  const ampc::detail::QPRows& rows() const { return this->qpBuilder().rows(); }
};

const auto param{ampc::Parameterization::bspline(20, 2, 8)};
const int T{param.horizon_steps};

// Mass-spring-damper (position, velocity) asked to move to position 1
void configure(ampc::MPC& mpc, double u_limit = 3.0)
{
  Matrix2d Ac;
  Ac << 0, 1, -0.6, -0.1;
  mpc.setModelContinuous2Discrete(Ac, Vector2d{0, 0.2}, Vector2d::Zero(), 0.1);
  mpc.setStateWeights(Vector2d{1.0, 0.1});
  mpc.setReferenceState(Vector2d{1.0, 0.0});
  mpc.setInputLimits(VectorXd::Constant(1, -u_limit),
                     VectorXd::Constant(1, u_limit));
}

// Polishing makes the solutions of equivalent QPs agree to round-off
OSQPSettings polishedSettings()
{
  OSQPSettings settings{ampc::OSQPSolver::getRecommendedSettings()};
  settings.polishing = true;
  settings.max_iter = 20000;
  return settings;
}

VectorXd solveControlPoints(ampc::MPC& mpc, const Vector2d& x0)
{
  EXPECT_TRUE(mpc.initializeSolver(polishedSettings()));
  EXPECT_EQ(mpc.solve(x0), ampc::SolveStatus::Success);
  VectorXd c{param.num_control_points};
  mpc.getInputControlPoints(c);
  return c;
}

// Predicted states as columns x_1, ..., x_T and inputs u_0, ..., u_{T-1}
MatrixXd states(const ampc::MPC& mpc)
{
  VectorXd x_traj{2 * T};
  mpc.getPredictedStateTrajectory(x_traj);
  return Map<const MatrixXd>(x_traj.data(), 2, T);
}

VectorXd inputs(const ampc::MPC& mpc)
{
  VectorXd u_traj{T};
  mpc.getInputTrajectory(u_traj);
  return u_traj;
}

const ampc::Options general_only{.num_general_constraints = 1};
const ampc::Options general_and_terminal{.num_general_constraints = 1,
                                         .num_terminal_constraints = 1};

// ---- equivalence with the existing bounds -----------------------------------

// Hx = rows of the identity and Hu = 0 on every step, plus the same rows as
// terminal constraints, equal state limits on the same states: the general
// rows also hold x_0 (inside the limits here) and stop at x_{T-1}, which the
// terminal rows cover
template <ampc::Formulation F> void checkIdentityRowsMatchStateLimits()
{
  struct Case
  {
    std::vector<int> states;
    VectorXd lo, hi;
  };
  const std::vector<Case> cases{
      {{0}, VectorXd::Constant(1, -0.5), VectorXd::Constant(1, 0.5)},
      {{0, 1}, Vector2d{-0.5, -inf}, Vector2d{0.5, inf}},
  };
  for (const Case& c : cases) {
    const auto p{static_cast<int>(c.states.size())};
    SCOPED_TRACE(::testing::Message() << p << " bounded states");
    ampc::MPC bounded{
        2, 1, param,
        ampc::Options{.saturate_states = true, .bounded_states = c.states}, F};
    configure(bounded);
    bounded.setStateLimits(c.lo, c.hi);

    MatrixXd Hx{MatrixXd::Zero(p, 2)};
    for (int i{0}; i < p; ++i)
      Hx(i, c.states[static_cast<size_t>(i)]) = 1.0;
    ampc::MPC general{2, 1, param,
                      ampc::Options{.num_general_constraints = p,
                                    .num_terminal_constraints = p},
                      F};
    configure(general);
    general.setGeneralConstraints(Hx, MatrixXd::Zero(p, 1), c.lo, c.hi);
    general.setTerminalConstraints(Hx, c.lo, c.hi);

    const VectorXd c_bounded{solveControlPoints(bounded, Vector2d::Zero())};
    const VectorXd c_general{solveControlPoints(general, Vector2d::Zero())};
    EXPECT_TRUE(expectEigenNear(c_general, c_bounded, 1e-8));
    // the limit is active, so the comparison means something
    EXPECT_NEAR(states(general).row(0).maxCoeff(), 0.5, 1e-8);
  }
}

TEST(GeneralConstraintsTester, givenIdentityRows_CondensedMatchesStateLimits)
{
  checkIdentityRowsMatchStateLimits<ampc::Formulation::Condensed>();
}

TEST(GeneralConstraintsTester, givenIdentityRows_SparseMatchesStateLimits)
{
  checkIdentityRowsMatchStateLimits<ampc::Formulation::Sparse>();
}

// Hx = 0 and Hu = I bound every sampled input u_0, ..., u_{T-1}, which is what
// saturate_input_trajectory does for degree 2
template <ampc::Formulation F> void checkInputRowsMatchSampledInputLimits()
{
  // (a tighter limit is active at more samples than there are control points,
  // which OSQP's polishing does not resolve to round-off)
  const double limit{2.0};
  ampc::MPC sampled{2, 1, param,
                    ampc::Options{.saturate_input_trajectory = true}, F};
  configure(sampled, limit);
  ampc::MPC general{2, 1, param, general_only, F};
  configure(general, 100.0); // inactive limits on the control points
  general.setGeneralConstraints(RowVector2d::Zero(), MatrixXd::Identity(1, 1),
                                VectorXd::Constant(1, -limit),
                                VectorXd::Constant(1, limit));

  const VectorXd c_sampled{solveControlPoints(sampled, Vector2d::Zero())};
  const VectorXd c_general{solveControlPoints(general, Vector2d::Zero())};
  EXPECT_TRUE(expectEigenNear(c_general, c_sampled, 1e-8));
  EXPECT_NEAR(inputs(general).maxCoeff(), limit, 1e-8);
}

TEST(GeneralConstraintsTester, givenInputRows_CondensedMatchesSampledLimits)
{
  checkInputRowsMatchSampledInputLimits<ampc::Formulation::Condensed>();
}

TEST(GeneralConstraintsTester, givenInputRows_SparseMatchesSampledLimits)
{
  checkInputRowsMatchSampledInputLimits<ampc::Formulation::Sparse>();
}

// ---- coupled, initial-step, and terminal rows -------------------------------

// x_k + u_k <= c holds at every step (x_0 is the measured state) and is
// active; both formulations give the same solution
VectorXd checkCoupledConstraint(ampc::MPC& mpc, const Vector2d& x0)
{
  // active at fewer steps than there are control points, so polishing gives
  // the exact solution (a tighter c is active at more of them)
  const double c{3.0};
  configure(mpc);
  mpc.setGeneralConstraints(RowVector2d{1.0, 0.0}, MatrixXd::Identity(1, 1),
                            VectorXd::Constant(1, -inf),
                            VectorXd::Constant(1, c));
  const VectorXd ctrl{solveControlPoints(mpc, x0)};

  const MatrixXd x{states(mpc)};
  const VectorXd u{inputs(mpc)};
  VectorXd g{T};
  g(0) = x0(0) + u(0);
  for (int k{1}; k < T; ++k)
    g(k) = x(0, k - 1) + u(k);
  EXPECT_LE(g.maxCoeff(), c + 1e-8);
  EXPECT_NEAR(g.maxCoeff(), c, 1e-8); // active
  return ctrl;
}

TEST(GeneralConstraintsTester, givenCoupledConstraint_HoldsAtEveryStep)
{
  ampc::MPC condensed{2, 1, param, general_only};
  ampc::MPC sparse{2, 1, param, general_only, ampc::Formulation::Sparse};
  const VectorXd c_condensed{
      checkCoupledConstraint(condensed, Vector2d::Zero())};
  const VectorXd c_sparse{checkCoupledConstraint(sparse, Vector2d::Zero())};
  EXPECT_TRUE(expectEigenNear(c_condensed, c_sparse, 1e-8));
}

// A custom basis (dense band: Hu on every coefficient at every step, all
// condensed columns from the first step on) equal to the B-spline's sampled
// basis gives the B-spline's solution, and a Laguerre basis holds the
// constraint at every step in both formulations. The custom basis bounds
// every sampled input, so the B-spline does too (saturate_input_trajectory).
TEST(GeneralConstraintsTester, givenCustomBasis_HoldsAtEveryStep)
{
  const auto custom{ampc::Parameterization::fromBasis(T, denseBasis(param))};
  const ampc::Options sampled{.saturate_input_trajectory = true,
                              .num_general_constraints = 1};
  ampc::MPC condensed_b{2, 1, param, sampled};
  ampc::MPC condensed_c{2, 1, custom, general_only};
  ampc::MPC sparse_b{2, 1, param, sampled, ampc::Formulation::Sparse};
  ampc::MPC sparse_c{2, 1, custom, general_only, ampc::Formulation::Sparse};
  const VectorXd c_b{checkCoupledConstraint(condensed_b, Vector2d::Zero())};
  EXPECT_TRUE(expectEigenNear(
      checkCoupledConstraint(condensed_c, Vector2d::Zero()), c_b, 1e-8));
  EXPECT_TRUE(expectEigenNear(
      checkCoupledConstraint(sparse_b, Vector2d::Zero()), c_b, 1e-8));
  EXPECT_TRUE(expectEigenNear(
      checkCoupledConstraint(sparse_c, Vector2d::Zero()), c_b, 1e-8));

  // same number of coefficients as the B-spline (solveControlPoints)
  const auto laguerre{ampc::Parameterization::fromBasis(
      T, laguerreBasis(T, param.num_control_points, 0.7))};
  ampc::MPC condensed_l{2, 1, laguerre, general_only};
  ampc::MPC sparse_l{2, 1, laguerre, general_only, ampc::Formulation::Sparse};
  EXPECT_TRUE(expectEigenNear(
      checkCoupledConstraint(condensed_l, Vector2d::Zero()),
      checkCoupledConstraint(sparse_l, Vector2d::Zero()), 1e-8));
}

// Starting at position 1, x_0 + u_0 <= 3 limits u_0 to 2, through the
// measured state only
TEST(GeneralConstraintsTester, givenMeasuredState_InitialRowsLimitFirstInput)
{
  auto check = [](ampc::MPC& mpc) {
    configure(mpc, 10.0);
    mpc.setReferenceState(Vector2d{3.0, 0.0});
    mpc.setGeneralConstraints(RowVector2d{1.0, 0.0}, MatrixXd::Identity(1, 1),
                              VectorXd::Constant(1, -inf),
                              VectorXd::Constant(1, 3.0));
    (void)solveControlPoints(mpc, Vector2d{1.0, 0.0});
    VectorXd u0{1};
    mpc.getNextInput(u0);
    EXPECT_NEAR(u0(0), 2.0, 1e-8);
  };
  ampc::MPC condensed{2, 1, param, general_only};
  check(condensed);
  ampc::MPC sparse{2, 1, param, general_only, ampc::Formulation::Sparse};
  check(sparse);
}

// Changing x0 alone only moves bounds: nothing is pending, A is unchanged,
// and the k = 0 rows hold h - Hx x_0 (sparse: the rows for k >= 1 do not move)
template <ampc::Formulation F> void checkInitialRowsFollowX0()
{
  ConstraintInspector mpc{2, 1, param, general_and_terminal, F};
  configure(mpc);
  const RowVector2d Hx{1.0, 0.5};
  mpc.setGeneralConstraints(Hx, MatrixXd::Constant(1, 1, 0.3),
                            VectorXd::Constant(1, -2.0),
                            VectorXd::Constant(1, 1.5));
  mpc.setTerminalConstraints(RowVector2d{1.0, 1.0}, VectorXd::Constant(1, -inf),
                             VectorXd::Constant(1, 2.0));
  ASSERT_TRUE(mpc.initializeSolver());
  const Vector2d x0_a{0.1, 0.2}, x0_b{-0.3, 0.4};
  ASSERT_EQ(mpc.solve(x0_a), ampc::SolveStatus::Success);
  EXPECT_TRUE(mpc.pending().empty());
  const MatrixXd A{mpc.A()};
  const VectorXd l{mpc.l()}, u{mpc.u()};

  EXPECT_EQ(mpc.prepare(), ampc::SolveStatus::Success);
  mpc.assemble(x0_b);
  EXPECT_TRUE(expectEigenNear(mpc.A(), A, 0.0));
  const ampc::detail::RowBlock rows{mpc.rows().general_constraints};
  ASSERT_EQ(rows.rows, T);
  EXPECT_DOUBLE_EQ(mpc.l()(rows.offset), -2.0 - Hx.dot(x0_b));
  EXPECT_DOUBLE_EQ(mpc.u()(rows.offset), 1.5 - Hx.dot(x0_b));
  EXPECT_NE(mpc.u()(rows.offset), u(rows.offset));
  if (F == ampc::Formulation::Sparse) {
    EXPECT_TRUE(expectEigenNear(mpc.l().tail(mpc.l().size() - rows.offset - 1),
                                l.tail(l.size() - rows.offset - 1), 0.0));
  }
}

TEST(GeneralConstraintsTester, givenNewX0_CondensedMovesOnlyBounds)
{
  checkInitialRowsFollowX0<ampc::Formulation::Condensed>();
}

TEST(GeneralConstraintsTester, givenNewX0_SparseMovesOnlyBounds)
{
  checkInitialRowsFollowX0<ampc::Formulation::Sparse>();
}

// hf_min <= Hf x_T <= hf_max holds at the final predicted state; here it pins
// the final position below the reference
TEST(GeneralConstraintsTester, givenTerminalConstraints_HoldAtFinalState)
{
  const ampc::Options terminal{.num_terminal_constraints = 2};
  Matrix2d Hf;
  Hf << 1.0, 0.0, 1.0, 1.0;
  const Vector2d hf_min{0.3, -inf}, hf_max{0.3, 0.35};
  auto check = [&](ampc::MPC& mpc) {
    configure(mpc);
    mpc.setTerminalConstraints(Hf, hf_min, hf_max);
    const VectorXd c{solveControlPoints(mpc, Vector2d::Zero())};
    const Vector2d xT{states(mpc).col(T - 1)};
    EXPECT_NEAR(xT(0), 0.3, 1e-8);
    EXPECT_LE(xT(0) + xT(1), 0.35 + 1e-8);
    return c;
  };
  ampc::MPC condensed{2, 1, param, terminal};
  ampc::MPC sparse{2, 1, param, terminal, ampc::Formulation::Sparse};
  EXPECT_TRUE(expectEigenNear(check(condensed), check(sparse), 1e-8));
}

// With one step the general rows are only the k = 0 rows (Hu u_0 with Hx x_0
// in the bounds, no state rows) and the terminal rows are on x_1. The cost
// drives u_0 to its limit, so whichever constraint is tighter is active. One
// step barely moves the state, so the weights are large enough for OSQP to
// converge on the active constraint.
template <ampc::Formulation F> VectorXd checkSingleStepHorizon()
{
  const auto one_step{ampc::Parameterization::moveBlocking(1, 1)};
  ConstraintInspector mpc{2, 1, one_step, general_and_terminal, F};
  configure(mpc);
  mpc.setStateWeights(Vector2d{1e4, 1e3});
  // 0.3 u_0 <= 0.5 - Hx x_0, so u_0 <= 1 for the x0 below
  const RowVector2d Hx{1.0, 0.5};
  const Vector2d x0{0.1, 0.2};
  mpc.setGeneralConstraints(Hx, MatrixXd::Constant(1, 1, 0.3),
                            VectorXd::Constant(1, -inf),
                            VectorXd::Constant(1, 0.5));
  mpc.setTerminalConstraints(RowVector2d{0.0, 1.0}, VectorXd::Constant(1, -inf),
                             VectorXd::Constant(1, inf));
  EXPECT_TRUE(mpc.initializeSolver(polishedSettings()));
  EXPECT_EQ(mpc.rows().general_constraints.rows, 1);
  EXPECT_EQ(mpc.rows().terminal_constraints.rows, 1);

  EXPECT_EQ(mpc.solve(x0), ampc::SolveStatus::Success);
  // the one general row is Hu on the control point, with Hx x_0 in its bounds
  const int row{mpc.rows().general_constraints.offset};
  EXPECT_DOUBLE_EQ(mpc.A()(row, 0), 0.3);
  EXPECT_DOUBLE_EQ(mpc.u()(row), 0.5 - Hx.dot(x0));
  EXPECT_EQ(mpc.l()(row), -inf);
  VectorXd u0{1};
  mpc.getInputTrajectory(u0);
  EXPECT_NEAR(u0(0), 1.0, 1e-8);

  // a tighter bound on the velocity of x_1 takes over
  mpc.setTerminalConstraints(RowVector2d{0.0, 1.0}, VectorXd::Constant(1, -inf),
                             VectorXd::Constant(1, 0.205));
  EXPECT_EQ(mpc.solve(x0), ampc::SolveStatus::Success);
  Vector2d x1;
  mpc.getPredictedStateTrajectory(x1);
  mpc.getInputTrajectory(u0);
  EXPECT_NEAR(x1(1), 0.205, 1e-8);
  EXPECT_LT(u0(0), 1.0 - 1e-3);
  return u0;
}

TEST(GeneralConstraintsTester, givenSingleStepHorizon_ConstraintsAreActive)
{
  EXPECT_TRUE(expectEigenNear(
      checkSingleStepHorizon<ampc::Formulation::Condensed>(),
      checkSingleStepHorizon<ampc::Formulation::Sparse>(), 1e-8));
}

// ---- runtime changes --------------------------------------------------------

// New Hx, Hu, Hf (same nonzero pattern) applied by prepare() give exactly the
// QP of a freshly built MPC, and the same solution
template <ampc::Formulation F> void checkMatrixChangeMatchesFreshMPC()
{
  auto set = [](ampc::MPC& mpc, double s) {
    mpc.setGeneralConstraints(
        s * RowVector2d{1.0, 0.2}, MatrixXd::Constant(1, 1, 0.5 * s),
        VectorXd::Constant(1, -inf), VectorXd::Constant(1, 2.0));
    mpc.setTerminalConstraints(s * RowVector2d{1.0, 0.5},
                               VectorXd::Constant(1, -inf),
                               VectorXd::Constant(1, 1.5));
  };
  const Vector2d x0{0.1, 0.0};
  ConstraintInspector changed{2, 1, param, general_and_terminal, F};
  configure(changed);
  set(changed, 1.0);
  ASSERT_TRUE(changed.initializeSolver(polishedSettings()));
  ASSERT_EQ(changed.solve(x0), ampc::SolveStatus::Success);

  set(changed, 1.3);
  using ampc::detail::DataChange;
  EXPECT_EQ(changed.pending(),
            DataChange::ConstraintMatrices | DataChange::Limits);
  ASSERT_EQ(changed.prepare(), ampc::SolveStatus::Success);
  EXPECT_TRUE(changed.pending().empty());
  ASSERT_EQ(changed.solve(x0), ampc::SolveStatus::Success);
  EXPECT_TRUE(changed.isWithinSparsityPattern());

  ConstraintInspector fresh{2, 1, param, general_and_terminal, F};
  configure(fresh);
  set(fresh, 1.3);
  ASSERT_TRUE(fresh.initializeSolver(polishedSettings()));
  ASSERT_EQ(fresh.solve(x0), ampc::SolveStatus::Success);

  EXPECT_TRUE(expectEigenNear(changed.A(), fresh.A(), 0.0));
  EXPECT_TRUE(expectEigenNear(changed.l(), fresh.l(), 0.0));
  EXPECT_TRUE(expectEigenNear(changed.u(), fresh.u(), 0.0));
  VectorXd c_changed{param.num_control_points}, c_fresh{c_changed.size()};
  changed.getInputControlPoints(c_changed);
  fresh.getInputControlPoints(c_fresh);
  EXPECT_TRUE(expectEigenNear(c_changed, c_fresh, 1e-8));
}

TEST(GeneralConstraintsTester, givenNewMatrices_CondensedMatchesFreshMPC)
{
  checkMatrixChangeMatchesFreshMPC<ampc::Formulation::Condensed>();
}

TEST(GeneralConstraintsTester, givenNewMatrices_SparseMatchesFreshMPC)
{
  checkMatrixChangeMatchesFreshMPC<ampc::Formulation::Sparse>();
}

// The setters mark the matrices only when they change; bounds are limits
TEST(GeneralConstraintsTester, givenSetters_MarkMatricesOnlyWhenChanged)
{
  using ampc::detail::DataChange;
  ConstraintInspector mpc{2, 1, param, general_and_terminal};
  configure(mpc);
  const RowVector2d Hx{1.0, 0.0};
  const MatrixXd Hu{MatrixXd::Ones(1, 1)};
  const VectorXd lo{VectorXd::Constant(1, -1.0)};
  const VectorXd hi{VectorXd::Constant(1, 1.0)};
  mpc.setGeneralConstraints(Hx, Hu, lo, hi);
  mpc.setTerminalConstraints(Hx, lo, hi);
  mpc.assemble(Vector2d::Zero());
  ASSERT_TRUE(mpc.pending().empty());

  mpc.setGeneralConstraints(Hx, Hu, 2 * lo, 2 * hi);
  mpc.setTerminalConstraints(Hx, 2 * lo, 2 * hi);
  EXPECT_EQ(mpc.pending(), DataChange::Limits);
  mpc.assemble(Vector2d::Zero());

  mpc.setGeneralConstraints(Hx, 2 * Hu, lo, hi);
  EXPECT_EQ(mpc.pending(), DataChange::ConstraintMatrices | DataChange::Limits);
  mpc.assemble(Vector2d::Zero());
  mpc.setTerminalConstraints(2 * Hx, lo, hi);
  EXPECT_EQ(mpc.pending(), DataChange::ConstraintMatrices | DataChange::Limits);

  EXPECT_TRUE(expectEigenNear(mpc.getGeneralConstraintStateMatrix(), Hx, 0.0));
  EXPECT_TRUE(
      expectEigenNear(mpc.getGeneralConstraintInputMatrix(), 2 * Hu, 0.0));
  EXPECT_TRUE(expectEigenNear(mpc.getGeneralConstraintMin(), lo, 0.0));
  EXPECT_TRUE(expectEigenNear(mpc.getGeneralConstraintMax(), hi, 0.0));
  EXPECT_TRUE(expectEigenNear(mpc.getTerminalConstraintMatrix(), 2 * Hx, 0.0));
  EXPECT_TRUE(expectEigenNear(mpc.getTerminalConstraintMin(), lo, 0.0));
  EXPECT_TRUE(expectEigenNear(mpc.getTerminalConstraintMax(), hi, 0.0));
}

// ---- layout, validation, and printing ---------------------------------------

TEST(GeneralConstraintsTester, givenOptions_AddRowsAfterTheOtherBlocks)
{
  const ampc::Options opts{.saturate_states = true,
                           .num_general_constraints = 3,
                           .num_terminal_constraints = 2};
  ConstraintInspector mpc{2, 1, param, opts, ampc::Formulation::Sparse};
  const ampc::detail::QPRows& rows{mpc.rows()};
  EXPECT_EQ(rows.general_constraints.offset,
            rows.state_bounds.offset + rows.state_bounds.rows);
  EXPECT_EQ(rows.general_constraints.rows, 3 * T);
  EXPECT_EQ(rows.terminal_constraints.offset,
            rows.general_constraints.offset + 3 * T);
  EXPECT_EQ(rows.terminal_constraints.rows, 2);
  EXPECT_EQ(mpc.A().rows(), rows.terminal_constraints.offset + 2);
}

TEST(GeneralConstraintsTester, givenInvalidArguments_Throws)
{
  EXPECT_THROW(
      (ampc::MPC{2, 1, param, ampc::Options{.num_general_constraints = -1}}),
      std::invalid_argument);
  EXPECT_THROW(
      (ampc::MPC{2, 1, param, ampc::Options{.num_terminal_constraints = -1},
                 ampc::Formulation::Sparse}),
      std::invalid_argument);

  // disabled
  ampc::MPC plain{2, 1, param};
  const RowVector2d Hx{1.0, 0.0};
  const MatrixXd Hu{MatrixXd::Zero(1, 1)};
  const VectorXd lo{VectorXd::Constant(1, -1.0)};
  const VectorXd hi{VectorXd::Constant(1, 1.0)};
  auto expectNotEnabled = [](auto&& call) {
    try {
      call();
      ADD_FAILURE() << "expected std::logic_error";
    } catch (const std::logic_error& e) {
      EXPECT_NE(std::string{e.what()}.find("not enabled"), std::string::npos)
          << e.what();
    }
  };
  expectNotEnabled([&] { plain.setGeneralConstraints(Hx, Hu, lo, hi); });
  expectNotEnabled([&] { plain.setTerminalConstraints(Hx, lo, hi); });
  expectNotEnabled([&] { (void)plain.getGeneralConstraintStateMatrix(); });
  expectNotEnabled([&] { (void)plain.getGeneralConstraintInputMatrix(); });
  expectNotEnabled([&] { (void)plain.getGeneralConstraintMin(); });
  expectNotEnabled([&] { (void)plain.getGeneralConstraintMax(); });
  expectNotEnabled([&] { (void)plain.getTerminalConstraintMatrix(); });
  expectNotEnabled([&] { (void)plain.getTerminalConstraintMin(); });
  expectNotEnabled([&] { (void)plain.getTerminalConstraintMax(); });

  // sizes and bound order
  ampc::MPC mpc{2, 1, param, general_and_terminal, ampc::Formulation::Sparse};
  EXPECT_THROW(mpc.setGeneralConstraints(Matrix2d::Identity(), Hu, lo, hi),
               std::invalid_argument);
  EXPECT_THROW(mpc.setGeneralConstraints(Hx, MatrixXd::Zero(1, 2), lo, hi),
               std::invalid_argument);
  EXPECT_THROW(mpc.setGeneralConstraints(Hx, Hu, Vector2d::Zero(), hi),
               std::invalid_argument);
  EXPECT_THROW(mpc.setGeneralConstraints(Hx, Hu, lo, Vector2d::Zero()),
               std::invalid_argument);
  EXPECT_THROW(mpc.setGeneralConstraints(Hx, Hu, hi, lo),
               std::invalid_argument);
  EXPECT_THROW(mpc.setTerminalConstraints(RowVector3d::Zero(), lo, hi),
               std::invalid_argument);
  EXPECT_THROW(mpc.setTerminalConstraints(Hx, Vector2d::Zero(), hi),
               std::invalid_argument);
  EXPECT_THROW(mpc.setTerminalConstraints(Hx, hi, lo), std::invalid_argument);
  // infinite bounds are allowed
  mpc.setGeneralConstraints(Hx, Hu, VectorXd::Constant(1, -inf),
                            VectorXd::Constant(1, inf));

  // each setter is required before initializeSolver()
  configure(mpc);
  EXPECT_THROW((void)mpc.initializeSolver(), std::logic_error);
  mpc.setTerminalConstraints(Hx, lo, hi);
  EXPECT_TRUE(mpc.initializeSolver());

  ampc::MPC no_general{2, 1, param, general_only};
  configure(no_general);
  EXPECT_THROW((void)no_general.initializeSolver(), std::logic_error);
}

TEST(GeneralConstraintsTester, givenConstraints_PrintsThem)
{
  ampc::MPC mpc{2, 1, ampc::Parameterization::linearInterp(4, 3),
                general_and_terminal};
  mpc.setGeneralConstraints(RowVector2d{1, 2}, MatrixXd::Constant(1, 1, 3),
                            VectorXd::Constant(1, -4),
                            VectorXd::Constant(1, 5));
  mpc.setTerminalConstraints(RowVector2d{6, 7}, VectorXd::Constant(1, -8),
                             VectorXd::Constant(1, 9));
  std::ostringstream inline_os, multiline_os;
  printInline(inline_os, mpc);
  print(multiline_os, mpc);
  EXPECT_NE(inline_os.str().find("num_general_constraints=1, "
                                 "num_terminal_constraints=1)"),
            std::string::npos);
  EXPECT_NE(inline_os.str().find(", Hx=[[1, 2]], Hu=[[3]], h_min=[-4], "
                                 "h_max=[5], Hf=[[6, 7]], hf_min=[-8], "
                                 "hf_max=[9])"),
            std::string::npos)
      << inline_os.str();
  EXPECT_NE(
      multiline_os.str().find("\n  Hx = [[1, 2]]\n  Hu = [[3]]\n  h_min = "
                              "[-4]\n  h_max = [5]\n  Hf = [[6, 7]]\n  "
                              "hf_min = [-8]\n  hf_max = [9]"),
      std::string::npos)
      << multiline_os.str();
}

} // namespace
