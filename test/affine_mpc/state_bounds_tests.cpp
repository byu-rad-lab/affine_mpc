#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <gtest/gtest.h>
#include <limits>
#include <vector>

#include "affine_mpc/condensed_mpc.hpp"
#include "affine_mpc/options.hpp"
#include "affine_mpc/osqp_solver.hpp"
#include "affine_mpc/parameterization.hpp"
#include "affine_mpc/sparse_mpc.hpp"
#include "utils.hpp"

namespace ampc = affine_mpc;
using namespace Eigen;

namespace {

const double inf{std::numeric_limits<double>::infinity()};

// Exposes the assembled QP and the slack part of the solution
template <class Base> class BoundsInspector : public Base
{
public:
  using Base::Base;
  void assemble(const Ref<const VectorXd>& x0) { (void)this->qpUpdateX0(x0); }
  const MatrixXd& P() const { return this->qp().P; }
  const MatrixXd& A() const { return this->qp().A; }
  const VectorXd& q() const { return this->qp().q; }
  const VectorXd& l() const { return this->qp().l; }
  const VectorXd& u() const { return this->qp().u; }
  // the slacks are the last QP variables
  VectorXd slack() const
  {
    const auto k{static_cast<Index>(this->getBoundedStates().size())};
    return this->solution_map_.tail(k);
  }
};

// Mass-spring-damper (position, velocity) asked to go past its position limit
void configure(ampc::MPCBase& mpc)
{
  Matrix2d Ac;
  Ac << 0, 1, -0.6, -0.1;
  mpc.setModelContinuous2Discrete(Ac, Vector2d{0, 0.2}, Vector2d::Zero(), 0.1);
  mpc.setStateWeights(Vector2d{1.0, 0.1});
  mpc.setReferenceState(Vector2d{1.0, 0.0});
  mpc.setInputLimits(VectorXd::Constant(1, -3.0), VectorXd::Constant(1, 3.0));
}

const auto param{ampc::Parameterization::bspline(20, 2, 8)};
const VectorXd x_lo{VectorXd::Constant(1, -0.5)};
const VectorXd x_hi{VectorXd::Constant(1, 0.5)};
const ampc::Options position_only{.saturate_states = true,
                                  .bounded_states = {0}};
const ampc::Options soft_position{
    .saturate_states = true, .soft_state_bounds = true, .bounded_states = {0}};

// Polishing makes the solutions of equivalent QPs agree to round-off
bool initialize(ampc::MPCBase& mpc)
{
  OSQPSettings settings{ampc::OSQPSolver::getRecommendedSettings()};
  settings.polishing = true;
  settings.max_iter = 20000;
  return mpc.initializeSolver(settings);
}

VectorXd solveControlPoints(ampc::MPCBase& mpc, const Vector2d& x0)
{
  EXPECT_TRUE(initialize(mpc));
  EXPECT_EQ(mpc.solve(x0), ampc::SolveStatus::Success);
  VectorXd c{param.num_control_points};
  mpc.getInputControlPoints(c);
  return c;
}

// Largest violation of the position limit by the predicted states
double positionViolation(const ampc::MPCBase& mpc)
{
  VectorXd x_traj{2 * param.horizon_steps};
  mpc.getPredictedStateTrajectory(x_traj);
  double violation{0.0};
  for (int k{0}; k < param.horizon_steps; ++k)
    violation = std::max(violation, std::abs(x_traj(2 * k)) - 0.5);
  return violation;
}

// Limits on the position only equal limits on both states with the velocity
// unbounded, with one row per step fewer
template <class MPC> void checkSubsetMatchesInfiniteLimits()
{
  BoundsInspector<MPC> full{2, 1, param,
                            ampc::Options{.saturate_states = true}};
  configure(full);
  full.setStateLimits(Vector2d{-0.5, -inf}, Vector2d{0.5, inf});
  BoundsInspector<MPC> partial{2, 1, param, position_only};
  configure(partial);
  partial.setStateLimits(x_lo, x_hi);

  const VectorXd c_full{solveControlPoints(full, Vector2d::Zero())};
  const VectorXd c_partial{solveControlPoints(partial, Vector2d::Zero())};
  EXPECT_TRUE(expectEigenNear(c_partial, c_full, 1e-8));
  EXPECT_EQ(full.A().rows() - partial.A().rows(), param.horizon_steps);
  // the limit is active, so the comparison means something
  EXPECT_NEAR(positionViolation(partial), 0.0, 1e-8);
  VectorXd x_traj{2 * param.horizon_steps};
  partial.getPredictedStateTrajectory(x_traj);
  EXPECT_NEAR(x_traj(Eigen::seqN(0, param.horizon_steps, 2)).maxCoeff(), 0.5,
              1e-8);
}

// Soft limits that can be met give the hard solution with zero slack when the
// L1 penalty is large enough (exact penalty); the L2 penalty does not change
// that
template <class MPC> void checkExactWhenFeasible()
{
  MPC hard{2, 1, param, position_only};
  configure(hard);
  hard.setStateLimits(x_lo, x_hi);
  BoundsInspector<MPC> soft{2, 1, param, soft_position};
  configure(soft);
  soft.setStateLimits(x_lo, x_hi);
  soft.setStateBoundPenalties(VectorXd::Constant(1, 10.0),
                              VectorXd::Constant(1, 100.0));

  const VectorXd c_hard{solveControlPoints(hard, Vector2d::Zero())};
  const VectorXd c_soft{solveControlPoints(soft, Vector2d::Zero())};
  EXPECT_TRUE(expectEigenNear(c_soft, c_hard, 1e-7));
  EXPECT_LT(soft.slack().cwiseAbs().maxCoeff(), 1e-8);
}

// Starting past the limit, the hard problem is infeasible; the soft one
// solves, and the slack is the largest violation over the horizon
template <class MPC> void checkSoftRecoversInfeasible()
{
  const Vector2d x_outside{0.9, 0.0};
  MPC hard{2, 1, param, position_only};
  configure(hard);
  hard.setStateLimits(x_lo, x_hi);
  ASSERT_TRUE(initialize(hard));
  EXPECT_EQ(hard.solve(x_outside), ampc::SolveStatus::PrimalInfeasible);

  BoundsInspector<MPC> soft{2, 1, param, soft_position};
  configure(soft);
  soft.setStateLimits(x_lo, x_hi);
  soft.setStateBoundPenalties(VectorXd::Constant(1, 10.0),
                              VectorXd::Constant(1, 100.0));
  (void)solveControlPoints(soft, x_outside);
  EXPECT_GT(soft.slack()(0), 0.1);
  EXPECT_NEAR(soft.slack()(0), positionViolation(soft), 1e-7);
}

// An L2 penalty alone lets an active limit move a little, less for a larger
// penalty
template <class MPC> double l2Violation(double l2)
{
  BoundsInspector<MPC> mpc{2, 1, param, soft_position};
  configure(mpc);
  mpc.setStateLimits(x_lo, x_hi);
  mpc.setStateBoundPenalties(VectorXd::Zero(1), VectorXd::Constant(1, l2));
  (void)solveControlPoints(mpc, Vector2d::Zero());
  EXPECT_NEAR(mpc.slack()(0), positionViolation(mpc), 1e-7);
  return mpc.slack()(0);
}

// The assembled P, A, q, l, u
template <class MPC>
std::vector<MatrixXd> assembledQP(const ampc::Options& opts)
{
  BoundsInspector<MPC> mpc{2, 1, param, opts};
  configure(mpc);
  mpc.setStateLimits(Vector2d::Constant(-0.5), Vector2d::Constant(0.5));
  mpc.assemble(Vector2d::Zero());
  return {mpc.P(), mpc.A(), mpc.q(), mpc.l(), mpc.u()};
}

// Listing every state is the same as the default (all states)
template <class MPC> void checkExplicitAllMatchesDefault()
{
  const auto implicit{assembledQP<MPC>({.saturate_states = true})};
  const auto explicit_all{
      assembledQP<MPC>({.saturate_states = true, .bounded_states = {0, 1}})};
  for (size_t i{0}; i < implicit.size(); ++i)
    EXPECT_TRUE(expectEigenNear(explicit_all[i], implicit[i], 0.0)) << i;
}

} // namespace

TEST(StateBoundsTester, givenAllStatesSelected_QPMatchesDefaultBitwise)
{
  checkExplicitAllMatchesDefault<ampc::CondensedMPC>();
  checkExplicitAllMatchesDefault<ampc::SparseMPC>();
}

TEST(StateBoundsTester, givenSubsetOfStates_CondensedMatchesInfiniteLimits)
{
  checkSubsetMatchesInfiniteLimits<ampc::CondensedMPC>();
}

TEST(StateBoundsTester, givenSubsetOfStates_SparseMatchesInfiniteLimits)
{
  checkSubsetMatchesInfiniteLimits<ampc::SparseMPC>();
}

TEST(StateBoundsTester, givenFeasibleLimits_CondensedSoftMatchesHard)
{
  checkExactWhenFeasible<ampc::CondensedMPC>();
}

TEST(StateBoundsTester, givenFeasibleLimits_SparseSoftMatchesHard)
{
  checkExactWhenFeasible<ampc::SparseMPC>();
}

TEST(StateBoundsTester, givenInfeasibleLimits_CondensedSoftSolves)
{
  checkSoftRecoversInfeasible<ampc::CondensedMPC>();
}

TEST(StateBoundsTester, givenInfeasibleLimits_SparseSoftSolves)
{
  checkSoftRecoversInfeasible<ampc::SparseMPC>();
}

TEST(StateBoundsTester, givenL2PenaltyOnly_ViolationShrinksWithPenalty)
{
  for (const bool sparse : {false, true}) {
    SCOPED_TRACE(sparse ? "sparse" : "condensed");
    const double loose{sparse ? l2Violation<ampc::SparseMPC>(1.0)
                              : l2Violation<ampc::CondensedMPC>(1.0)};
    const double tight{sparse ? l2Violation<ampc::SparseMPC>(100.0)
                              : l2Violation<ampc::CondensedMPC>(100.0)};
    EXPECT_GT(loose, 1e-3);
    EXPECT_LT(tight, 0.5 * loose);
  }
}

TEST(StateBoundsTester, givenInvalidOptions_Throws)
{
  auto make = [](const ampc::Options& opts) {
    ampc::CondensedMPC mpc{2, 1, param, opts};
  };
  expectInvalidArgumentWithMessage([&]() { make({.bounded_states = {0}}); },
                                   "require saturate_states");
  expectInvalidArgumentWithMessage([&]() { make({.soft_state_bounds = true}); },
                                   "require saturate_states");
  for (const std::vector<int>& bad :
       {std::vector<int>{1, 0}, std::vector<int>{0, 0}, std::vector<int>{2},
        std::vector<int>{-1}}) {
    expectInvalidArgumentWithMessage(
        [&]() { make({.saturate_states = true, .bounded_states = bad}); },
        "strictly increasing");
  }
}

TEST(StateBoundsTester, givenSoftBoundsWithoutPenalties_InitializeThrows)
{
  ampc::CondensedMPC mpc{2, 1, param, soft_position};
  configure(mpc);
  mpc.setStateLimits(x_lo, x_hi);
  expectLogicErrorWithMessage([&]() { (void)mpc.initializeSolver(); },
                              "penalties must be set");

  expectInvalidArgumentWithMessage(
      [&]() { mpc.setStateLimits(Vector2d::Zero(), Vector2d::Ones()); },
      "must have size 1");
  expectInvalidArgumentWithMessage(
      [&]() {
        mpc.setStateBoundPenalties(VectorXd::Constant(1, -1.0),
                                   VectorXd::Zero(1));
      },
      "non-negative");
  mpc.setStateBoundPenalties(VectorXd::Constant(1, 2.0),
                             VectorXd::Constant(1, 3.0));
  EXPECT_EQ(mpc.getBoundedStates(), std::vector<int>{0});
  EXPECT_EQ(mpc.getStateBoundL1Penalties()(0), 2.0);
  EXPECT_EQ(mpc.getStateBoundL2Penalties()(0), 3.0);
  EXPECT_TRUE(mpc.initializeSolver());

  ampc::CondensedMPC hard{2, 1, param, ampc::Options{.saturate_states = true}};
  expectLogicErrorWithMessage(
      [&]() {
        hard.setStateBoundPenalties(Vector2d::Ones(), Vector2d::Ones());
      },
      "Soft state bounds are not enabled");
  EXPECT_EQ(hard.getBoundedStates(), (std::vector<int>{0, 1}));
}
