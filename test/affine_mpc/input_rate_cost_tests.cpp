#include <Eigen/Core>
#include <gtest/gtest.h>
#include <string>
#include <utility>
#include <vector>

#include "affine_mpc/mpc.hpp"
#include "affine_mpc/options.hpp"
#include "affine_mpc/osqp_solver.hpp"
#include "affine_mpc/parameterization.hpp"
#include "utils.hpp"

namespace ampc = affine_mpc;
using namespace Eigen;

namespace {

// Keeps the QP assembled with the public assembleQP()
class RateInspector : public ampc::MPC
{
public:
  using ampc::MPC::MPC;
  void assemble(const Ref<const VectorXd>& x0) { assembleQP(x0, qp_); }
  MatrixXd P() const { return MatrixXd{qp_.P}; }
  const VectorXd& q() const { return qp_.q; }

private:
  ampc::QuadraticProgram qp_;
};

// Two-input system with every entry of Ad and Bd nonzero
void configure(ampc::MPC& mpc)
{
  Matrix2d Ad;
  Ad << 0.997, 0.0998, -0.0599, 0.987;
  Matrix2d Bd;
  Bd << 0.001, 0.003, 0.02, -0.01;
  mpc.setModelDiscrete(Ad, Bd, Vector2d{0.002, -0.01});
  mpc.setStateWeights(Vector2d{1.0, 0.2}, Vector2d{3.0, 0.4});
  mpc.setReferenceState(Vector2d{1.0, 0.0});
  mpc.setInputLimits(Vector2d::Constant(-2.0), Vector2d::Constant(2.0));
}

// 1/2 sum_k (u_k - u_{k-1})^T diag(R_rate) (u_k - u_{k-1}), u_{-1} = u_prev,
// summed directly over the sampled inputs
double rateCost(const ampc::Parameterization& param,
                const VectorXd& control_points,
                const Vector2d& R_rate,
                const Vector2d& u_prev)
{
  const int m{2};
  const VectorXd u{param.evaluate(m, control_points)};
  double cost{0.0};
  Vector2d previous{u_prev};
  for (int k{0}; k < param.horizon_steps; ++k) {
    const Vector2d du{u.segment<2>(k * m) - previous};
    cost += 0.5 * du.dot(R_rate.cwiseProduct(du));
    previous = u.segment<2>(k * m);
  }
  return cost;
}

std::vector<ampc::Parameterization> bases()
{
  using P = ampc::Parameterization;
  return {
      P::moveBlocking(10, 10),   P::moveBlocking(12, 4, 1.3),
      P::linearInterp(10, 4),    P::bspline(15, 2, 6),
      P::bspline(20, 3, 7, 0.8), P::fromBasis(12, laguerreBasis(12, 4, 0.6))};
}

// The QP terms added by the rate cost, compared with the cost summed over
// the sampled inputs: 1/2 c^T H c + g^T c + const must equal it for any c
template <ampc::Formulation F> void checkAgainstDirectSum()
{
  const Vector2d R_rate{0.7, 1.9}, u_prev{0.3, -0.4};
  const Vector2d x0{0.5, -0.2};
  for (const ampc::Parameterization& param : bases()) {
    SCOPED_TRACE("degree " + std::to_string(param.degree) + ", T "
                 + std::to_string(param.horizon_steps));
    RateInspector with_rate{2, 2, param,
                            ampc::Options{.use_input_rate_cost = true}, F};
    RateInspector without{2, 2, param, F};
    configure(with_rate);
    configure(without);
    with_rate.setInputRateWeights(R_rate);
    with_rate.setPreviousInput(u_prev);
    with_rate.assemble(x0);
    without.assemble(x0);

    const int nc{2 * param.num_control_points};
    const MatrixXd H{with_rate.P().topLeftCorner(nc, nc)
                     - without.P().topLeftCorner(nc, nc)};
    const VectorXd g{with_rate.q().head(nc) - without.q().head(nc)};
    // the rate cost only acts on the control points
    EXPECT_TRUE(
        expectEigenNear(with_rate.P().bottomRightCorner(
                            without.P().rows() - nc, without.P().cols() - nc),
                        without.P().bottomRightCorner(without.P().rows() - nc,
                                                      without.P().cols() - nc),
                        1e-12));
    const double constant{0.5 * u_prev.dot(R_rate.cwiseProduct(u_prev))};

    std::srand(11);
    for (int trial{0}; trial < 3; ++trial) {
      const VectorXd c{VectorXd::Random(nc)};
      const double qp_cost{0.5 * c.dot(H * c) + g.dot(c) + constant};
      EXPECT_NEAR(qp_cost, rateCost(param, c, R_rate, u_prev), 1e-10);
    }
  }
}

// Total variation of the applied inputs, including the step from u_prev
template <ampc::Formulation F> double inputVariation(double rate_weight)
{
  const auto param{ampc::Parameterization::bspline(20, 2, 8)};
  ampc::MPC mpc{2, 2, param, ampc::Options{.use_input_rate_cost = true}, F};
  configure(mpc);
  mpc.setInputRateWeights(Vector2d::Constant(rate_weight));
  EXPECT_TRUE(mpc.initializeSolver());
  EXPECT_EQ(mpc.solve(Vector2d{0.0, 0.0}), ampc::SolveStatus::Success);
  VectorXd u{2 * param.horizon_steps};
  mpc.getInputTrajectory(u);
  double variation{u.head<2>().lpNorm<1>()}; // u_prev is zero
  for (int k{1}; k < param.horizon_steps; ++k)
    variation += (u.segment<2>(2 * k) - u.segment<2>(2 * (k - 1))).lpNorm<1>();
  return variation;
}

} // namespace

TEST(InputRateCostTester, givenCondensedMPC_QPMatchesCostSummedOverInputs)
{
  checkAgainstDirectSum<ampc::Formulation::Condensed>();
}

TEST(InputRateCostTester, givenSparseMPC_QPMatchesCostSummedOverInputs)
{
  checkAgainstDirectSum<ampc::Formulation::Sparse>();
}

TEST(InputRateCostTester, givenLargerRateWeight_InputsChangeLess)
{
  for (const bool sparse : {false, true}) {
    SCOPED_TRACE(sparse ? "sparse" : "condensed");
    const double smooth{
        sparse ? inputVariation<ampc::Formulation::Sparse>(100.0)
               : inputVariation<ampc::Formulation::Condensed>(100.0)};
    const double rough{
        sparse ? inputVariation<ampc::Formulation::Sparse>(0.01)
               : inputVariation<ampc::Formulation::Condensed>(0.01)};
    EXPECT_LT(smooth, 0.5 * rough);
  }
}

TEST(InputRateCostTester, givenRateCostOnly_PreviousInputFollowsSolves)
{
  const auto param{ampc::Parameterization::linearInterp(10, 4)};
  ampc::MPC mpc{2, 2, param, ampc::Options{.use_input_rate_cost = true}};
  configure(mpc);
  ASSERT_TRUE(mpc.initializeSolver());
  ASSERT_TRUE(mpc.getPreviousInput().isZero());
  ASSERT_EQ(mpc.solve(Vector2d{0.5, 0.0}), ampc::SolveStatus::Success);
  Vector2d u0;
  mpc.getNextInput(u0);
  EXPECT_TRUE(expectEigenNear(mpc.getPreviousInput(), u0, 0.0));
}

TEST(InputRateCostTester, givenRateCost_CondensedAndSparseAgree)
{
  const auto param{ampc::Parameterization::bspline(15, 2, 6)};
  const ampc::Options opts{.use_input_rate_cost = true};
  ampc::MPC condensed{2, 2, param, opts};
  ampc::MPC sparse{2, 2, param, opts, ampc::Formulation::Sparse};
  Vector2d x{0.5, -0.2};
  for (ampc::MPC* mpc : {static_cast<ampc::MPC*>(&condensed),
                         static_cast<ampc::MPC*>(&sparse)}) {
    configure(*mpc);
    mpc->setInputRateWeights(Vector2d{0.7, 1.9});
    mpc->setPreviousInput(Vector2d{0.3, -0.4});
    ASSERT_TRUE(mpc->initializeSolver());
  }
  // a few closed-loop steps, so u_prev comes from the previous solve
  for (int k{0}; k < 5; ++k) {
    ASSERT_EQ(condensed.solve(x), ampc::SolveStatus::Success);
    ASSERT_EQ(sparse.solve(x), ampc::SolveStatus::Success);
    Vector2d u_condensed, u_sparse, x_next;
    condensed.getNextInput(u_condensed);
    sparse.getNextInput(u_sparse);
    ASSERT_TRUE(expectEigenNear(u_condensed, u_sparse, 1e-4));
    condensed.propagateModel(x, u_condensed, x_next);
    x = x_next;
  }
}

TEST(InputRateCostTester, givenCustomBasisOfBspline_ReproducesBsplineSolution)
{
  // A custom basis equal to the B-spline's sampled basis gives the same rate
  // cost (dense band: G and the u_0 entries over every coefficient) and bounds
  // every sample, as the B-spline does with saturate_input_trajectory
  const auto bspline{ampc::Parameterization::bspline(15, 2, 6, 1.3)};
  const auto custom{ampc::Parameterization::fromBasis(15, denseBasis(bspline))};
  const ampc::Options opts{.use_input_rate_cost = true};
  ampc::Options bspline_opts{opts};
  bspline_opts.saturate_input_trajectory = true;

  ampc::MPC condensed_b{2, 2, bspline, bspline_opts};
  ampc::MPC condensed_c{2, 2, custom, opts};
  ampc::MPC sparse_b{2, 2, bspline, bspline_opts, ampc::Formulation::Sparse};
  ampc::MPC sparse_c{2, 2, custom, opts, ampc::Formulation::Sparse};
  auto settings{ampc::OSQPSolver::getRecommendedSettings()};
  settings.polishing = true;
  settings.max_iter = 20000;
  for (ampc::MPC* mpc : {static_cast<ampc::MPC*>(&condensed_b),
                         static_cast<ampc::MPC*>(&condensed_c),
                         static_cast<ampc::MPC*>(&sparse_b),
                         static_cast<ampc::MPC*>(&sparse_c)}) {
    configure(*mpc);
    mpc->setInputRateWeights(Vector2d{0.7, 1.9});
    mpc->setPreviousInput(Vector2d{0.3, -0.4});
    ASSERT_TRUE(mpc->initializeSolver(settings));
  }

  const Vector2d x0{0.5, -0.2};
  const int nc{2 * bspline.num_control_points};
  VectorXd c_b{nc}, c_c{nc};
  for (auto [b, c] :
       {std::pair<ampc::MPC*, ampc::MPC*>{&condensed_b, &condensed_c},
        std::pair<ampc::MPC*, ampc::MPC*>{&sparse_b, &sparse_c}}) {
    ASSERT_EQ(b->solve(x0), ampc::SolveStatus::Success);
    ASSERT_EQ(c->solve(x0), ampc::SolveStatus::Success);
    b->getInputControlPoints(c_b);
    c->getInputControlPoints(c_c);
    EXPECT_TRUE(expectEigenNear(c_c, c_b, 1e-6));
  }
}

TEST(InputRateCostTester,
     givenZeroRateWeightAtInit_SparseStaysInPatternWhenItBecomesNonzero)
{
  const auto param{ampc::Parameterization::bspline(15, 2, 6)};
  const ampc::Options opts{.use_input_rate_cost = true};
  RateInspector mpc{2, 2, param, opts, ampc::Formulation::Sparse};
  configure(mpc);
  mpc.setInputRateWeights(Vector2d::Zero());
  ASSERT_TRUE(mpc.initializeSolver());
  mpc.setInputRateWeights(Vector2d{0.7, 1.9});
  const Vector2d x0{0.5, -0.2};
  ASSERT_EQ(mpc.solve(x0), ampc::SolveStatus::Success);
  EXPECT_TRUE(mpc.isWithinSparsityPattern());

  // the same weights from the start give the same solution
  ampc::MPC fresh{2, 2, param, opts, ampc::Formulation::Sparse};
  configure(fresh);
  fresh.setInputRateWeights(Vector2d{0.7, 1.9});
  ASSERT_TRUE(fresh.initializeSolver());
  ASSERT_EQ(fresh.solve(x0), ampc::SolveStatus::Success);
  VectorXd c{2 * param.num_control_points}, c_fresh{c.size()};
  mpc.getInputControlPoints(c);
  fresh.getInputControlPoints(c_fresh);
  EXPECT_TRUE(expectEigenNear(c, c_fresh, 1e-5));
}

TEST(InputRateCostTester, givenInvalidRateWeights_Throws)
{
  const auto param{ampc::Parameterization::linearInterp(10, 4)};
  ampc::MPC disabled{2, 2, param};
  expectLogicErrorWithMessage(
      [&]() { disabled.setInputRateWeights(Vector2d::Ones()); },
      "Input rate cost is not enabled");

  ampc::MPC mpc{2, 2, param, ampc::Options{.use_input_rate_cost = true}};
  EXPECT_TRUE(
      expectEigenNear(mpc.getInputRateWeights(), Vector2d::Ones(), 0.0));
  expectInvalidArgumentWithMessage(
      [&]() { mpc.setInputRateWeights(VectorXd::Ones(3)); }, "must have size");
  expectInvalidArgumentWithMessage(
      [&]() { mpc.setInputRateWeights(Vector2d{1.0, -1.0}); },
      "must be non-negative");
}
