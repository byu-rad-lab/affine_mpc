#include <Eigen/Core>
#include <cstdlib>
#include <gtest/gtest.h>
#include <sstream>
#include <string>
#include <vector>

#include "affine_mpc/mpc.hpp"
#include "affine_mpc/options.hpp"
#include "affine_mpc/osqp_solver.hpp"
#include "affine_mpc/parameterization.hpp"
#include "utils.hpp"

namespace ampc = affine_mpc;
using namespace Eigen;

namespace {

constexpr ampc::InputCostMode Trajectory{ampc::InputCostMode::Trajectory};
constexpr ampc::InputCostMode ControlPoints{ampc::InputCostMode::ControlPoints};

// Keeps the QP assembled with the public assembleQP()
class CostInspector : public ampc::MPC
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

// Degrees 0-3, move-blocking with unequal blocks, geometric knots, and a
// custom basis
std::vector<ampc::Parameterization> bases()
{
  using P = ampc::Parameterization;
  return {
      P::moveBlocking(10, 10),   P::moveBlocking(12, 4, 1.3),
      P::bspline(10, 0, 4),      P::linearInterp(10, 4),
      P::bspline(15, 2, 6),      P::bspline(20, 3, 7, 0.8),
      P::bspline(16, 2, 6, 1.2), P::fromBasis(12, laguerreBasis(12, 4, 0.6))};
}

std::string name(const ampc::Parameterization& param)
{
  return "degree " + std::to_string(param.degree) + ", T "
         + std::to_string(param.horizon_steps) + ", nc "
         + std::to_string(param.num_control_points);
}

// A reference outside the span of every basis above
VectorXd referenceTrajectory(int T)
{
  VectorXd u_ref{2 * T};
  for (int k{0}; k < T; ++k) {
    u_ref(2 * k) = 0.5 * std::sin(0.7 * k);
    u_ref(2 * k + 1) = -0.3 + 0.05 * k * k / T;
  }
  return u_ref;
}

// The control point block of P and the linear term the trajectory cost adds,
// built directly from the dense basis: (Phi^T Phi) (x) diag(R) in the
// interleaved layout, and -Phi^T R u_ref per channel
void directTerms(const ampc::Parameterization& param,
                 const Vector2d& R,
                 const VectorXd& u_ref,
                 MatrixXd& H,
                 VectorXd& g)
{
  const int m{2}, ncp{param.num_control_points}, T{param.horizon_steps};
  const MatrixXd Phi{denseBasis(param)};
  const MatrixXd G{Phi.transpose() * Phi};
  H.setZero(m * ncp, m * ncp);
  g.setZero(m * ncp);
  for (int ch{0}; ch < m; ++ch) {
    for (int i{0}; i < ncp; ++i) {
      for (int j{0}; j < ncp; ++j)
        H(m * i + ch, m * j + ch) = G(i, j) * R(ch);
      for (int k{0}; k < T; ++k)
        g(m * i + ch) -= Phi(k, i) * R(ch) * u_ref(m * k + ch);
    }
  }
}

template <ampc::Formulation F> void checkAgainstDirectConstruction()
{
  const Vector2d R{0.7, 1.9}, x0{0.5, -0.2};
  for (const ampc::Parameterization& param : bases()) {
    SCOPED_TRACE(name(param));
    const VectorXd u_ref{referenceTrajectory(param.horizon_steps)};
    CostInspector with_cost{2, 2, param, {.input_cost = Trajectory}, F};
    CostInspector without{2, 2, param, F};
    configure(with_cost);
    configure(without);
    with_cost.setInputWeights(R);
    with_cost.setReferenceInputTrajectory(u_ref);
    with_cost.assemble(x0);
    without.assemble(x0);

    const int nc{2 * param.num_control_points};
    MatrixXd H_direct;
    VectorXd g_direct;
    directTerms(param, R, u_ref, H_direct, g_direct);
    const MatrixXd H{with_cost.P().topLeftCorner(nc, nc)
                     - without.P().topLeftCorner(nc, nc)};
    const VectorXd g{with_cost.q().head(nc) - without.q().head(nc)};
    EXPECT_TRUE(expectEigenNear(H, H_direct, 1e-12));
    EXPECT_TRUE(expectEigenNear(g, g_direct, 1e-12));
    // the cost only acts on the control points
    const Index rest{without.P().rows() - nc};
    EXPECT_TRUE(expectEigenNear(with_cost.P().bottomRightCorner(rest, rest),
                                without.P().bottomRightCorner(rest, rest),
                                0.0));
    EXPECT_TRUE(
        expectEigenNear(with_cost.q().tail(rest), without.q().tail(rest), 0.0));
  }
}

// 1/2 sum_k (x_k - x_ref,k)^T Q_k (x_k - x_ref,k)
//   + 1/2 sum_k (u_k - u_ref,k)^T R (u_k - u_ref,k), summed directly over the
// states x_1, ..., x_T (from x0 and the inputs) and the sampled inputs
double directObjective(const ampc::MPC& mpc,
                       const VectorXd& x0,
                       const VectorXd& u,
                       const Vector2d& R,
                       const VectorXd& u_ref,
                       VectorXd& x_traj)
{
  const int n{2}, m{2}, T{mpc.getHorizonSteps()};
  const VectorXd& x_ref{mpc.getReferenceStateTrajectory()};
  const VectorXd Q{mpc.getStateWeights()}, Qf{mpc.getTerminalStateWeights()};
  x_traj.resize(n * T);
  VectorXd x{x0}, x_next{n};
  double cost{0.0};
  for (int k{0}; k < T; ++k) {
    const VectorXd du{u.segment(m * k, m) - u_ref.segment(m * k, m)};
    cost += 0.5 * du.dot(R.cwiseProduct(du));
    mpc.propagateModel(x, u.segment(m * k, m), x_next);
    x = x_next;
    x_traj.segment(n * k, n) = x;
    const VectorXd dx{x - x_ref.segment(n * k, n)};
    cost += 0.5 * dx.dot((k == T - 1 ? Qf : Q).cwiseProduct(dx));
  }
  return cost;
}

// The QP variables z for control points c and predicted states x_traj
VectorXd
qpVariables(const ampc::MPC& mpc, const VectorXd& c, const VectorXd& x_traj)
{
  VectorXd z{VectorXd::Zero(mpc.getNumQPVariables())};
  for (const ampc::QPBlock& block : mpc.getQPVariableBlocks()) {
    if (block.name == "control_points")
      z.segment(block.offset, block.size) = c;
    else if (block.name == "states")
      z.segment(block.offset, block.size) = x_traj;
  }
  return z;
}

// At the solution and at a perturbed point, the QP objective must equal the
// directly summed objective up to the same constant
template <ampc::Formulation F> void checkObjectiveAtSolution()
{
  const Vector2d R{0.7, 1.9};
  const VectorXd x0{Vector2d{0.5, -0.2}};
  auto settings{ampc::OSQPSolver::getRecommendedSettings(true)};
  settings.polishing = true;
  for (const ampc::Parameterization& param : bases()) {
    SCOPED_TRACE(name(param));
    const int T{param.horizon_steps}, nc{2 * param.num_control_points};
    const VectorXd u_ref{referenceTrajectory(T)};
    ampc::MPC mpc{2, 2, param, {.input_cost = Trajectory}, F};
    configure(mpc);
    // wide limits, so the solution is the unconstrained minimum
    mpc.setInputLimits(Vector2d::Constant(-1e3), Vector2d::Constant(1e3));
    mpc.setInputWeights(R);
    mpc.setReferenceInputTrajectory(u_ref);
    ASSERT_TRUE(mpc.initializeSolver(settings));
    ASSERT_EQ(mpc.solve(x0), ampc::SolveStatus::Success);

    const ampc::QuadraticProgram qp{mpc.getQP()};
    auto qpObjective = [&qp](const VectorXd& z) {
      return 0.5 * z.dot(qp.P * z) + qp.q.dot(z);
    };
    VectorXd c{nc}, u{2 * T}, x_traj, x_pred{2 * T};
    mpc.getInputControlPoints(c);
    mpc.getInputTrajectory(u);
    mpc.getPredictedStateTrajectory(x_pred);
    const double direct{directObjective(mpc, x0, u, R, u_ref, x_traj)};
    EXPECT_TRUE(expectEigenNear(x_traj, x_pred, 1e-6));
    const double constant{direct - qpObjective(qpVariables(mpc, c, x_traj))};

    std::srand(5);
    const VectorXd c2{c + 0.3 * VectorXd::Random(nc)};
    const VectorXd u2{param.evaluate(2, c2)};
    const double direct2{directObjective(mpc, x0, u2, R, u_ref, x_traj)};
    EXPECT_NEAR(direct2 - qpObjective(qpVariables(mpc, c2, x_traj)), constant,
                1e-9 * (1.0 + std::abs(direct2)));
    // the solution minimizes the direct objective (no bound is active)
    EXPECT_LT(direct, direct2);
  }
}

} // namespace

TEST(InputCostTester, givenTrajectoryCondensed_QPMatchesDirectConstruction)
{
  checkAgainstDirectConstruction<ampc::Formulation::Condensed>();
}

TEST(InputCostTester, givenTrajectorySparse_QPMatchesDirectConstruction)
{
  checkAgainstDirectConstruction<ampc::Formulation::Sparse>();
}

TEST(InputCostTester, givenTrajectoryCondensed_ObjectiveMatchesDirectSum)
{
  checkObjectiveAtSolution<ampc::Formulation::Condensed>();
}

TEST(InputCostTester, givenTrajectorySparse_ObjectiveMatchesDirectSum)
{
  checkObjectiveAtSolution<ampc::Formulation::Sparse>();
}

TEST(InputCostTester, givenDegreeZero_TrajectoryWeightsEachBlockByItsLength)
{
  // Degree 0: each sample is one control point, so the trajectory cost is the
  // control point cost with R scaled by the length of each block
  const Vector2d R{0.7, 1.9}, u{0.4, -0.6}, x0{0.5, -0.2};
  using P = ampc::Parameterization;
  for (const P& param : {P::moveBlocking(12, 4, 1.3), P::bspline(10, 0, 4)}) {
    SCOPED_TRACE(name(param));
    const VectorXd lengths{denseBasis(param).colwise().sum().transpose()};
    ASSERT_GT(lengths.maxCoeff(), lengths.minCoeff()); // unequal blocks
    for (const ampc::Formulation f :
         {ampc::Formulation::Condensed, ampc::Formulation::Sparse}) {
      CostInspector trajectory{2, 2, param, {.input_cost = Trajectory}, f};
      CostInspector control_points{
          2, 2, param, {.input_cost = ControlPoints}, f};
      CostInspector without{2, 2, param, f};
      for (CostInspector* mpc : {&trajectory, &control_points, &without}) {
        configure(*mpc);
        if (mpc != &without) {
          mpc->setInputWeights(R);
          mpc->setReferenceInput(u);
        }
        mpc->assemble(x0);
      }
      const int nc{2 * param.num_control_points};
      const VectorXd scale{lengths.replicate(1, 2).transpose().reshaped()};
      const MatrixXd H_traj{trajectory.P().topLeftCorner(nc, nc)
                            - without.P().topLeftCorner(nc, nc)};
      const MatrixXd H_cp{control_points.P().topLeftCorner(nc, nc)
                          - without.P().topLeftCorner(nc, nc)};
      EXPECT_TRUE(expectEigenNear(H_traj, scale.asDiagonal() * H_cp, 1e-12));
      const VectorXd g_traj{trajectory.q().head(nc) - without.q().head(nc)};
      const VectorXd g_cp{control_points.q().head(nc) - without.q().head(nc)};
      EXPECT_TRUE(expectEigenNear(g_traj, scale.cwiseProduct(g_cp), 1e-12));
    }
  }
}

TEST(InputCostTester, givenEqualBlocks_TrajectoryEqualsScaledControlPointCost)
{
  // Move-blocking with equal blocks of length L: Trajectory with R gives the
  // solution of ControlPoints with L R
  const int T{12}, nc{4}, L{T / nc};
  const auto param{ampc::Parameterization::moveBlocking(T, nc)};
  const Vector2d R{0.7, 1.9}, u{0.4, -0.6}, x0{0.5, -0.2};
  ampc::MPC trajectory{2, 2, param, {.input_cost = Trajectory}};
  ampc::MPC control_points{2, 2, param, {.input_cost = ControlPoints}};
  configure(trajectory);
  configure(control_points);
  trajectory.setInputWeights(R);
  control_points.setInputWeights(L * R);
  trajectory.setReferenceInput(u);
  control_points.setReferenceInput(u);
  auto settings{ampc::OSQPSolver::getRecommendedSettings(true)};
  settings.polishing = true;
  ASSERT_TRUE(trajectory.initializeSolver(settings));
  ASSERT_TRUE(control_points.initializeSolver(settings));
  ASSERT_EQ(trajectory.solve(x0), ampc::SolveStatus::Success);
  ASSERT_EQ(control_points.solve(x0), ampc::SolveStatus::Success);
  VectorXd c_traj{2 * nc}, c_cp{2 * nc};
  trajectory.getInputControlPoints(c_traj);
  control_points.getInputControlPoints(c_cp);
  EXPECT_TRUE(expectEigenNear(c_traj, c_cp, 1e-8));
}

TEST(InputCostTester, givenTrajectory_CondensedAndSparseAgree)
{
  auto settings{ampc::OSQPSolver::getRecommendedSettings(true)};
  settings.polishing = true;
  settings.max_iter = 20000;
  for (const ampc::Parameterization& param : bases()) {
    SCOPED_TRACE(name(param));
    const int T{param.horizon_steps};
    const ampc::Options opts{.input_cost = Trajectory,
                             .use_input_rate_cost = true};
    ampc::MPC condensed{2, 2, param, opts};
    ampc::MPC sparse{2, 2, param, opts, ampc::Formulation::Sparse};
    for (ampc::MPC* mpc : {&condensed, &sparse}) {
      configure(*mpc);
      mpc->setInputWeights(Vector2d{0.7, 1.9});
      mpc->setInputRateWeights(Vector2d{0.3, 0.2});
      mpc->setReferenceInputTrajectory(referenceTrajectory(T));
      ASSERT_TRUE(mpc->initializeSolver(settings));
    }
    Vector2d x{0.5, -0.2};
    for (int step{0}; step < 3; ++step) {
      ASSERT_EQ(condensed.solve(x), ampc::SolveStatus::Success);
      ASSERT_EQ(sparse.solve(x), ampc::SolveStatus::Success);
      VectorXd u_condensed{2 * T}, u_sparse{2 * T};
      condensed.getInputTrajectory(u_condensed);
      sparse.getInputTrajectory(u_sparse);
      EXPECT_TRUE(expectEigenNear(u_condensed, u_sparse, 1e-5));
      Vector2d x_next;
      condensed.propagateModel(x, u_condensed.head<2>(), x_next);
      x = x_next;
      // a new reference each step (vector-only update)
      condensed.setReferenceInputTrajectory((1.0 + step)
                                            * referenceTrajectory(T));
      sparse.setReferenceInputTrajectory((1.0 + step) * referenceTrajectory(T));
    }
  }
}

TEST(InputCostTester, givenTrajectoryReferenceChange_OnlyQChanges)
{
  // a reference change leaves P as it was (no refactorization), a weight
  // change rebuilds it
  const auto param{ampc::Parameterization::bspline(15, 2, 6)};
  const Vector2d x0{0.5, -0.2};
  for (const ampc::Formulation f :
       {ampc::Formulation::Condensed, ampc::Formulation::Sparse}) {
    CostInspector mpc{2, 2, param, {.input_cost = Trajectory}, f};
    configure(mpc);
    mpc.assemble(x0);
    const MatrixXd P{mpc.P()};
    const VectorXd q{mpc.q()};

    mpc.setReferenceInputTrajectory(referenceTrajectory(15));
    mpc.assemble(x0);
    EXPECT_TRUE(expectEigenNear(mpc.P(), P, 0.0));
    EXPECT_FALSE(mpc.q().isApprox(q));

    mpc.setInputWeights(Vector2d{0.7, 1.9});
    mpc.assemble(x0);
    EXPECT_FALSE(mpc.P().isApprox(P));
  }
}

TEST(InputCostTester, givenCustomBasisTrajectory_SetReferenceInputIsAllowed)
{
  const auto param{
      ampc::Parameterization::fromBasis(12, laguerreBasis(12, 4, 0.6))};
  ampc::MPC control_points{2, 2, param, {.input_cost = ControlPoints}};
  expectLogicErrorWithMessage(
      [&]() { control_points.setReferenceInput(Vector2d::Ones()); },
      "A custom basis has no control points");
  ampc::MPC trajectory{2, 2, param, {.input_cost = Trajectory}};
  trajectory.setReferenceInput(Vector2d{0.4, -0.6});
  EXPECT_TRUE(expectEigenNear(trajectory.getReferenceInputTrajectory(),
                              Vector2d{0.4, -0.6}.replicate(12, 1), 0.0));
}

TEST(InputCostTester, givenOff_InputCostSettersThrow)
{
  const auto param{ampc::Parameterization::linearInterp(10, 4)};
  ampc::MPC mpc{2, 2, param};
  expectLogicErrorWithMessage([&]() { mpc.setInputWeights(Vector2d::Ones()); },
                              "Input cost is not enabled");
  expectLogicErrorWithMessage(
      [&]() { mpc.setWeights(Vector2d::Ones(), Vector2d::Ones()); },
      "Input cost is not enabled");
  expectLogicErrorWithMessage(
      [&]() { mpc.setReferenceInput(Vector2d::Ones()); },
      "[MPC::setReferenceInput] Input cost is not enabled");
  expectLogicErrorWithMessage(
      [&]() { mpc.setReferenceInputTrajectory(VectorXd::Ones(20)); },
      "[MPC::setReferenceInputTrajectory]");
  expectLogicErrorWithMessage(
      [&]() { mpc.setReferenceInputControlPoints(VectorXd::Ones(8)); },
      "[MPC::setReferenceInputControlPoints]");
  expectLogicErrorWithMessage([&]() { (void)mpc.getInputWeights(); },
                              "Input cost is not enabled");
}

TEST(InputCostTester, givenTrajectory_StreamsMode)
{
  const ampc::Options opts{.input_cost = Trajectory};
  std::ostringstream ss;
  ss << opts;
  EXPECT_EQ(ss.str().rfind("Options(input_cost=Trajectory, ", 0), 0u)
      << ss.str();
  std::ostringstream modes;
  modes << ampc::InputCostMode::Off << ' ' << ControlPoints << ' '
        << Trajectory;
  EXPECT_EQ(modes.str(), "Off ControlPoints Trajectory");

  ampc::MPC mpc{2, 2, ampc::Parameterization::linearInterp(10, 4), opts};
  mpc.setInputWeights(Vector2d{0.5, 2.0});
  std::ostringstream summary;
  summary << mpc;
  EXPECT_NE(summary.str().find("R = [0.5, 2]"), std::string::npos)
      << summary.str();
}
