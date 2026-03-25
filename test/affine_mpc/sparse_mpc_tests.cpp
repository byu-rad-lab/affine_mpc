#include "affine_mpc/options.hpp"
#include "affine_mpc/parameterization.hpp"
#include "affine_mpc/sparse_mpc.hpp"

#include <Eigen/Core>
#include <gtest/gtest.h>

#include "utils.hpp"

using namespace Eigen;
namespace ampc = affine_mpc;

// Tests are done with a Mass-Spring-Damper system (2 states, 1 input)
class SparseMPCProtectedTester : public affine_mpc::SparseMPC
{
public:
  using SparseMPC::SparseMPC;
  virtual ~SparseMPCProtectedTester() = default;

  void setModel()
  {
    Matrix2d A;
    Vector2d B, w;
    A << 0, 1, -0.6, -0.1;
    B << 0, 0.2;
    w.setZero();
    setModelContinuous2Discrete(A, B, w, 0.1);
  }

  void updateQPTerms(const Ref<const VectorXd>& x0)
  {
    affine_mpc::SparseMPC::qpUpdateX0(x0);
  }

  const auto getAd() const { return this->Ad_; }
  const auto getBd() const { return this->Bd_; }
  const auto getWd() const { return this->wd_; }
  const auto getQbig() const { return this->Q_big_; }
  const auto getRbig() const { return this->R_big_; }
  const auto getP() const { return this->P_; }
  const auto getQ() const { return this->q_; }
  const auto getA() const { return this->A_; }
  const auto getL() const { return this->l_; }
  const auto getU() const { return this->u_; }
  const auto getSplineWeights() const { return this->spline_weights_; }
  const auto getSplineSegmentIdxs() const { return this->spline_segment_idxs_; }
};

// ---- Constraint tests ------------------------------------------------------

TEST(SparseMPCProtectedTester, givenModel_FormsModelConstraintsCorrectly)
{
  const int n{2}, m{1}, T{5}, nc{3};
  SparseMPCProtectedTester mpc{n, m,
                               ampc::Parameterization::linearInterp(T, nc)};
  mpc.setModel();

  const MatrixXd A = mpc.getA();
  const int ctrls_dim = m * nc;
  const int x_traj_dim = n * T;
  const int total_cols = ctrls_dim + x_traj_dim;
  const int total_rows = x_traj_dim   // model constraints
                         + ctrls_dim; // input saturation

  ASSERT_EQ(A.rows(), total_rows);
  ASSERT_EQ(A.cols(), total_cols);

  const MatrixXd A_model = A.topRows(x_traj_dim);

  const MatrixXd Ad = mpc.getAd();
  const MatrixXd Bd = mpc.getBd();
  const MatrixXd In = MatrixXd::Identity(n, n);
  const MatrixXd zeroA = MatrixXd::Zero(n, n);
  const MatrixXd zeroB = MatrixXd::Zero(n, m);

  MatrixXd A_test{x_traj_dim, total_cols};
  // clang-format off
  A_test.block(0, 0, x_traj_dim, ctrls_dim) << Bd,     zeroB,  zeroB,
                                               0.5*Bd, 0.5*Bd, zeroB,
                                               zeroB,  Bd,     zeroB,
                                               zeroB,  0.5*Bd, 0.5*Bd,
                                               zeroB,  zeroB,  Bd;

  A_test.block(0, ctrls_dim, x_traj_dim, x_traj_dim) <<
      -In,    zeroA, zeroA, zeroA, zeroA,
       Ad,   -In,    zeroA, zeroA, zeroA,
       zeroA, Ad,   -In,    zeroA, zeroA,
       zeroA, zeroA, Ad,   -In,    zeroA,
       zeroA, zeroA, zeroA, Ad,   -In;
  // clang-format on

  ASSERT_TRUE(expectEigenNear(A_model, A_test, 1e-15));

  const Vector2d x0{0, 1.0};
  mpc.updateQPTerms(x0); // to update l_ and u_ with model constraints

  const VectorXd l_model = mpc.getL().head(x_traj_dim);
  const VectorXd u_model = mpc.getU().head(x_traj_dim);
  const VectorXd wd = mpc.getWd();

  VectorXd l_test{x_traj_dim}, u_test{x_traj_dim};
  l_test << -(Ad * x0 + wd), -wd, -wd, -wd, -wd;
  u_test = l_test;

  ASSERT_TRUE(expectEigenNear(l_model, l_test, 1e-15));
  ASSERT_TRUE(expectEigenNear(u_model, u_test, 1e-15));
}

TEST(SparseMPCProtectedTester,
     givenInputLimits_FormsSaturationConstraintsCorrectly)
{
  const int n{2}, m{1}, T{5}, nc{3};
  SparseMPCProtectedTester mpc{n, m,
                               ampc::Parameterization::linearInterp(T, nc)};
  mpc.setModel();

  // update l_ and u_ with input limits
  const VectorXd u_min = VectorXd::Zero(m);
  const VectorXd u_max = VectorXd::Constant(m, 3.0);
  mpc.setInputLimits(u_min, u_max);
  mpc.updateQPTerms(VectorXd::Zero(n));

  // test shape of input saturation portion of A
  const MatrixXd A_sat = mpc.getA().bottomRows(m * nc);
  ASSERT_EQ(A_sat.cols(), m * nc + n * T);

  // test values of input saturation portion of A
  const MatrixXd A_sat_test = MatrixXd::Identity(m * nc, A_sat.cols());
  ASSERT_TRUE(expectEigenNear(A_sat, A_sat_test, 1e-15));

  // test values of l_ and u_ for input saturation constraints
  const VectorXd l_sat = mpc.getL().tail(m * nc);
  const VectorXd u_sat = mpc.getU().tail(m * nc);
  const VectorXd l_sat_test = u_min.replicate(nc, 1);
  const VectorXd u_sat_test = u_max.replicate(nc, 1);
  ASSERT_TRUE(expectEigenNear(l_sat, l_sat_test, 1e-15));
  ASSERT_TRUE(expectEigenNear(u_sat, u_sat_test, 1e-15));
}

TEST(SparseMPCProtectedTester, givenSlewRate_FormsSlewConstraintsCorrectly)
{
  const int n{2}, m{1}, T{5}, nc{3};
  SparseMPCProtectedTester mpc{n, m,
                               ampc::Parameterization::linearInterp(T, nc),
                               ampc::Options{.slew_control_points = true}};
  mpc.setModel();

  VectorXd slew{1};
  slew << 0.5;
  mpc.setSlewRate(slew);
  mpc.updateQPTerms(VectorXd::Zero(n)); // to update l_ and u_

  const MatrixXd A = mpc.getA();
  const int ctrls_dim = m * nc;
  const int x_traj_dim = n * T;
  const int slew_rows = ctrls_dim - m; // nc-1 slew constraints

  // Slew rate block starts at x_traj_dim + ctrls_dim
  const int slew_start = x_traj_dim + ctrls_dim;
  ASSERT_EQ(A.rows(), slew_start + slew_rows);

  // test values of slew rate portion of A
  const MatrixXd A_slew = A.bottomRows(slew_rows);
  MatrixXd A_slew_test = MatrixXd::Zero(slew_rows, A.cols());
  A_slew_test.diagonal().setConstant(-1.0);
  A_slew_test.rightCols(A.cols() - m).diagonal().setConstant(1.0);
  ASSERT_TRUE(expectEigenNear(A_slew, A_slew_test, 1e-15));

  // test values of l_ and u_ for slew rate constraints
  const VectorXd l_slew = mpc.getL().tail(slew_rows);
  const VectorXd u_slew = mpc.getU().tail(slew_rows);
  VectorXd l_slew_test{slew_rows}, u_slew_test{slew_rows};
  l_slew_test << -slew, -slew;
  u_slew_test << slew, slew;
  ASSERT_TRUE(expectEigenNear(l_slew, l_slew_test, 1e-15));
  ASSERT_TRUE(expectEigenNear(u_slew, u_slew_test, 1e-15));
}

TEST(SparseMPCProtectedTester,
     givenStateSaturation_FormsStateSaturationConstraintsCorrectly)
{
  const int n{2}, m{1}, T{5}, nc{3};
  SparseMPCProtectedTester mpc{n, m,
                               ampc::Parameterization::linearInterp(T, nc),
                               ampc::Options{.saturate_states = true}};
  mpc.setModel();

  const Vector2d x_min{-1.0, -5.0}, x_max{3.0, 5.0};
  mpc.setStateLimits(x_min, x_max);
  mpc.updateQPTerms(VectorXd::Zero(n)); // to update l_ and u_

  const MatrixXd A = mpc.getA();
  const int ctrls_dim = m * nc;
  const int x_traj_dim = n * T;
  const int x_sat_start = x_traj_dim + ctrls_dim;

  // test shape of A
  ASSERT_EQ(A.rows(), x_sat_start + x_traj_dim);
  ASSERT_EQ(A.cols(), ctrls_dim + x_traj_dim);

  // test values of state saturation portion of A
  const MatrixXd A_sat = A.bottomRows(x_traj_dim);
  MatrixXd A_sat_test{x_traj_dim, ctrls_dim + x_traj_dim};
  A_sat_test << MatrixXd::Zero(x_traj_dim, ctrls_dim),
      MatrixXd::Identity(x_traj_dim, x_traj_dim);
  ASSERT_TRUE(expectEigenNear(A_sat, A_sat_test, 1e-15));

  // test values of l_ and u_ for state saturation constraints
  const VectorXd l_sat = mpc.getL().tail(x_traj_dim);
  const VectorXd u_sat = mpc.getU().tail(x_traj_dim);
  const VectorXd l_sat_test = x_min.replicate(T, 1);
  const VectorXd u_sat_test = x_max.replicate(T, 1);
  ASSERT_TRUE(expectEigenNear(l_sat, l_sat_test, 1e-15));
  ASSERT_TRUE(expectEigenNear(u_sat, u_sat_test, 1e-15));
}

TEST(SparseMPCProtectedTester,
     givenSlewRateAndStateSaturation_FormsConstraintsCorrectly)
{
  const int n{2}, m{1}, T{5}, nc{3};
  SparseMPCProtectedTester mpc{
      n, m, ampc::Parameterization::linearInterp(T, nc),
      ampc::Options{.slew_control_points = true, .saturate_states = true}};
  mpc.setModel();

  VectorXd slew{1};
  slew << 0.5;
  mpc.setSlewRate(slew);

  Vector2d x_min{-1.0, -5.0}, x_max{3.0, 5.0};
  mpc.setStateLimits(x_min, x_max);

  mpc.updateQPTerms(VectorXd::Zero(n)); // to update l_ and u_

  const MatrixXd A = mpc.getA();
  const int ctrls_dim = m * nc;
  const int x_traj_dim = n * T;
  const int slew_rows = ctrls_dim - m; // nc-1 slew constraints
  const int slew_start = x_traj_dim + ctrls_dim;
  const int x_sat_start = slew_start + slew_rows;

  // test shape of A
  ASSERT_EQ(A.rows(), x_sat_start + x_traj_dim);
  ASSERT_EQ(A.cols(), ctrls_dim + x_traj_dim);

  // test values of state saturation portion of A
  const MatrixXd A_sat = A.bottomRows(x_traj_dim);
  MatrixXd A_sat_test{x_traj_dim, ctrls_dim + x_traj_dim};
  A_sat_test << MatrixXd::Zero(x_traj_dim, ctrls_dim),
      MatrixXd::Identity(x_traj_dim, x_traj_dim);
  ASSERT_TRUE(expectEigenNear(A_sat, A_sat_test, 1e-15));

  // test values of slew rate portion of A
  const MatrixXd A_slew = A.middleRows(slew_start, slew_rows);
  MatrixXd A_slew_test = MatrixXd::Zero(slew_rows, A.cols());
  A_slew_test.diagonal().setConstant(-1.0);
  A_slew_test.rightCols(A.cols() - m).diagonal().setConstant(1.0);
  ASSERT_TRUE(expectEigenNear(A_slew, A_slew_test, 1e-15));

  // test values of l_ and u_ for state saturation constraints
  const VectorXd l_sat = mpc.getL().tail(x_traj_dim);
  const VectorXd u_sat = mpc.getU().tail(x_traj_dim);
  const VectorXd l_sat_test = x_min.replicate(T, 1);
  const VectorXd u_sat_test = x_max.replicate(T, 1);
  ASSERT_TRUE(expectEigenNear(l_sat, l_sat_test, 1e-15));
  ASSERT_TRUE(expectEigenNear(u_sat, u_sat_test, 1e-15));

  // test values of l_ and u_ for slew rate constraints
  const VectorXd l_slew = mpc.getL().segment(slew_start, slew_rows);
  const VectorXd u_slew = mpc.getU().segment(slew_start, slew_rows);
  VectorXd l_slew_test{slew_rows}, u_slew_test{slew_rows};
  l_slew_test << -slew, -slew;
  u_slew_test << slew, slew;
  ASSERT_TRUE(expectEigenNear(l_slew, l_slew_test, 1e-15));
  ASSERT_TRUE(expectEigenNear(u_slew, u_slew_test, 1e-15));
}

// ---- Cost matrix (P_) and cost vector (q_) tests ---------------------------

TEST(SparseMPCProtectedTester, givenStateWeightsOnly_FormsCorrectCostTerms)
{
  const int n{2}, m{1}, T{5}, nc{3};
  SparseMPCProtectedTester mpc{n, m,
                               ampc::Parameterization::linearInterp(T, nc)};
  mpc.setModel();

  const Vector2d Q_diag{2.0, 3.0};
  const Vector2d x_goal_step{1.0, 0.1};
  mpc.setStateWeights(Q_diag);
  mpc.setReferenceState(x_goal_step);

  const Vector2d x0{0.0, 0.0};
  mpc.updateQPTerms(x0);

  const MatrixXd P = mpc.getP();
  const VectorXd q = mpc.getQ();
  const int ctrls_dim = m * nc;
  const int x_traj_dim = n * T;

  MatrixXd P_expected{ctrls_dim + x_traj_dim, ctrls_dim + x_traj_dim};
  P_expected.setZero();
  P_expected.diagonal().tail(x_traj_dim) = Q_diag.replicate(T, 1);
  ASSERT_TRUE(expectEigenNear(P, P_expected, 1e-15));

  // State cost part of q: -Q_big * x_goal
  VectorXd q_expected{ctrls_dim + x_traj_dim};
  q_expected.setZero();
  q_expected.tail(x_traj_dim) =
      (-Q_diag.cwiseProduct(x_goal_step)).replicate(T, 1);
  ASSERT_TRUE(expectEigenNear(q, q_expected, 1e-15));
}

TEST(SparseMPCProtectedTester, givenInputCost_FormsCorrectCostTerms)
{
  const int n{2}, m{1}, T{5}, nc{3};
  SparseMPCProtectedTester mpc{n, m,
                               ampc::Parameterization::linearInterp(T, nc),
                               ampc::Options{.use_input_cost = true}};
  mpc.setModel();

  const Vector2d Q_diag{1.0, 1.0};
  const VectorXd R_diag = VectorXd::Constant(m, 0.5);
  const VectorXd u_goal = VectorXd::Constant(m, 2.0);
  const Vector2d x_goal_step{1.0, 0.1};
  mpc.setWeights(Q_diag, R_diag);
  mpc.setReferenceInput(u_goal);
  mpc.setReferenceState(x_goal_step);

  const Vector2d x0{0.0, 0.0};
  mpc.updateQPTerms(x0);

  const MatrixXd P = mpc.getP();
  const VectorXd q = mpc.getQ();
  const int ctrls_dim = m * nc;
  const int x_traj_dim = n * T;

  MatrixXd P_expected{ctrls_dim + x_traj_dim, ctrls_dim + x_traj_dim};
  P_expected.setZero();
  P_expected.diagonal() << R_diag.replicate(nc, 1), Q_diag.replicate(T, 1);
  ASSERT_TRUE(expectEigenNear(P, P_expected, 1e-15));

  VectorXd q_expected{ctrls_dim + x_traj_dim};
  q_expected << (-R_diag.cwiseProduct(u_goal)).replicate(nc, 1),
      (-Q_diag.cwiseProduct(x_goal_step)).replicate(T, 1);
  ASSERT_TRUE(expectEigenNear(q, q_expected, 1e-15));
}

TEST(SparseMPCProtectedTester,
     givenWeightsAndReferenceUpdatedSimultaneously_QvectorIsCorrect)
{
  // Verifies the refs_changed_ flag is not lost when weights_changed_ fires
  // first. Both updates must be reflected in q_ after a single qpUpdateX0 call.
  const int n{2}, m{1}, T{5}, nc{3};
  SparseMPCProtectedTester mpc{n, m,
                               ampc::Parameterization::linearInterp(T, nc),
                               ampc::Options{.use_input_cost = true}};
  mpc.setModel();

  const Vector2d Q_diag{1.0, 1.0};
  const VectorXd R_diag = VectorXd::Constant(m, 0.5);
  const Vector2d x_goal_step{1.0, 0.1};
  const VectorXd u_goal = VectorXd::Constant(m, 2.0);
  mpc.setWeights(Q_diag, R_diag);
  mpc.setReferenceState(x_goal_step);
  mpc.setReferenceInput(u_goal);
  mpc.setInputLimits(VectorXd::Constant(m, -10.0), VectorXd::Constant(m, 10.0));
  ASSERT_TRUE(mpc.initializeSolver());

  const Vector2d x0{0.0, 0.0};
  mpc.updateQPTerms(x0); // first solve cycle

  // Now update both weights and reference between solves
  const Vector2d Q_diag2{2.0, 3.0};
  const VectorXd R_diag2 = VectorXd::Constant(m, 4.0);
  const Vector2d x_goal_step2{0.5, -0.5};
  const VectorXd u_goal2 = VectorXd::Constant(m, 0.5);
  mpc.setWeights(Q_diag2, R_diag2);    // sets weights_changed_ = true
  mpc.setReferenceState(x_goal_step2); // sets refs_changed_ = true
  mpc.setReferenceInput(u_goal2);      // sets refs_changed_ = true

  // weights_changed_ branch fires, must use new x_goal_ and u_goal_
  mpc.updateQPTerms(x0);

  const VectorXd q = mpc.getQ();
  const int ctrls_dim = m * nc;
  const int x_traj_dim = n * T;

  // Expected q: -(R_diag2 * u_goal2).replicate(nc) for controls,
  //             -(Q_diag2 * x_goal_step2).replicate(T) for states
  VectorXd q_expected{ctrls_dim + x_traj_dim};
  q_expected << (-R_diag2.cwiseProduct(u_goal2)).replicate(nc, 1),
      (-Q_diag2.cwiseProduct(x_goal_step2)).replicate(T, 1);

  ASSERT_TRUE(expectEigenNear(q, q_expected, 1e-12));
}

// ---- End-to-end solve tests ------------------------------------------------

TEST(SparseMPCProtectedTester, initializedAndAskedToSolve_SolvesCorrectly)
{
  const int n{2}, m{1}, T{10}, nc{10};
  const bool use_input_cost{true};
  SparseMPCProtectedTester mpc{n, m,
                               ampc::Parameterization::linearInterp(T, nc),
                               ampc::Options{.use_input_cost = true}};
  mpc.setModel();

  Vector2d Q_diag{1.0, 0.1};
  VectorXd R_diag = VectorXd::Constant(m, 1e-4);
  mpc.setWeights(Q_diag, R_diag);

  Vector2d x_goal{1.0, 0.0};
  mpc.setReferenceState(x_goal);

  VectorXd u_min = VectorXd::Constant(m, 0.0);
  VectorXd u_max = VectorXd::Constant(m, 3.0);
  VectorXd u_goal = VectorXd::Zero(m);
  mpc.setInputLimits(u_min, u_max);
  mpc.setReferenceInput(u_goal);

  ASSERT_TRUE(mpc.initializeSolver());

  Vector2d x0{0.0, 0.0};
  ASSERT_EQ(mpc.solve(x0), ampc::SolveStatus::Success);

  VectorXd u_star{m};
  mpc.getNextInput(u_star);

  // From rest, with goal at (1,0) and u in [0,3], optimal first input is max
  VectorXd u_expected = VectorXd::Constant(m, 3.0);
  ASSERT_TRUE(expectEigenNear(u_star, u_expected, 1e-5));

  // verify input bounds were respected in the trajectory
  VectorXd u_traj{m * nc};
  mpc.getParameterizedInputTrajectory(u_traj);

  for (int i = 0; i < nc; ++i) {
    EXPECT_LE(u_traj(i), u_max(0) + 1e-5);
    EXPECT_GE(u_traj(i), u_min(0) - 1e-5);
  }
}

TEST(SparseMPCProtectedTester, initializedAndAskedToSolve_RespectsSlewRate)
{
  const int n{2}, m{1}, T{10}, nc{10};
  SparseMPCProtectedTester mpc{
      n, m, ampc::Parameterization::linearInterp(T, nc),
      ampc::Options{.use_input_cost = true, .slew_control_points = true}};
  mpc.setModel();

  Vector2d Q_diag{1.0, 0.11};
  VectorXd R_diag = VectorXd::Constant(m, 1e-4);
  mpc.setWeights(Q_diag, R_diag);

  Vector2d x_goal{1.0, 0.0};
  mpc.setReferenceState(x_goal);

  VectorXd u_min = VectorXd::Constant(m, 0.0);
  VectorXd u_max = VectorXd::Constant(m, 3.0);
  VectorXd u_slew = VectorXd::Constant(m, 1.0);
  VectorXd u_goal = VectorXd::Zero(m);
  mpc.setInputLimits(u_min, u_max);
  mpc.setReferenceInput(u_goal);
  mpc.setSlewRate(u_slew);

  OSQPSettings settings{affine_mpc::OSQPSolver::getRecommendedSettings(true)};
  ASSERT_TRUE(mpc.initializeSolver(settings));

  Vector2d x0{0.0, 0.0};
  ASSERT_EQ(mpc.solve(x0), ampc::SolveStatus::Success);

  // verify slew rate was respected in the trajectory
  VectorXd u_traj{m * nc};
  mpc.getParameterizedInputTrajectory(u_traj);

  int slew_violations{0};
  for (int i = 0; i < nc - 1; ++i)
    slew_violations += std::abs(u_traj(i + 1) - u_traj(i)) > u_slew(0) + 1e-5;
  ASSERT_EQ(slew_violations, 0);
}

TEST(SparseMPCProtectedTester, initializedAndAskedToSolve_RespectsStateBounds)
{
  const int n{2}, m{1}, T{10}, nc{10};
  SparseMPCProtectedTester mpc{
      n, m, ampc::Parameterization::linearInterp(T, nc),
      ampc::Options{.use_input_cost = true, .saturate_states = true}};
  mpc.setModel();

  mpc.setWeights(Vector2d{1.0, 0.1}, VectorXd::Constant(m, 1e-4));
  mpc.setReferenceState(Vector2d{1.0, 0.0});
  mpc.setInputLimits(VectorXd::Constant(m, -3.0), VectorXd::Constant(m, 3.0));

  // Tight state bounds that will be active given x0={0,0} and goal at (1,0)
  const Vector2d x_min{-0.5, -2.0}, x_max{0.5, 2.0};
  mpc.setStateLimits(x_min, x_max);

  ASSERT_TRUE(mpc.initializeSolver());
  ASSERT_EQ(mpc.solve(Vector2d{0.0, 0.0}), ampc::SolveStatus::Success);

  VectorXd x_traj(n * T);
  mpc.getPredictedStateTrajectory(x_traj);

  int bound_violations{0};
  for (int k = 0; k < T; ++k) {
    Vector2d xk = x_traj.segment(n * k, n);
    bound_violations += (xk.array() > x_max.array() + 1e-5).any();
    bound_violations += (xk.array() < x_min.array() - 1e-5).any();
  }
  ASSERT_EQ(bound_violations, 0);
}

TEST(SparseMPCProtectedTester,
     givenWeightUpdateBetweenSolves_ProducesCorrectSolution)
{
  const int n{2}, m{1}, T{10}, nc{10};
  SparseMPCProtectedTester mpc{n, m,
                               ampc::Parameterization::linearInterp(T, nc),
                               ampc::Options{.use_input_cost = true}};
  mpc.setModel();

  mpc.setWeights(Vector2d{1.0, 0.1}, VectorXd::Constant(m, 1e-4));
  mpc.setReferenceState(Vector2d{1.0, 0.0});
  mpc.setInputLimits(VectorXd::Constant(m, 0.0), VectorXd::Constant(m, 3.0));
  mpc.setReferenceInput(VectorXd::Zero(m));

  ASSERT_TRUE(mpc.initializeSolver());

  const Vector2d x0{0.0, 0.0};
  ASSERT_EQ(mpc.solve(x0), ampc::SolveStatus::Success);
  VectorXd u_first{m};
  mpc.getNextInput(u_first);

  // Heavy input penalty: solution should move away from saturation
  mpc.setWeights(Vector2d{1.0, 0.1}, VectorXd::Constant(m, 1.0));
  ASSERT_EQ(mpc.solve(x0), ampc::SolveStatus::Success);
  VectorXd u_second{m};
  mpc.getNextInput(u_second);

  ASSERT_LT(u_second(0), u_first(0) - 1e-3);
}
