#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <Eigen/SparseCore>
#include <cmath>
#include <gtest/gtest.h>
#include <limits>
#include <sstream>

#include "affine_mpc/detail/qp.hpp"
#include "affine_mpc/osqp_solver.hpp"
#include "affine_mpc/solve_status.hpp"
#include "utils.hpp"

class OSQPSolverProtectedTester : public affine_mpc::OSQPSolver
{
public:
  OSQPSolverProtectedTester(const int n, const int m) : OSQPSolver(n, m) {}
  ~OSQPSolverProtectedTester() {}

  bool testInitializeP(const Eigen::Ref<const MatrixXF>& P)
  {
    MatrixXF A(m_, n_);
    A.setZero();
    VectorXF q{n_};
    VectorXF l{m_}, u{m_};
    l.setZero();
    u.setOnes();
    OSQPSettings settings{affine_mpc::OSQPSolver::getDefaultSettings()};
    return initialize(P, A, q, l, u, settings);
  }

  void testInitializeA(const Eigen::Ref<Eigen::MatrixXd> A)
  {
    MatrixXF P{n_, n_};
    P.setZero();
    VectorXF q{n_};
    VectorXF l{m_}, u{m_};
    l.setZero();
    u.setOnes();
    OSQPSettings settings{affine_mpc::OSQPSolver::getDefaultSettings()};
    bool success = initialize(P, A, q, l, u, settings);
    ASSERT_TRUE(success);
  }

  auto getPx() { return P_x_; }
  auto getPi() { return P_i_; }
  auto getPp() { return P_p_; }
  auto getAx() { return A_x_; }
  auto getAi() { return A_i_; }
  auto getAp() { return A_p_; }
};

TEST(OSQPSolverProtectedTester, givenIdentityP_FormsCscPCorrectly)
{
  const int n{6}, m{4};
  OSQPSolverProtectedTester base(n, m);

  Eigen::Matrix<OSQPFloat, n, n> P;
  P.setZero();
  P.diagonal().setOnes();

  bool success = base.testInitializeP(P);
  ASSERT_TRUE(success);

  Eigen::Matrix<OSQPFloat, 6, 1> px_true;
  px_true.setOnes();
  Eigen::Matrix<OSQPInt, 6, 1> pi_true;
  pi_true << 0, 1, 2, 3, 4, 5;
  Eigen::Matrix<OSQPInt, 7, 1> pp_true;
  pp_true << 0, 1, 2, 3, 4, 5, 6;

  ASSERT_TRUE(expectEigenNear(px_true, base.getPx(), 1e-6));
  ASSERT_TRUE(expectEigenNear(pi_true, base.getPi(), 1e-6));
  ASSERT_TRUE(expectEigenNear(pp_true, base.getPp(), 1e-6));
}

TEST(OSQPSolverProtectedTester, givenIdentityA_FormsCscACorrectly)
{
  const int n{6}, m{4};
  OSQPSolverProtectedTester base{n, m};

  Eigen::Matrix<OSQPFloat, m, n> A;
  A.setZero();
  A.diagonal().setOnes();

  base.testInitializeA(A);

  Eigen::Matrix<OSQPFloat, 4, 1> ax_true;
  ax_true.setOnes();
  Eigen::Matrix<OSQPInt, 4, 1> ai_true;
  ai_true << 0, 1, 2, 3;
  Eigen::Matrix<OSQPInt, 7, 1> ap_true;
  ap_true << 0, 1, 2, 3, 4, 4, 4;

  ASSERT_TRUE(expectEigenNear(ax_true, base.getAx(), 1e-6));
  ASSERT_TRUE(expectEigenNear(ai_true, base.getAi(), 1e-6));
  ASSERT_TRUE(expectEigenNear(ap_true, base.getAp(), 1e-6));
}

TEST(OSQPSolverProtectedTester, givenFullP_FormsCscPCorrectly)
{
  const int n{2}, m{3};
  OSQPSolverProtectedTester base{n, m};

  Eigen::Matrix2d P;
  P << 4, 1, 1, 2;

  bool success = base.testInitializeP(P);
  ASSERT_TRUE(success);

  Eigen::Matrix<OSQPFloat, 3, 1> px_true{4.0, 1.0, 2.0};
  Eigen::Matrix<OSQPInt, 3, 1> pi_true{0, 0, 1};
  Eigen::Matrix<OSQPInt, 3, 1> pp_true{0, 1, 3};

  ASSERT_TRUE(expectEigenNear(px_true, base.getPx(), 1e-6));
  ASSERT_TRUE(expectEigenNear(pi_true, base.getPi(), 1e-6));
  ASSERT_TRUE(expectEigenNear(pp_true, base.getPp(), 1e-6));
}

TEST(OSQPSolverProtectedTester, givenFullA_FormsCscACorrectly)
{
  const int n{2}, m{3};
  OSQPSolverProtectedTester base{n, m};

  Eigen::Matrix<OSQPFloat, m, n> A;
  A << 1, 2, 3, 4, 5, 6;

  base.testInitializeA(A);

  Eigen::Matrix<OSQPFloat, 6, 1> ax_true;
  ax_true << 1, 3, 5, 2, 4, 6;
  Eigen::Matrix<OSQPInt, 6, 1> ai_true;
  ai_true << 0, 1, 2, 0, 1, 2;
  Eigen::Matrix<OSQPInt, 3, 1> ap_true{0, 3, 6};

  ASSERT_TRUE(expectEigenNear(ax_true, base.getAx(), 1e-6));
  ASSERT_TRUE(expectEigenNear(ai_true, base.getAi(), 1e-6));
  ASSERT_TRUE(expectEigenNear(ap_true, base.getAp(), 1e-6));
}

TEST(OSQPSolverProtectedTester, givenSparseP_FormsCscPCorrectly)
{
  const int n{6}, m{4};
  OSQPSolverProtectedTester base{n, m};

  Eigen::Matrix<OSQPFloat, n, n> P;
  P << 1, 0, 0, 0, 4, 0, 1, 0, 0, 2, 0, 0, 1, 1, 0, 0, 5, 0, 1, 1, 1, 3, 0, 0,
      1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 6;

  bool success = base.testInitializeP(P);
  ASSERT_TRUE(!success); // non-convex test case should fail

  // the upper triangle's nonzeros plus the whole diagonal (zeros included)
  Eigen::Matrix<OSQPFloat, 9, 1> px_true;
  px_true << 1, 0, 0, 2, 3, 4, 5, 0, 6;
  Eigen::Matrix<OSQPInt, 9, 1> pi_true;
  pi_true << 0, 1, 2, 1, 3, 0, 2, 4, 5;
  Eigen::Matrix<OSQPInt, 7, 1> pp_true;
  pp_true << 0, 1, 2, 3, 5, 8, 9;

  ASSERT_TRUE(expectEigenNear(px_true, base.getPx(), 1e-6));
  ASSERT_TRUE(expectEigenNear(pi_true, base.getPi(), 1e-6));
  ASSERT_TRUE(expectEigenNear(pp_true, base.getPp(), 1e-6));
}

TEST(OSQPSolverProtectedTester, givenExampleA_FormsCscACorrectly)
{
  const int n{2}, m{3};
  OSQPSolverProtectedTester base{n, m};

  Eigen::Matrix<OSQPFloat, m, n> A;
  A << 1, 1, 1, 0, 0, 1;

  base.testInitializeA(A);

  Eigen::Matrix<OSQPFloat, 4, 1> ax_true{1.0, 1.0, 1.0, 1.0};
  Eigen::Matrix<OSQPInt, 4, 1> ai_true{0, 1, 0, 2};
  Eigen::Matrix<OSQPInt, 3, 1> ap_true{0, 2, 4};

  ASSERT_TRUE(expectEigenNear(ax_true, base.getAx(), 1e-6));
  ASSERT_TRUE(expectEigenNear(ai_true, base.getAi(), 1e-6));
  ASSERT_TRUE(expectEigenNear(ap_true, base.getAp(), 1e-6));
}

TEST(OSQPSolverProtectedTester, askedToUpdateP_ReplacesPdataCorrectly)
{
  const int n{6}, m{2};
  OSQPSolverProtectedTester base{n, m};

  Eigen::Matrix<OSQPFloat, m, n> A;
  A.setZero();
  A.diagonal().setOnes();
  Eigen::Matrix<OSQPFloat, n, 1> q;
  q.setZero();
  Eigen::Matrix<OSQPFloat, m, 1> l, u;
  l.setZero();
  u.setOnes();

  Eigen::Matrix<OSQPFloat, n, n> P;
  P << 1, 0, 0, 0, 0, 0, 1, 2, 0, 0, 0, 0, 1, 1, 3, 0, 0, 0, 1, 1, 1, 4, 0, 0,
      1, 1, 1, 1, 5, 0, 1, 1, 1, 1, 1, 6;

  bool success{true};
  OSQPSettings settings{affine_mpc::OSQPSolver::getDefaultSettings()};
  success &= base.initialize(P, A, q, l, u, settings);

  P << 5, 0, 0, 0, 0, 0, 1, 4, 0, 0, 0, 0, 1, 1, 3, 0, 0, 0, 1, 1, 1, 2, 0, 0,
      1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 0;

  success &= base.updateCostMatrix(P);

  Eigen::Matrix<OSQPFloat, 6, 1> px_true;
  px_true << 5, 4, 3, 2, 1, 0;
  Eigen::Matrix<OSQPInt, 6, 1> pi_true;
  pi_true << 0, 1, 2, 3, 4, 5;
  Eigen::Matrix<OSQPInt, 7, 1> pp_true;
  pp_true << 0, 1, 2, 3, 4, 5, 6;

  ASSERT_TRUE(expectEigenNear(px_true, base.getPx(), 1e-6));
  ASSERT_TRUE(expectEigenNear(pi_true, base.getPi(), 1e-6));
  ASSERT_TRUE(expectEigenNear(pp_true, base.getPp(), 1e-6));
  ASSERT_TRUE(success);
}

TEST(OSQPSolverProtectedTester, askedToUpdateA_ReplacesAdataCorrectly)
{
  const int n{6}, m{2}, A_nnz_max{2};
  OSQPSolverProtectedTester base{n, m};

  Eigen::Matrix<OSQPFloat, m, n> A;
  A << 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0;
  Eigen::Matrix<OSQPFloat, n, 1> q;
  q.setZero();
  Eigen::Matrix<OSQPFloat, m, 1> l, u;
  l.setZero();
  u.setOnes();

  Eigen::Matrix<OSQPFloat, n, n> P;
  P.setZero();
  P.diagonal().setOnes();

  bool success{true};
  OSQPSettings settings{affine_mpc::OSQPSolver::getDefaultSettings()};
  success &= base.initialize(P, A, q, l, u, settings);

  // This changes the structure but the internal check only counts the number of
  // non-zero elements which is still 2 in this case - so this is OK although
  // normally the structure shouldn't change
  A << 9, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1;

  success &= base.updateConstraintMatrix(A);

  Eigen::Matrix<OSQPFloat, A_nnz_max, 1> ax_true;
  ax_true << 9, 0;
  Eigen::Matrix<OSQPInt, A_nnz_max, 1> ai_true;
  ai_true << 0, 1;
  Eigen::Matrix<OSQPInt, n + 1, 1> ap_true;
  ap_true << 0, 1, 2, 2, 2, 2, 2;

  ASSERT_TRUE(expectEigenNear(ax_true, base.getAx(), 1e-6));
  ASSERT_TRUE(expectEigenNear(ai_true, base.getAi(), 1e-6));
  ASSERT_TRUE(expectEigenNear(ap_true, base.getAp(), 1e-6));
  ASSERT_TRUE(success);
}

TEST(OSQPSolverProtectedTester,
     askedToUpdateExampleMatrices_ReplacesAandPdataCorrectly)
{
  const int n{2}, m{3}, P_nnz_max{3}, A_nnz_max{4};
  OSQPSolverProtectedTester base{n, m};

  Eigen::Matrix<OSQPFloat, m, n> A;
  A << 1, 1, 1, 0, 0, 1;
  Eigen::Matrix<OSQPFloat, n, 1> q;
  q.setOnes();
  Eigen::Matrix<OSQPFloat, m, 1> l, u;
  l << 1, 0, 0;
  u << 1, 0.7, 0.7;
  Eigen::Matrix<OSQPFloat, n, n> P;
  P << 4, 1, 1, 2;
  OSQPSettings settings{affine_mpc::OSQPSolver::getDefaultSettings()};

  bool success{true};
  success &= base.initialize(P, A, q, l, u, settings);

  A << 1.2, 1.1, 1.5, 0, 0, 0.8;
  P << 0, 1, 9, 2;

  success &= base.updateCostMatrix(P);
  success &= base.updateConstraintMatrix(A);

  Eigen::Matrix<OSQPFloat, P_nnz_max, 1> px_true;
  px_true << 0, 1, 2;
  Eigen::Matrix<OSQPInt, P_nnz_max, 1> pi_true;
  pi_true << 0, 0, 1;
  Eigen::Matrix<OSQPInt, n + 1, 1> pp_true;
  pp_true << 0, 1, 3;

  Eigen::Matrix<OSQPFloat, A_nnz_max, 1> ax_true;
  ax_true << 1.2, 1.5, 1.1, 0.8;
  Eigen::Matrix<OSQPInt, A_nnz_max, 1> ai_true;
  ai_true << 0, 1, 0, 2;
  Eigen::Matrix<OSQPInt, n + 1, 1> ap_true;
  ap_true << 0, 2, 4;

  ASSERT_TRUE(expectEigenNear(px_true, base.getPx(), 1e-6));
  ASSERT_TRUE(expectEigenNear(pi_true, base.getPi(), 1e-6));
  ASSERT_TRUE(expectEigenNear(pp_true, base.getPp(), 1e-6));
  ASSERT_TRUE(expectEigenNear(ax_true, base.getAx(), 1e-6));
  ASSERT_TRUE(expectEigenNear(ai_true, base.getAi(), 1e-6));
  ASSERT_TRUE(expectEigenNear(ap_true, base.getAp(), 1e-6));
  ASSERT_TRUE(success);
}

TEST(OSQPSolverProtectedTester, askedToSolveExample_solvesCorrectly)
{
  const int n{2}, m{3};
  OSQPSolverProtectedTester base{n, m};

  Eigen::Matrix<OSQPFloat, m, n> A;
  A << 1, 1, 1, 0, 0, 1;
  Eigen::Matrix<OSQPFloat, n, 1> q;
  q.setOnes();
  Eigen::Matrix<OSQPFloat, m, 1> l, u;
  l << 1, 0, 0;
  u << 1, 0.7, 0.7;
  Eigen::Matrix<OSQPFloat, n, n> P;
  P << 4, 1, 1, 2;
  OSQPSettings settings{affine_mpc::OSQPSolver::getDefaultSettings()};
  settings.alpha = 1.0;
  settings.verbose = false;
  // polishing tests the boundary conditions better but is a little slower
  settings.polishing = true;

  const bool success{base.initialize(P, A, q, l, u, settings)};
  ASSERT_TRUE(success);

  Eigen::Vector2d calculated_solution;
  const affine_mpc::SolveStatus status{base.solve(calculated_solution)};
  ASSERT_EQ(status, affine_mpc::SolveStatus::Success);

  Eigen::Vector2d actual_solution{0.3, 0.7};
  ASSERT_TRUE(expectEigenNear(calculated_solution, actual_solution, 1e-6));
}

TEST(OSQPSolverProtectedTester, solvingExampleAfterVectorUpdate_solvesCorrectly)
{
  const int n{2}, m{3};
  OSQPSolverProtectedTester base{n, m};

  Eigen::Matrix<OSQPFloat, m, n> A;
  A << 1, 1, 1, 0, 0, 1;
  Eigen::Matrix<OSQPFloat, n, 1> q;
  q.setOnes();
  Eigen::Matrix<OSQPFloat, m, 1> l, u;
  l << 1, 0, 0;
  u << 1, 0.7, 0.7;
  Eigen::Matrix<OSQPFloat, n, n> P;
  P << 4, 1, 1, 2;
  OSQPSettings settings{affine_mpc::OSQPSolver::getDefaultSettings()};
  settings.alpha = 1.0;
  settings.verbose = false;
  // polishing tests the boundary conditions better but is a little slower
  settings.polishing = true;

  bool success{true};
  success &= base.initialize(P, A, q, l, u, settings);

  Eigen::Vector2d calculated_solution;
  affine_mpc::SolveStatus status{base.solve(calculated_solution)};
  ASSERT_EQ(status, affine_mpc::SolveStatus::Success);

  q << 2, 3;
  l << 2, -1, -1;
  u << 2, 2.5, 2.5;
  success &= base.updateCostVector(q);
  success &= base.updateBounds(l, u);
  status = base.solve(calculated_solution);
  ASSERT_EQ(status, affine_mpc::SolveStatus::Success);

  Eigen::Vector2d actual_solution{0.75, 1.25};
  ASSERT_TRUE(expectEigenNear(calculated_solution, actual_solution, 1e-5));

  q.setOnes();
  l << 1, 0, 0;
  u << 1, 0.7, 0.7;
  success &= base.updateCostVector(q);
  success &= base.updateBounds(l, u);
  status = base.solve(calculated_solution);
  ASSERT_EQ(status, affine_mpc::SolveStatus::Success);

  actual_solution << 0.3, 0.7;
  ASSERT_TRUE(expectEigenNear(calculated_solution, actual_solution, 2e-5));

  ASSERT_TRUE(success);
}

TEST(OSQPSolverProtectedTester, solvingExampleAfterMatrixUpdate_solvesCorrectly)
{
  const int n{2}, m{3};
  OSQPSolverProtectedTester base{n, m};

  Eigen::Matrix<OSQPFloat, m, n> A;
  A << 1, 1, 1, 0, 0, 1;
  Eigen::Matrix<OSQPFloat, n, 1> q;
  q.setOnes();
  Eigen::Matrix<OSQPFloat, m, 1> l, u;
  l << 1, 0, 0;
  u << 1, 0.7, 0.7;
  Eigen::Matrix<OSQPFloat, n, n> P;
  P << 4, 1, 1, 2;
  OSQPSettings settings{affine_mpc::OSQPSolver::getDefaultSettings()};
  settings.alpha = 1.0;
  settings.verbose = false;
  settings.polishing = true;

  bool success{true};
  success &= base.initialize(P, A, q, l, u, settings);

  Eigen::Vector2d calculated_solution;
  affine_mpc::SolveStatus status{base.solve(calculated_solution)};

  P << 5, 1.5, 1.5, 1;
  A << 1.2, 1.1, 1.5, 0, 0, 0.8;
  success &= base.updateCostMatrix(P);
  success &= base.updateConstraintMatrix(A);
  status = base.solve(calculated_solution);
  ASSERT_EQ(status, affine_mpc::SolveStatus::Success);

  Eigen::Vector2d actual_solution{0.03125, 0.875};
  ASSERT_TRUE(expectEigenNear(calculated_solution, actual_solution, 1e-5));

  P << 4, 1, 1, 2;
  A << 1, 1, 1, 0, 0, 1;
  success &= base.updateCostMatrix(P);
  success &= base.updateConstraintMatrix(A);
  status = base.solve(calculated_solution);
  ASSERT_EQ(status, affine_mpc::SolveStatus::Success);

  actual_solution << 0.3, 0.7;
  ASSERT_TRUE(expectEigenNear(calculated_solution, actual_solution, 1e-5));

  ASSERT_TRUE(success);
}

TEST(OSQPSolverProtectedTester,
     givenMatricesOutsidePattern_IsWithinSparsityPatternReturnsFalse)
{
  const int n{3}, m{2};
  OSQPSolverProtectedTester base{n, m};

  Eigen::Matrix<OSQPFloat, n, n> P;
  P.setIdentity();
  Eigen::Matrix<OSQPFloat, m, n> A;
  A << 1, 0, 0, 0, 1, 0;
  Eigen::Matrix<OSQPFloat, n, 1> q;
  q.setZero();
  Eigen::Matrix<OSQPFloat, m, 1> l, u;
  l.setZero();
  u.setOnes();

  // no pattern exists before initialization
  EXPECT_FALSE(base.isWithinSparsityPattern(P, A));

  OSQPSettings settings{affine_mpc::OSQPSolver::getDefaultSettings()};
  ASSERT_TRUE(base.initialize(P, A, q, l, u, settings));
  EXPECT_TRUE(base.isWithinSparsityPattern(P, A));

  // explicit zeros within the pattern are fine
  P(1, 1) = 0;
  A(0, 0) = 0;
  EXPECT_TRUE(base.isWithinSparsityPattern(P, A));

  // the lower triangle of P is not read
  P(2, 0) = 0.5;
  EXPECT_TRUE(base.isWithinSparsityPattern(P, A));

  // upper triangle of P outside the pattern
  P(0, 2) = 0.5;
  EXPECT_FALSE(base.isWithinSparsityPattern(P, A));
  P(0, 2) = 0;

  // A outside the pattern
  A(1, 2) = 1;
  EXPECT_FALSE(base.isWithinSparsityPattern(P, A));
}

TEST(OSQPSolverProtectedTester, askedForSolveInfo_ReportsLastSolve)
{
  const int n{2}, m{3};
  OSQPSolverProtectedTester base{n, m};

  // defaults before initialization
  const affine_mpc::SolveInfo info_init{base.getSolveInfo()};
  EXPECT_EQ(info_init.status, affine_mpc::SolveStatus::NotInitialized);
  EXPECT_EQ(info_init.iterations, 0);
  EXPECT_TRUE(std::isnan(info_init.primal_residual));
  EXPECT_TRUE(std::isnan(info_init.dual_residual));

  Eigen::Matrix<OSQPFloat, m, n> A;
  A << 1, 1, 1, 0, 0, 1;
  Eigen::Matrix<OSQPFloat, n, 1> q;
  q.setOnes();
  Eigen::Matrix<OSQPFloat, m, 1> l, u;
  l << 1, 0, 0;
  u << 1, 0.7, 0.7;
  Eigen::Matrix<OSQPFloat, n, n> P;
  P << 4, 1, 1, 2;
  OSQPSettings settings{affine_mpc::OSQPSolver::getDefaultSettings()};
  settings.verbose = false;
  settings.polishing = true;
  ASSERT_TRUE(base.initialize(P, A, q, l, u, settings));

  Eigen::Vector2d solution;
  ASSERT_EQ(base.solve(solution), affine_mpc::SolveStatus::Success);

  // solution is (0.3, 0.7): 1/2 z^T P z + q^T z = 0.88 + 1.0
  const affine_mpc::SolveInfo info{base.getSolveInfo()};
  EXPECT_EQ(info.status, affine_mpc::SolveStatus::Success);
  EXPECT_GT(info.iterations, 0);
  EXPECT_NEAR(info.objective, 1.88, 1e-5);
  EXPECT_LE(info.primal_residual, 1e-6);
  EXPECT_LE(info.dual_residual, 1e-6);
  EXPECT_GE(info.qp_solve_time, 0.0);
  // only MPC::solve() measures the whole call
  EXPECT_EQ(info.mpc_solve_time, 0.0);

  std::ostringstream oss;
  oss << info;
  EXPECT_NE(oss.str().find("SolveInfo(status=Success, iterations="),
            std::string::npos);
}

TEST(OSQPSolverProtectedTester, givenQPInterface_SetsUpWithZeroCostVector)
{
  // OSQP fixes its cost scaling at setup from P and ||q||_inf, so setting up
  // with q = 0 and then updating q differs from setting up with q
  const int n{2}, m{3};
  affine_mpc::detail::QP qp;
  qp.P.resize(n, n);
  qp.P << 4, 1, 1, 2;
  qp.A.resize(m, n);
  qp.A << 1, 1, 1, 0, 0, 1;
  qp.q = Eigen::Vector2d{100.0, 300.0}; // dominates P's column norms
  qp.l = Eigen::Vector3d{1.0, 0.0, 0.0};
  qp.u = Eigen::Vector3d{1.0, 0.7, 0.7};
  OSQPSettings settings{affine_mpc::OSQPSolver::getRecommendedSettings()};
  settings.eps_abs = settings.eps_rel = 1e-8;

  affine_mpc::OSQPSolver by_qp{n, m, settings};
  ASSERT_TRUE(by_qp.initialize(qp, affine_mpc::QPBlockLayout{}));
  affine_mpc::OSQPSolver zero_then_q{n, m, settings};
  ASSERT_TRUE(zero_then_q.initialize(qp.P, qp.A, Eigen::Vector2d::Zero(), qp.l,
                                     qp.u, settings));
  ASSERT_TRUE(zero_then_q.updateCostVector(qp.q));
  affine_mpc::OSQPSolver with_q{n, m, settings};
  ASSERT_TRUE(with_q.initialize(qp.P, qp.A, qp.q, qp.l, qp.u, settings));

  Eigen::Vector2d z_qp, z_zero, z_q;
  ASSERT_EQ(by_qp.solve(z_qp), affine_mpc::SolveStatus::Success);
  ASSERT_EQ(zero_then_q.solve(z_zero), affine_mpc::SolveStatus::Success);
  ASSERT_EQ(with_q.solve(z_q), affine_mpc::SolveStatus::Success);

  EXPECT_EQ(by_qp.getSolveInfo().iterations,
            zero_then_q.getSolveInfo().iterations);
  EXPECT_EQ(z_qp, z_zero);
  // the test can tell the two setups apart
  EXPECT_NE(by_qp.getSolveInfo().iterations, with_q.getSolveInfo().iterations);
}

TEST(OSQPSolverProtectedTester, givenQPInterfaceMatrixUpdate_RescalesWithZeroQ)
{
  // A matrix update recomputes OSQP's scaling, the cost scaling from the q it
  // holds; through the QP interface it must match a fresh setup with q = 0
  const int n{2}, m{3};
  affine_mpc::detail::QP qp;
  qp.P.resize(n, n);
  qp.P << 4, 1, 1, 2;
  qp.A.resize(m, n);
  qp.A << 1, 1, 1, 0, 0, 1;
  qp.q = Eigen::Vector2d{100.0, 300.0}; // dominates P's column norms
  qp.l = Eigen::Vector3d{1.0, 0.0, 0.0};
  qp.u = Eigen::Vector3d{1.0, 0.7, 0.7};
  OSQPSettings settings{affine_mpc::OSQPSolver::getRecommendedSettings()};
  settings.eps_abs = settings.eps_rel = 1e-8;
  Eigen::Matrix2d P_new;
  P_new << 8, 2, 2, 3;

  affine_mpc::OSQPSolver by_qp{n, m, settings};
  ASSERT_TRUE(by_qp.initialize(qp, affine_mpc::QPBlockLayout{}));
  qp.P = P_new;
  ASSERT_TRUE(by_qp.updateMatrices(qp, true, false));

  affine_mpc::OSQPSolver fresh{n, m, settings};
  ASSERT_TRUE(fresh.initialize(qp.P, qp.A, Eigen::Vector2d::Zero(), qp.l, qp.u,
                               settings));
  ASSERT_TRUE(fresh.updateCostVector(qp.q));

  // the raw update keeps OSQP's behavior: it rescales with the q it holds
  affine_mpc::OSQPSolver raw{n, m, settings};
  ASSERT_TRUE(raw.initialize(Eigen::Matrix2d{{4, 1}, {1, 2}}, qp.A,
                             Eigen::Vector2d::Zero(), qp.l, qp.u, settings));
  ASSERT_TRUE(raw.updateCostVector(qp.q));
  ASSERT_TRUE(raw.updateCostMatrix(P_new));

  Eigen::Vector2d z_qp, z_fresh, z_raw;
  ASSERT_EQ(by_qp.solve(z_qp), affine_mpc::SolveStatus::Success);
  ASSERT_EQ(fresh.solve(z_fresh), affine_mpc::SolveStatus::Success);
  ASSERT_EQ(raw.solve(z_raw), affine_mpc::SolveStatus::Success);
  EXPECT_EQ(by_qp.getSolveInfo().iterations, fresh.getSolveInfo().iterations);
  EXPECT_TRUE(expectEigenNear(z_qp, z_fresh, 1e-12));
  EXPECT_NE(by_qp.getSolveInfo().iterations, raw.getSolveInfo().iterations);
}

TEST(OSQPSolverProtectedTester, givenExample_ReportsDualSolution)
{
  const int n{2}, m{3};
  OSQPSolverProtectedTester base{n, m};
  Eigen::Vector3d y{Eigen::Vector3d::Constant(7.0)};
  EXPECT_FALSE(base.getDualSolution(y));
  EXPECT_EQ(y, Eigen::Vector3d::Constant(7.0));

  Eigen::Matrix<OSQPFloat, m, n> A;
  A << 1, 1, 1, 0, 0, 1;
  Eigen::Matrix<OSQPFloat, n, 1> q;
  q.setOnes();
  Eigen::Matrix<OSQPFloat, m, 1> l, u;
  l << 1, 0, 0;
  u << 1, 0.7, 0.7;
  Eigen::Matrix<OSQPFloat, n, n> P;
  P << 4, 1, 1, 2;
  OSQPSettings settings{affine_mpc::OSQPSolver::getDefaultSettings()};
  settings.verbose = false;
  settings.polishing = true;
  ASSERT_TRUE(base.initialize(P, A, q, l, u, settings));
  Eigen::Vector2d solution;
  ASSERT_EQ(base.solve(solution), affine_mpc::SolveStatus::Success);

  // z = (0.3, 0.7): P z + q = (2.9, 2.7) = -A^T y with row 0 an equality,
  // row 1 inactive, and row 2 at its upper bound (positive)
  ASSERT_TRUE(base.getDualSolution(y));
  EXPECT_TRUE(expectEigenNear(y, Eigen::Vector3d{-2.9, 0.0, 0.2}, 1e-5));
}

TEST(OSQPSolverProtectedTester, askedBeforeInitialize_ReturnsSafeDefaults)
{
  const int n{2}, m{3};
  OSQPSolverProtectedTester base{n, m};

  EXPECT_EQ(base.getSolutionMap().size(), 0);

  Eigen::Vector2d solution{1.0, 2.0};
  EXPECT_EQ(base.solve(solution), affine_mpc::SolveStatus::NotInitialized);
  EXPECT_EQ(solution, Eigen::Vector2d(1.0, 2.0));
}

TEST(OSQPSolverProtectedTester, givenColdStart_NextSolveStartsFromZero)
{
  const int n{2}, m{3};
  OSQPSolverProtectedTester base{n, m};
  base.coldStart(); // no-op before initialization

  Eigen::Matrix<OSQPFloat, m, n> A;
  A << 1, 1, 1, 0, 0, 1;
  Eigen::Matrix<OSQPFloat, n, 1> q;
  q.setOnes();
  Eigen::Matrix<OSQPFloat, m, 1> l, u;
  l << 1, 0, 0;
  u << 1, 0.7, 0.7;
  Eigen::Matrix<OSQPFloat, n, n> P;
  P << 4, 1, 1, 2;
  OSQPSettings settings{affine_mpc::OSQPSolver::getRecommendedSettings()};
  settings.verbose = false;
  // deterministic iteration counts: fixed rho, check termination every step
  settings.adaptive_rho = false;
  settings.check_termination = 1;
  ASSERT_TRUE(base.initialize(P, A, q, l, u, settings));

  Eigen::Vector2d first, warm, cold;
  ASSERT_EQ(base.solve(first), affine_mpc::SolveStatus::Success);
  const int iters_from_zero{base.getSolveInfo().iterations};

  // warm start from the converged solution needs fewer iterations
  ASSERT_EQ(base.solve(warm), affine_mpc::SolveStatus::Success);
  EXPECT_LT(base.getSolveInfo().iterations, iters_from_zero);

  // cold start leaves the solution buffer alone
  base.coldStart();
  expectEigenNear(Eigen::Vector2d{base.getSolutionMap()}, warm, 0.0);

  // and the next solve repeats the solve from zero exactly
  ASSERT_EQ(base.solve(cold), affine_mpc::SolveStatus::Success);
  EXPECT_EQ(base.getSolveInfo().iterations, iters_from_zero);
  expectEigenNear(cold, first, 0.0);
}

TEST(OSQPSolverProtectedTester,
     givenBothMatrices_UpdateMatricesMatchesSeparateUpdates)
{
  const int n{2}, m{3};
  Eigen::Matrix<OSQPFloat, m, n> A, A_new;
  A << 1, 1, 1, 0, 0, 1;
  A_new << 1, 2, 1, 0, 0, 1;
  Eigen::Matrix<OSQPFloat, n, n> P, P_new;
  P << 4, 1, 1, 2;
  P_new << 5, 1, 1, 3;
  Eigen::Matrix<OSQPFloat, n, 1> q;
  q.setOnes();
  Eigen::Matrix<OSQPFloat, m, 1> l, u;
  l << 1, 0, 0;
  u << 1, 0.7, 0.7;
  OSQPSettings settings{affine_mpc::OSQPSolver::getDefaultSettings()};
  settings.verbose = false;

  OSQPSolverProtectedTester combined{n, m}, separate{n, m};
  EXPECT_FALSE(combined.updateMatrices(P_new, A_new)); // not initialized
  ASSERT_TRUE(combined.initialize(P, A, q, l, u, settings));
  ASSERT_TRUE(separate.initialize(P, A, q, l, u, settings));

  ASSERT_TRUE(combined.updateMatrices(P_new, A_new));
  ASSERT_TRUE(separate.updateCostMatrix(P_new));
  ASSERT_TRUE(separate.updateConstraintMatrix(A_new));
  expectEigenNear(combined.getPx(), separate.getPx(), 0.0);
  expectEigenNear(combined.getAx(), separate.getAx(), 0.0);

  Eigen::Vector2d x_combined, x_separate;
  ASSERT_EQ(combined.solve(x_combined), affine_mpc::SolveStatus::Success);
  ASSERT_EQ(separate.solve(x_separate), affine_mpc::SolveStatus::Success);
  expectEigenNear(x_combined, x_separate, 1e-12);
}

TEST(OSQPSolverProtectedTester,
     givenFailedMatrixUpdate_RecoversWhenValidDataResent)
{
  const int n{2}, m{3};
  Eigen::Matrix<OSQPFloat, m, n> A;
  A << 1, 1, 1, 0, 0, 1;
  Eigen::Matrix<OSQPFloat, n, n> P, P_bad;
  P << 4, 1, 1, 2;
  P_bad << std::numeric_limits<double>::quiet_NaN(), 1, 1, 2;
  Eigen::Matrix<OSQPFloat, n, 1> q;
  q.setOnes();
  Eigen::Matrix<OSQPFloat, m, 1> l, u;
  l << 1, 0, 0;
  u << 1, 0.7, 0.7;
  OSQPSettings settings{affine_mpc::OSQPSolver::getDefaultSettings()};
  settings.verbose = false;
  settings.polishing = true;

  OSQPSolverProtectedTester base{n, m};
  ASSERT_TRUE(base.initialize(P, A, q, l, u, settings));
  EXPECT_FALSE(base.updateCostMatrix(P_bad));

  // resending the valid matrix and vectors restores the original problem
  ASSERT_TRUE(base.updateCostMatrix(P));
  ASSERT_TRUE(base.updateCostVector(q));
  ASSERT_TRUE(base.updateBounds(l, u));
  Eigen::Vector2d solution;
  ASSERT_EQ(base.solve(solution), affine_mpc::SolveStatus::Success);
  expectEigenNear(solution, Eigen::Vector2d{0.3, 0.7}, 1e-6);
}

TEST(OSQPSolverProtectedTester, givenInfiniteBounds_ConvergesToUnconstrainedMin)
{
  // OSQP marks unbounded rows by |bound| >= OSQP_INFTY; IEEE infinities are
  // valid inputs too but kept OSQP from detecting convergence
  const int n{2}, m{2};
  OSQPSolverProtectedTester base{n, m};
  Eigen::Matrix<OSQPFloat, n, n> P;
  P << 4, 1, 1, 2;
  const Eigen::Matrix<OSQPFloat, m, n> A{Eigen::Matrix2d::Identity()};
  const Eigen::Matrix<OSQPFloat, n, 1> q{1.0, 1.0};
  const double inf{std::numeric_limits<double>::infinity()};
  const Eigen::Matrix<OSQPFloat, m, 1> l{-inf, -inf}, u{inf, inf};
  OSQPSettings settings{affine_mpc::OSQPSolver::getRecommendedSettings()};
  ASSERT_TRUE(base.initialize(P, A, q, l, u, settings));

  const Eigen::Vector2d expected{P.ldlt().solve(-q)};
  Eigen::Vector2d solution;
  ASSERT_EQ(base.solve(solution), affine_mpc::SolveStatus::Success);
  EXPECT_TRUE(expectEigenNear(solution, expected, 1e-5));

  // finite bounds first, then infinite ones through an update
  const Eigen::Matrix<OSQPFloat, m, 1> l_finite{-1.0, -1.0}, u_finite{0, 0};
  ASSERT_TRUE(base.updateBounds(l_finite, u_finite));
  ASSERT_EQ(base.solve(solution), affine_mpc::SolveStatus::Success);
  ASSERT_TRUE(base.updateBounds(l, u));
  ASSERT_EQ(base.solve(solution), affine_mpc::SolveStatus::Success);
  EXPECT_TRUE(expectEigenNear(solution, expected, 1e-5));
}

namespace {

using SparseMatrixXF = affine_mpc::OSQPSolver::SparseMatrixXF;
using MatrixXF = affine_mpc::OSQPSolver::MatrixXF;
using Mask = Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic>;

// M's values on the entries of a mask, stored even where they are zero
SparseMatrixXF stored(const MatrixXF& M, const Mask& mask)
{
  SparseMatrixXF S{M.rows(), M.cols()};
  S.reserve(Eigen::VectorXi::Constant(M.cols(), static_cast<int>(M.rows())));
  for (Eigen::Index j{0}; j < M.cols(); ++j)
    for (Eigen::Index i{0}; i < M.rows(); ++i)
      if (mask(i, j))
        S.insert(i, j) = M(i, j);
  S.makeCompressed();
  return S;
}

struct SparseProblem
{
  MatrixXF P{3, 3}, A{2, 3};
  Mask P_mask{3, 3}, A_mask{2, 3};
  Eigen::Vector3d q{1.0, -1.0, 0.5};
  Eigen::Vector2d l{-1.0, 0.0}, u{1.0, 2.0};
  SparseProblem()
  {
    // stored zeros (P(0, 1), P(0, 2), P(1, 1), A(1, 0)) and a stored lower
    // entry P(1, 0), which the solver ignores
    P << 4, 0, 0, 0, 0, 0, 0, 0, 3;
    P_mask << 1, 1, 1, 1, 1, 0, 0, 0, 1;
    A << 1, 0, 1, 0, 1, 1;
    A_mask << 1, 0, 1, 1, 1, 1;
  }
};

} // namespace

TEST(OSQPSolverProtectedTester, givenSparseMatrices_PatternIsTheStoredEntries)
{
  SparseProblem p;
  OSQPSolverProtectedTester sparse{3, 2};
  OSQPSettings settings{affine_mpc::OSQPSolver::getRecommendedSettings()};
  SparseMatrixXF Ps{stored(p.P, p.P_mask)}, As{stored(p.A, p.A_mask)};
  ASSERT_TRUE(sparse.initialize(Ps, As, p.q, p.l, p.u, settings));

  // every stored entry of P's upper triangle and of A, zero or not (the
  // stored lower entry P(1, 0) is ignored)
  const Eigen::Matrix<OSQPInt, 4, 1> pp_true{0, 1, 3, 5};
  const Eigen::Matrix<OSQPInt, 5, 1> pi_true{0, 0, 1, 0, 2};
  const Eigen::Matrix<OSQPFloat, 5, 1> px_true{4, 0, 0, 0, 3};
  const Eigen::Matrix<OSQPInt, 4, 1> ap_true{0, 2, 3, 5};
  const Eigen::Matrix<OSQPInt, 5, 1> ai_true{0, 1, 1, 0, 1};
  const Eigen::Matrix<OSQPFloat, 5, 1> ax_true{1, 0, 1, 1, 1};
  EXPECT_TRUE(expectEigenNear(sparse.getPp(), pp_true, 0.0));
  EXPECT_TRUE(expectEigenNear(sparse.getPi(), pi_true, 0.0));
  EXPECT_TRUE(expectEigenNear(sparse.getPx(), px_true, 0.0));
  EXPECT_TRUE(expectEigenNear(sparse.getAp(), ap_true, 0.0));
  EXPECT_TRUE(expectEigenNear(sparse.getAi(), ai_true, 0.0));
  EXPECT_TRUE(expectEigenNear(sparse.getAx(), ax_true, 0.0));
  Eigen::Vector3d z_sparse, z_dense;
  ASSERT_EQ(sparse.solve(z_sparse), affine_mpc::SolveStatus::Success);

  // stored entries that were zero become nonzero, through each update; the
  // solutions match a dense solver initialized with the new values
  auto expectSolvesLikeFreshDense = [&]() {
    OSQPSolverProtectedTester dense{3, 2};
    ASSERT_TRUE(dense.initialize(p.P, p.A, p.q, p.l, p.u, settings));
    ASSERT_EQ(dense.solve(z_dense), affine_mpc::SolveStatus::Success);
    ASSERT_EQ(sparse.solve(z_sparse), affine_mpc::SolveStatus::Success);
    EXPECT_TRUE(expectEigenNear(z_sparse, z_dense, 1e-5));
  };
  p.P(0, 1) = p.P(1, 0) = 0.5;
  p.P(1, 1) = 2.0;
  ASSERT_TRUE(sparse.updateCostMatrix(stored(p.P, p.P_mask)));
  p.A(1, 0) = 0.7;
  ASSERT_TRUE(sparse.updateConstraintMatrix(stored(p.A, p.A_mask)));
  expectSolvesLikeFreshDense();
  p.P(0, 2) = p.P(2, 0) = 0.3;
  p.A(0, 0) = 0.0;
  ASSERT_TRUE(
      sparse.updateMatrices(stored(p.P, p.P_mask), stored(p.A, p.A_mask)));
  expectSolvesLikeFreshDense();

  // the update kind must match the initialization
  OSQPSolverProtectedTester dense{3, 2};
  ASSERT_TRUE(dense.initialize(p.P, p.A, p.q, p.l, p.u, settings));
  Ps = stored(p.P, p.P_mask);
  As = stored(p.A, p.A_mask);
  EXPECT_FALSE(dense.updateCostMatrix(Ps));
  EXPECT_FALSE(dense.updateMatrices(Ps, As));
  EXPECT_FALSE(sparse.updateCostMatrix(p.P));
  EXPECT_FALSE(sparse.updateConstraintMatrix(p.A));
  EXPECT_FALSE(dense.isWithinSparsityPattern(Ps, As));
  EXPECT_FALSE(sparse.isWithinSparsityPattern(p.P, p.A));
}

TEST(OSQPSolverProtectedTester, givenSparseFullPattern_UpdatesWithoutCopy)
{
  // every stored entry nonzero: the stored values are OSQP's values
  MatrixXF P{2, 2}, A{2, 2};
  P << 4, 1, 1, 2;
  A << 1, 1, 0, 1;
  Mask P_mask{2, 2}, A_mask{2, 2};
  P_mask << 1, 1, 0, 1;
  A_mask << 1, 1, 0, 1;
  const Eigen::Vector2d q{1.0, 1.0}, l{1.0, 0.0}, u{1.0, 0.7};
  OSQPSolverProtectedTester dense{2, 2}, sparse{2, 2};
  OSQPSettings settings{affine_mpc::OSQPSolver::getRecommendedSettings()};
  ASSERT_TRUE(dense.initialize(P, A, q, l, u, settings));
  ASSERT_TRUE(sparse.initialize(stored(P, P_mask), stored(A, A_mask), q, l, u,
                                settings));
  P << 5, 1.5, 1.5, 1;
  A << 1.2, 1.1, 0, 0.8;
  ASSERT_TRUE(dense.updateMatrices(P, A));
  ASSERT_TRUE(sparse.updateMatrices(stored(P, P_mask), stored(A, A_mask)));
  Eigen::Vector2d z_dense, z_sparse;
  ASSERT_EQ(dense.solve(z_dense), affine_mpc::SolveStatus::Success);
  ASSERT_EQ(sparse.solve(z_sparse), affine_mpc::SolveStatus::Success);
  EXPECT_TRUE(expectEigenNear(z_sparse, z_dense, 0.0));
}

TEST(OSQPSolverProtectedTester, givenSparsePWithoutDiagonal_InitializeFails)
{
  MatrixXF P{2, 2}, A{1, 2};
  P << 1, 0, 0, 1;
  A << 1, 1;
  Mask P_mask{2, 2}, A_mask{1, 2};
  P_mask << 1, 0, 0, 0; // P(1, 1) not stored
  A_mask << 1, 1;
  const Eigen::Vector2d q{0.0, 0.0};
  const Eigen::Matrix<double, 1, 1> l{0.0}, u{1.0};
  OSQPSolverProtectedTester sparse{2, 1};
  OSQPSettings settings{affine_mpc::OSQPSolver::getDefaultSettings()};
  EXPECT_FALSE(sparse.initialize(stored(P, P_mask), stored(A, A_mask), q, l, u,
                                 settings));
}

TEST(OSQPSolverProtectedTester,
     givenSparseStoredEntries_AreWithinSparsityPatternWhateverTheirValues)
{
  SparseProblem p;
  OSQPSolverProtectedTester sparse{3, 2};
  SparseMatrixXF Ps{stored(p.P, p.P_mask)}, As{stored(p.A, p.A_mask)};
  EXPECT_FALSE(sparse.isWithinSparsityPattern(Ps, As)); // not initialized
  OSQPSettings settings{affine_mpc::OSQPSolver::getDefaultSettings()};
  ASSERT_TRUE(sparse.initialize(Ps, As, p.q, p.l, p.u, settings));
  EXPECT_TRUE(sparse.isWithinSparsityPattern(Ps, As));

  // stored entries that were zero at initialization, and a stored lower entry
  // of P (not read)
  Ps.coeffRef(1, 1) = 2.0;
  Ps.coeffRef(0, 2) = 0.5;
  Ps.coeffRef(1, 0) = 7.0;
  As.coeffRef(1, 0) = 1.0;
  EXPECT_TRUE(sparse.isWithinSparsityPattern(Ps, As));
  // explicit zeros
  As.coeffRef(0, 0) = 0.0;
  EXPECT_TRUE(sparse.isWithinSparsityPattern(Ps, As));
}
