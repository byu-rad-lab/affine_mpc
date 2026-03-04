#include <Eigen/Core>
#include <gtest/gtest.h>

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
  const int n{6}, m{4}, P_nnz_max{6};
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
  const int n{6}, m{4}, P_nnz_max{6}, A_nnz_max{4};
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
  const int n{6}, m{4}, P_nnz_max{6};
  OSQPSolverProtectedTester base{n, m};

  Eigen::Matrix<OSQPFloat, n, n> P;
  P << 1, 0, 0, 0, 4, 0, 1, 0, 0, 2, 0, 0, 1, 1, 0, 0, 5, 0, 1, 1, 1, 3, 0, 0,
      1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 6;

  bool success = base.testInitializeP(P);
  ASSERT_TRUE(!success); // non-convex test case should fail

  Eigen::Matrix<OSQPFloat, 6, 1> px_true;
  px_true << 1, 2, 3, 4, 5, 6;
  Eigen::Matrix<OSQPInt, 6, 1> pi_true;
  pi_true << 0, 1, 3, 0, 2, 5;
  Eigen::Matrix<OSQPInt, 7, 1> pp_true;
  pp_true << 0, 1, 1, 1, 3, 5, 6;

  ASSERT_TRUE(expectEigenNear(px_true, base.getPx(), 1e-6));
  ASSERT_TRUE(expectEigenNear(pi_true, base.getPi(), 1e-6));
  ASSERT_TRUE(expectEigenNear(pp_true, base.getPp(), 1e-6));
}

TEST(OSQPSolverProtectedTester, givenExampleA_FormsCscACorrectly)
{
  const int n{2}, m{3}, A_nnz_max{4};
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
  const int n{6}, m{2}, P_nnz_max{6};
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
  const int n{6}, m{2}, P_nnz_max{6}, A_nnz_max{2};
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
