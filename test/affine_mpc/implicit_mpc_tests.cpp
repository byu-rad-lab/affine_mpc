#include <gtest/gtest.h>

#include "affine_mpc/implicit_mpc.hpp"
#include "affine_mpc/mpc_logger.hpp"
#include "utils.hpp"


// Tests are done with a Mass-Spring-Damper system (2 states, 1 input)
class ImplicitMPCProtectedTester : public affine_mpc::ImplicitMPC
{
  constexpr static int N = 2;
  constexpr static int M = 1;

public:
  using ImplicitMPC::ImplicitMPC;
  virtual ~ImplicitMPCProtectedTester() = default;

  void setModel()
  {
    Eigen::Matrix2d A;
    Eigen::Vector2d B, w;
    A << 0, 1, -0.6, -0.1;
    B << 0, 0.2;
    w.setZero();
    double ts{0.1};
    setModelContinuous2Discrete(A, B, w, ts);
    // Matrix2d Ad;
    // Vector2d Bd, wd;
    // Ad << 0.99701147,0.09940219, -0.05964131,0.98707125;
    // Bd << 0.00099618, 0.01988044;
    // wd << 0.01193257, 0.138468;
    // setModelDiscrete(Ad, Bd, wd);
  }

  void findSAndV(const Eigen::Ref<const Eigen::VectorXd>& x0) { calcSAndV(x0); }

  void findPAndQ(const Eigen::Ref<const Eigen::VectorXd>& x0)
  {
    convertToQP(x0);
  }

  auto getAd() { return this->Ad_; }
  auto getBd() { return this->Bd_; }
  auto getWd() { return this->wd_; }
  auto getQbig() { return this->Q_big_; }
  auto getRbig() { return this->R_big_; }
  auto getS() { return this->S_; }
  auto getV() { return this->v_; }
  auto getP() { return this->P_; }
  auto getQ() { return this->q_; }
};

TEST(ImplicitMPCProtectedTester, givenModel_FormsSandVcorrectly)
{
  const int n{2}, m{1}, T{5}, p{3};
  ImplicitMPCProtectedTester msd_mpc{n, m, T, p};
  msd_mpc.setModel();

  Eigen::Vector2d x0{0, 0.1};
  msd_mpc.findSAndV(x0);

  Eigen::Matrix2d A{msd_mpc.getAd()};
  Eigen::Vector2d B{msd_mpc.getBd()};
  Eigen::Vector2d w{msd_mpc.getWd()};
  Eigen::Matrix2d I{Eigen::Matrix2d::Identity()};
  Eigen::Vector2d zero{Eigen::Vector2d::Zero()};
  Eigen::Matrix<double, n * T, m * p> S_test;
  S_test << B, zero, zero, (A + 0.5 * I) * B, 0.5 * B, zero,
      (A * A + 0.5 * A) * B, (0.5 * A + I) * B, zero,
      (A * A * A + 0.5 * A * A) * B, (0.5 * A * A + A + 0.5 * I) * B, 0.5 * B,
      (A * A * A * A + 0.5 * A * A * A) * B,
      (0.5 * A * A * A + A * A + 0.5 * A) * B, (0.5 * A + I) * B;

  Eigen::Matrix<double, n * T, 1> v_test;
  v_test << A * x0 + w, A * A * x0 + A * w + w,
      A * A * A * x0 + A * A * w + A * w + w,
      A * A * A * A * x0 + A * A * A * w + A * A * w + A * w + w,
      A * A * A * A * A * x0 + A * A * A * A * w + A * A * A * w + A * A * w +
          A * w + w;

  ASSERT_TRUE(expectEigenNear(msd_mpc.getS(), S_test, 1e-6));
  ASSERT_TRUE(expectEigenNear(msd_mpc.getV(), v_test, 1e-6));
}

TEST(ImplicitMPCProtectedTester, givenModel_FormsPandQcorrectly)
{
  const int n{2}, m{1}, T{5}, p{3};
  const bool use_input_cost{true};
  ImplicitMPCProtectedTester msd_mpc{n, m, T, p, use_input_cost};
  msd_mpc.setModel();

  Eigen::Matrix<double, n, 1> Q{1, 1};
  Eigen::Matrix<double, m, 1> R{1}, u_goal{1};
  Eigen::Vector2d x0{0, 0.1};
  Eigen::Matrix<double, n * T, 1> x_goal;
  x_goal.setOnes();

  msd_mpc.setWeights(Q, R);
  msd_mpc.setReferenceStateTrajectory(x_goal);
  msd_mpc.setReferenceInput(u_goal);
  msd_mpc.findPAndQ(x0);

  Eigen::Matrix2d A{msd_mpc.getAd()};
  Eigen::Vector2d B{msd_mpc.getBd()};
  Eigen::Vector2d w{msd_mpc.getWd()};
  Eigen::Matrix2d I{Eigen::Matrix2d::Identity()};
  Eigen::Vector2d zero{Eigen::Vector2d::Zero()};
  Eigen::Matrix<double, n * T, m * p> S;
  S << B, zero, zero, (A + 0.5 * I) * B, 0.5 * B, zero, (A * A + 0.5 * A) * B,
      (0.5 * A + I) * B, zero, (A * A * A + 0.5 * A * A) * B,
      (0.5 * A * A + A + 0.5 * I) * B, 0.5 * B,
      (A * A * A * A + 0.5 * A * A * A) * B,
      (0.5 * A * A * A + A * A + 0.5 * A) * B, (0.5 * A + I) * B;

  Eigen::Matrix<double, n * T, 1> v;
  v << A * x0 + w, A * A * x0 + A * w + w,
      A * A * A * x0 + A * A * w + A * w + w,
      A * A * A * A * x0 + A * A * A * w + A * A * w + A * w + w,
      A * A * A * A * A * x0 + A * A * A * A * w + A * A * A * w + A * A * w +
          A * w + w;

  Eigen::Matrix<double, m * p, m * p> P_test;
  P_test = msd_mpc.getRbig();
  P_test += S.transpose() * S;

  Eigen::Matrix<double, m * p, 1> q_test;
  q_test.setConstant(-1);
  q_test += S.transpose() * (v - x_goal);

  ASSERT_TRUE(expectEigenNear(msd_mpc.getP(), P_test, 1e-6));
  ASSERT_TRUE(expectEigenNear(msd_mpc.getQ(), q_test, 1e-6));
}

TEST(ImplicitMPCProtectedTester, initializedAndAskedToSolve_SolvesCorrecly)
{
  const int n{2}, m{1}, T{10}, p{10};
  const bool use_input_cost{true}, use_slew_rate{true};
  ImplicitMPCProtectedTester msd_mpc{n, m, T, p, use_input_cost, use_slew_rate};
  affine_mpc::MPCLogger logger{&msd_mpc, "~/tmp/mpc_data"};
  msd_mpc.setModel();

  Eigen::Matrix<double, n, 1> Q{1, 0.11};
  Eigen::Matrix<double, m, 1> R{.0001};
  msd_mpc.setWeights(Q, R);
  Eigen::Vector2d x0{1, 1}, x_goal{1, 0};
  msd_mpc.setReferenceState(x_goal);
  Eigen::Matrix<double, m, 1> u_goal{0}, u_min{0}, u_max{3}, slew{1};
  ASSERT_TRUE(msd_mpc.setInputLimits(u_min, u_max));
  msd_mpc.setReferenceInput(u_goal);
  ASSERT_TRUE(msd_mpc.setSlewRate(slew));

  // These are the recommended settings, but explicitly set here for clarity
  OSQPSettings settings{affine_mpc::OSQPSolver::getDefaultSettings()};
  settings.alpha = 1.0;
  settings.verbose = false;
  settings.eps_abs = 1e-6;
  settings.eps_rel = 1e-6;
  ASSERT_TRUE(msd_mpc.initializeSolver(settings));
  x0.setZero();

  Eigen::Matrix<double, m, 1> u_star, u_star_expected;
  bool solved;
  solved = msd_mpc.solve(x0);
  msd_mpc.getNextInput(u_star);
  logger.logPreviousSolve(0, 0.1, x0);

  u_star_expected << 3.0;

  ASSERT_TRUE(expectEigenNear(u_star, u_star_expected, 1e-5));

  Eigen::Matrix<double, m * p, 1> u_traj;
  solved = msd_mpc.solve(x0);
  msd_mpc.getParameterizedInputTrajectory(u_traj);
  logger.logPreviousSolve(0, 0.1, x0);
  int slew_errors{0};
  for (int i{0}; i < p - 1; ++i) {
    slew_errors += abs(u_traj(i + 1) - u_traj(i)) > slew(0) + 1e-6;
  }

  ASSERT_EQ(slew_errors, 0);
}
