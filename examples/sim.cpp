#include <Eigen/Core>
#include <iostream>

#include "affine_mpc/implicit_mpc.hpp"
#include "affine_mpc/mpc_logger.hpp"

namespace ampc = affine_mpc;


int main()
{
  const int n{2},m{1},T{10},p{3};
  const bool use_input_cost{true}, use_slew_rate{true};
  ampc::ImplicitMPC msd_mpc{n,m,T,p,use_input_cost,use_slew_rate};

  ampc::MPCLogger logger{&msd_mpc, "/tmp/ampc_example"};

  Eigen::Matrix2d A;
  Eigen::Vector2d B, w;
  A << 0,1, -0.6,-0.1;
  B << 0, 0.2;
  w.setZero();
  double ts{0.1};
  msd_mpc.setModelContinuous2Discrete(A, B, w, ts);

  Eigen::Matrix<double,m,1> u_min{0}, u_max{3}, slew{1};
  msd_mpc.setInputLimits(u_min, u_max);
  msd_mpc.setSlewRate(slew);

  Eigen::Matrix<double,n,1> Q_diag{1,0.11};
  Eigen::Matrix<double,m,1> R_diag{.0001};
  msd_mpc.setWeights(Q_diag, R_diag);

  Eigen::Vector2d x_goal{1,0};
  Eigen::Matrix<double,m,1> u_goal{0.01};
  msd_mpc.setReferenceState(x_goal);
  msd_mpc.setReferenceInput(u_goal);

  msd_mpc.initializeSolver();

  Eigen::Vector2d xk;
  xk.setZero();

  Eigen::Matrix<double,m,1> uk;
  bool solved;
  double tf{5};
  for (double t{0}; t < tf; t += ts)
  {
    solved = msd_mpc.solve(xk);
    if (!solved)
      std::cout << "Did not solve :(" << std::endl;
    msd_mpc.getNextInput(uk);
    logger.logPreviousSolve(t, ts, xk);
    msd_mpc.propagateModel(xk, uk, xk);
  }

  return 0;
}
