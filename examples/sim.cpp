#include <Eigen/Core>
#include <iostream>
#include <chrono>

#include "affine_mpc/condensed_mpc.hpp"
#include "affine_mpc/mpc_logger.hpp"
#include "affine_mpc/options.hpp"
#include "affine_mpc/parameterization.hpp"

namespace ampc = affine_mpc;
namespace fs = std::filesystem;


int main()
{
  const int n{2}, m{1}, T{10}, p{3};
  const auto param{ampc::Parameterization::linearInterp(T, p)};
  const ampc::Options opts{
      .use_input_cost = true,
      .slew_control_points = true,
  };
  ampc::CondensedMPC msd_mpc{n, m, param, opts};

  // generic option
  const auto savedir{fs::temp_directory_path() / "ampc_example"};

  double ts{0.1};
  ampc::MPCLogger logger{msd_mpc, savedir, ts, 1, false};
  logger.addMetadata("example_name", "mass_spring_damper");

  Eigen::Matrix2d A;
  Eigen::Vector2d B, w;
  A << 0, 1, -0.6, -0.1;
  B << 0, 0.2;
  w.setZero();
  msd_mpc.setModelContinuous2Discrete(A, B, w, ts);

  Eigen::Matrix<double, m, 1> u_min{0}, u_max{3}, slew{1};
  msd_mpc.setInputLimits(u_min, u_max);
  msd_mpc.setSlewRate(slew);

  Eigen::Matrix<double, n, 1> Q_diag{1, 0.11};
  Eigen::Matrix<double, m, 1> R_diag{.0001};
  msd_mpc.setWeights(Q_diag, R_diag);

  Eigen::Vector2d x_goal{1, 0};
  Eigen::Matrix<double, m, 1> u_goal{0.01};
  msd_mpc.setReferenceState(x_goal);
  msd_mpc.setReferenceInput(u_goal);

  if (!msd_mpc.initializeSolver()) {
    std::cerr << "Failed to initialize solver\n";
    return 1;
  }

  Eigen::Vector2d xk;
  xk.setZero();

  Eigen::Matrix<double, m, 1> uk;
  
  affine_mpc::SolveStatus status;
  double tf{5};
  for (double t{0}; t < tf; t += ts) {
    auto start = std::chrono::high_resolution_clock::now();
    status = msd_mpc.solve(xk);
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> diff = end - start;

    if (status != ampc::SolveStatus::Success)
      std::cout << "Solver status: " << status << std::endl;
    
    msd_mpc.getNextInput(uk);
    
    // Use the convenience overload!
    logger.logStep(t, xk, msd_mpc, diff.count());
    
    msd_mpc.propagateModel(xk, uk, xk);
  }

  return 0;
}
