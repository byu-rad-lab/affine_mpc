#include <Eigen/Core>
#include <chrono>
#include <filesystem>
#include <iostream>
#include <string>

#include "affine_mpc/condensed_mpc.hpp"
#include "affine_mpc/mpc_logger.hpp"
#include "affine_mpc/options.hpp"
#include "affine_mpc/parameterization.hpp"

namespace ampc = affine_mpc;
namespace fs = std::filesystem;


int main()
{
  const auto savedir{fs::temp_directory_path() / "ampc_example"};
  if (fs::exists(savedir / "log.npz")) {
    std::string answer;
    std::cout << "Log file already exists. Overwrite? [Y/n]: ";
    std::getline(std::cin, answer);
    if (answer.empty())
      answer = "y";
    if (std::tolower(answer.front()) != 'y') {
      std::cout << "Exiting." << std::endl;
      return 0;
    }
  }

  const int n{2}, m{1}, T{10}, nc{3};
  const auto param{ampc::Parameterization::linearInterp(T, nc)};
  const ampc::Options opts{
      .use_input_cost = true,
      .slew_control_points = true,
  };
  ampc::CondensedMPC msd_mpc{n, m, param, opts};

  Eigen::Matrix2d A;
  Eigen::Vector2d B, w;
  A << 0, 1, -0.6, -0.1;
  B << 0, 0.2;
  w.setZero();
  double ts{0.1};
  msd_mpc.setModelContinuous2Discrete(A, B, w, ts);

  Eigen::Matrix<double, m, 1> u_min{0}, u_max{3}, slew{1};
  msd_mpc.setInputLimits(u_min, u_max);
  msd_mpc.setSlewRate(slew);

  Eigen::Matrix<double, n, 1> Q_diag{1, 0.11};
  Eigen::Matrix<double, m, 1> R_diag{.0001};
  msd_mpc.setWeights(Q_diag, R_diag);

  Eigen::Vector2d x_ref{1, 0};
  Eigen::Matrix<double, m, 1> u_ref{0.01};
  msd_mpc.setReferenceState(x_ref);
  msd_mpc.setReferenceInput(u_ref);

  if (!msd_mpc.initializeSolver()) {
    std::cerr << "Failed to initialize solver\n";
    return 1;
  }

  const int pred_stride{1}; // 1 logs every step of each solve horizon
  ampc::MPCLogger logger{&msd_mpc, savedir, ts, pred_stride};
  logger.addMetadata("example_name", "mass_spring_damper");

  Eigen::Vector2d xk;
  xk.setZero(); // set x0

  Eigen::Matrix<double, m, 1> uk;

  double tf{5};
  affine_mpc::SolveStatus status;
  for (double t{0}; t < tf; t += ts) {
    auto start = std::chrono::high_resolution_clock::now();
    status = msd_mpc.solve(xk);
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> diff = end - start;

    if (status != ampc::SolveStatus::Success)
      std::cout << "Solver status: " << status << std::endl;

    msd_mpc.getNextInput(uk);
    logger.logStep(t, xk, diff.count());

    msd_mpc.propagateModel(xk, uk, xk);
  }

  std::cout << "Log file written to " << savedir / "log.npz" << std::endl;

  return 0;
}
