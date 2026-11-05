// Sets up and solves a mass-spring-damper MPC with an installed affine_mpc,
// with OSQP and, if the library was built with it, DAQP.
#include <affine_mpc/affine_mpc.hpp>

#if defined(OSQP_H) || defined(OSQP_API_TYPES_H) || defined(OSQP_CONFIGURE_H)
#error "a public affine_mpc header includes an OSQP header"
#endif

#include <Eigen/Core>
#include <iostream>

namespace ampc = affine_mpc;

namespace {

bool solveWith(const char* name, const bool daqp)
{
  const auto param{ampc::Parameterization::moveBlocking(10, 3)};
  ampc::MPC mpc{2, 1, param};
  Eigen::Matrix2d Ac;
  Ac << 0, 1, -0.6, -0.1;
  mpc.setModelContinuous2Discrete(Ac, Eigen::Vector2d{0, 0.2},
                                  Eigen::Vector2d::Zero(), 0.1);
  mpc.setInputLimits(Eigen::VectorXd::Constant(1, 0.0),
                     Eigen::VectorXd::Constant(1, 3.0));
  mpc.setStateWeights(Eigen::Vector2d{1.0, 0.1});
  mpc.setReferenceState(Eigen::Vector2d{1.0, 0.0});

  ampc::OSQPSettings settings{ampc::OSQPSolver::getRecommendedSettings()};
  settings.max_iter = 2000;
  const bool initialized{daqp ? mpc.initializeSolver(ampc::DAQPSettings{})
                              : mpc.initializeSolver(settings)};
  if (!initialized) {
    std::cerr << name << ": initialization failed\n";
    return false;
  }
  const ampc::SolveStatus status{mpc.solve(Eigen::Vector2d::Zero())};
  std::cout << name << ": " << status << '\n';
  return status == ampc::SolveStatus::Success;
}

} // namespace

int main()
{
  bool ok{solveWith("OSQP", false)};
  if (ampc::hasDAQP())
    ok = solveWith("DAQP", true) && ok;
  return ok ? 0 : 1;
}
