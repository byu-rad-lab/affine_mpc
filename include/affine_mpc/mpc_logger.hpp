#ifndef MPC_LOGGER_HPP
#define MPC_LOGGER_HPP

#include <Eigen/Core>
#include <fstream>

#include "affine_mpc/mpc_base.hpp"

namespace affine_mpc {


class MPCLogger
{
public:
  MPCLogger(const MPCBase* const mpc,
            const std::string& save_location = "/tmp/ampc_data");
  virtual ~MPCLogger();
  void logPreviousSolve(double t0,
                        double ts,
                        const Eigen::Ref<const Eigen::VectorXd>& x0,
                        double solve_time = -1,
                        int write_every = 1);
  void writeParamFile(const std::string& filename = "params.yaml");

protected:
  void handleStringSubstitutions();

private:
  const MPCBase* const mpc_;
  std::string save_dir_;
  bool wrote_params_;
  Eigen::VectorXd x_traj_, u_traj_;
  std::ofstream time_fout_;
  std::ofstream solve_time_fout_;
  std::ofstream states_fout_;
  std::ofstream refs_fout_;
  std::ofstream inputs_fout_;
  std::ofstream spline_knots_fout_;
};

} // namespace affine_mpc

#endif // MPC_LOGGER_HPP
