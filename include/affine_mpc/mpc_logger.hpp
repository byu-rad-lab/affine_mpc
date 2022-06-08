#ifndef MPC_LOGGER_HPP
#define MPC_LOGGER_HPP

#include <fstream>
#include <Eigen/Core>
#include "affine_mpc/mpc_base.hpp"

using namespace Eigen;

class MPCLogger
{
public:
  MPCLogger(const MPCBase* const mpc,
            const std::string& save_location="/tmp/mpc_data");
  virtual ~MPCLogger();
  void logPreviousSolve(double t0, double ts, const Ref<const VectorXd>& x0,
                        int write_every=1);
  void writeParamFile(const std::string& filename="params.yaml");

protected:
  void handleStringSubstitutions();

private:
  const MPCBase* const mpc_;
  std::string save_dir_;
  bool wrote_params_;
  VectorXd x_traj_, u_traj_;
  std::ofstream time_fout_;
  std::ofstream states_fout_;
  std::ofstream refs_fout_;
  std::ofstream inputs_fout_;
};

#endif // MPC_LOGGER_HPP
