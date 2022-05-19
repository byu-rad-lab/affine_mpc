#ifndef MPC_LOGGER_HPP
#define MPC_LOGGER_HPP

#include <fstream>
// #include <string>
#include <Eigen/Core>
#include "affine_mpc/mpc_base.hpp"

using namespace Eigen;

class MPCLogger
{
public:
  MPCLogger(const MPCBase* const mpc,
            const std::string& time_file="/tmp/mpc_time.txt",
            const std::string& states_file="/tmp/mpc_states.txt",
            const std::string& ref_states_file="/tmp/mpc_ref_states.txt",
            const std::string& inputs_file="/tmp/mpc_inputs.txt");
  virtual ~MPCLogger();
  void writeData(double t0, double ts, const Ref<const VectorXd>& x0,
                 const Ref<const VectorXd>& u_traj, const Ref<const VectorXd>& x_traj_des,
                 int write_every=1);

private:
  const MPCBase* const mpc_;
  VectorXd x_traj_;
  const bool write_x_, write_r_, write_u_;
  std::ofstream time_fout_;
  std::ofstream states_fout_;
  std::ofstream refs_fout_;
  std::ofstream inputs_fout_;
};

#endif // MPC_LOGGER_HPP
