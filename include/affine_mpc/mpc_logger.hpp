#ifndef AFFINE_MPC_MPC_LOGGER_HPP
#define AFFINE_MPC_MPC_LOGGER_HPP

#include <Eigen/Core>
#include <filesystem>
#include <fstream>

#include "affine_mpc/mpc_base.hpp"

/**
 * @file mpc_logger.hpp
 * @brief Defines the MPCLogger class for logging MPC solve results and
 *   parameters.
 */

namespace affine_mpc {

/**
 * @class MPCLogger
 * @brief Utility for logging MPC solve results, trajectories, and parameters to
 *   files.
 *
 * Logs state, input, reference trajectories, solve times, and parameterization
 * metadata for later analysis and visualization. Intended for use with MPCBase
 * and derived classes.
 */
class MPCLogger
{
public:
  /**
   * @brief Construct an MPCLogger for a given MPC instance.
   * @param mpc Pointer to MPCBase instance to log.
   * @param save_location Directory to save log files (default: /tmp/ampc_data).
   */
  MPCLogger(const MPCBase* const mpc,
            const std::string& save_location = "/tmp/ampc_data");

  // MPCLogger(const MPCBase* const mpc, const std::filesystem::path&
  // save_location) :
  //     MPCLogger(mpc, save_location.string())
  // {
  //   save_location.
  // }

  virtual ~MPCLogger();

  /**
   * @brief Log the results of the previous MPC solve, including the current
   *   state, full input trajectory, and predicted state trajectory.
   * @param t0 Initial time of the solve (e.g. current time at solve).
   * @param ts Discretization time step.
   * @param x0 Initial state at the start of the horizon (e.g. current state at
   *   solve).
   * @param solve_time Time taken to solve (optional). This is if you recorded
   *   the time separately (likely to include setup time). The solve time
   *   reported by OSQP is also logged separately.
   * @param write_every Write frequency during horizon. For example, if 2, then
   *   data will be written for every other time step in the horizon. However,
   *   the final time step will always be written regardless of this parameter.
   */
  void logPreviousSolve(double t0,
                        double ts,
                        const Eigen::Ref<const Eigen::VectorXd>& x0,
                        const double solve_time = -1,
                        const int write_every = 1);

  /**
   * @brief Write MPC parameters and metadata to a YAML file.
   * @param filename Output filename (default: params.yaml).
   */
  void writeParamFile(const std::string& filename = "params.yaml");

protected:
  /**
   * @brief Handle string substitutions for file naming and metadata.
   */
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

#endif // AFFINE_MPC_MPC_LOGGER_HPP
