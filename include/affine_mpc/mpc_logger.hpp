#ifndef AFFINE_MPC_MPC_LOGGER_HPP
#define AFFINE_MPC_MPC_LOGGER_HPP

#include <Eigen/Core>
#include <filesystem>
#include <fstream>
#include <string>

#include "affine_mpc/mpc_base.hpp"

/**
 *  mpc_logger.hpp
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
   * @param save_location Directory to save log files and temp binary data.
   * @param save_name Base name for the final .npz file (default: "log").
   */
  MPCLogger(const MPCBase* const mpc,
            const std::filesystem::path& save_location,
            const std::string& save_name = "log");

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
   * @brief Pack all temp binary data into a single .npz file and write parameters.
   */
  void finalize();

  /**
   * @brief Write MPC parameters and metadata to a YAML file.
   * @param filename Output filename (default: params.yaml).
   */
  void writeParamFile(const std::filesystem::path& filename =
                          std::filesystem::path{"params.yaml"});

private:
  const MPCBase* const mpc_;
  std::filesystem::path save_path_;
  std::string save_name_;
  bool wrote_params_;
  bool is_finalized_;
  int num_logged_steps_;

  Eigen::VectorXd x_traj_, u_traj_;

  // Temp binary output streams for high-frequency logging
  std::ofstream time_bin_;
  std::ofstream solve_times_bin_; // [user_solve_time, osqp_solve_time]
  std::ofstream states_bin_;
  std::ofstream refs_bin_;
  std::ofstream inputs_bin_;
};

} // namespace affine_mpc

#endif // AFFINE_MPC_MPC_LOGGER_HPP
