#ifndef AFFINE_MPC_MPC_LOGGER_HPP
#define AFFINE_MPC_MPC_LOGGER_HPP

#include <Eigen/Core>
#include <filesystem>
#include <fstream>
#include <map>
#include <string>
#include <variant>
#include <vector>

#include "affine_mpc/mpc_base.hpp"

namespace affine_mpc {

/**
 * @class MPCLogger
 * @brief High-performance binary logger for MPC data and metadata.
 *
 * Uses a "write-raw, pack-later" strategy to support 1kHz+ logging frequencies.
 * Stores per-step data in an .npz file and human-readable metadata in a .yaml
 * file.
 */
class MPCLogger
{
public:
  using MetadataValue = std::
      variant<int, double, std::string, Eigen::VectorXd, std::vector<double>>;

  struct MetadataEntry
  {
    MetadataValue value;
    int precision = -1; // -1 means use default
  };

  /**
   * @brief Construct an MPCLogger for a given MPC instance.
   * @param mpc The MPC object to log parameters from.
   * @param save_location Directory to save log files.
   * @param ts Simulation time step for predicting future timestamps.
   * @param prediction_stride Downsample factor for predicted trajectories. 0
   * logs only current step.
   * @param log_control_points If true, logs raw control points instead of
   * evaluated input trajectory.
   * @param save_name Base name for the log file.
   */
  MPCLogger(const MPCBase& mpc,
            const std::filesystem::path& save_location,
            double ts,
            int prediction_stride = 1,
            bool log_control_points = false,
            const std::string& save_name = "log");

  virtual ~MPCLogger();

  /**
   * @brief Convenience overload: Automatically fetches trajectories and
   * references from MPC.
   * @param t Current simulation time.
   * @param x0 Current state.
   * @param mpc The MPC object to log.
   * @param solve_time User-calculated solve time (optional).
   */
  void logStep(double t,
               const Eigen::Ref<const Eigen::VectorXd>& x0,
               const MPCBase& mpc,
               double solve_time = -1.0);

  /**
   * @brief Add or overwrite metadata to be saved in both NPZ and YAML.
   */
  void addMetadata(const std::string& key,
                   const MetadataValue& value,
                   int precision = -1);

  /**
   * @brief Manually capture a snapshot of an MPC object's current parameters.
   */
  void captureMPCSnapshot(const MPCBase& mpc);

  /**
   * @brief Pack all data and metadata into the final .npz and .yaml files.
   */
  void finalize();

  /**
   * @brief Write the internal metadata map to a YAML file.
   */
  void writeParamFile(const std::filesystem::path& filename = "params.yaml");

private:
  /**
   * @brief Internal raw logger: logs strided arrays directly to binary.
   */
  void logStep(double t,
               const Eigen::Ref<const Eigen::VectorXd>& states,
               const Eigen::Ref<const Eigen::VectorXd>& inputs,
               const Eigen::Ref<const Eigen::VectorXd>& ref_states,
               const Eigen::Ref<const Eigen::VectorXd>& ref_inputs,
               const Eigen::Ref<const Eigen::VectorXd>& solve_times);

  void initTempFiles();

  int state_dim_, input_dim_, horizon_steps_, num_ctrl_pts_, spline_degree_;
  double ts_;
  int prediction_stride_;
  bool log_control_points_;

  std::vector<int> strided_k_;
  int logged_x_dim_, logged_u_dim_;

  std::filesystem::path save_path_;
  std::string save_name_;
  bool is_finalized_;
  int num_logged_steps_;

  std::vector<std::string> metadata_keys_;
  std::map<std::string, MetadataEntry> metadata_registry_;

  // Internal buffers to avoid re-allocation during logging
  Eigen::VectorXd x_pred_buf_, u_pred_buf_;
  Eigen::VectorXd states_out_buf_, inputs_out_buf_;
  Eigen::VectorXd ref_states_out_buf_, ref_inputs_out_buf_;

  // Temp binary output streams
  std::ofstream time_bin_;
  std::ofstream solve_times_bin_;
  std::ofstream states_bin_;
  std::ofstream inputs_bin_;
  std::ofstream ref_states_bin_;
  std::ofstream ref_inputs_bin_;
};

} // namespace affine_mpc

#endif // AFFINE_MPC_MPC_LOGGER_HPP
