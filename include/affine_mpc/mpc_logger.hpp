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
 * Uses a "write-raw, pack-later" strategy to support high logging frequencies.
 * Per-step data is temporarily stored in binary files and then packed into a
 * single .npz file during finalization. Metadata is stored in a .yaml file.
 *
 * The logger is designed to be used within a simulation or control loop. It
 * provides a convenience method to automatically extract and stride
 * trajectories from an MPC object.
 */
class MPCLogger
{
public:
  using MetadataValue = std::
      variant<int, double, std::string, Eigen::VectorXd, std::vector<double>>;

  /**
   * @brief Construct an MPCLogger for a given MPC instance.
   *
   * The logger automatically captures a snapshot of the MPC object's current
   * parameters (dimensions, weights, limits, etc.) at construction. If these
   * parameters change later, captureMPCSnapshot() should be called manually.
   *
   * @param mpc The MPC object from which to log parameters.
   * @param save_dir Directory to save log files. Created if it doesn't exist.
   * @param ts Model propagation time step used to align predicted trajectories
   *   in time.
   * @param prediction_stride Factor to downsample predicted trajectories.
   *   - 0: Log only the current step (minimal mode).
   *   - 1: Log every step of the horizon.
   *   - K: Log every K-th step of the horizon.
   *   Note: The terminal state (T) is always included if prediction_stride > 0.
   * @param log_control_points If true, logs control points of the parameterized
   *   input trajectory instead of the evaluated dense input trajectory.
   * @param save_name Base name for the .npz output file (default: "log").
   */
  MPCLogger(const MPCBase& mpc,
            const std::filesystem::path& save_dir,
            double ts,
            int prediction_stride = 1,
            bool log_control_points = false,
            const std::string& save_name = "log");

  /**
   * @brief Destructor. Calls finalize() if the logger has not been finalized
   *   yet.
   */
  virtual ~MPCLogger();

  /**
   * @brief Log a single step of data, automatically fetching trajectories and
   *   references from the provided MPC object with applied striding logic.
   *
   * @param t Current simulation time (time of solve).
   * @param x0 Current state at time t (same as provided to solve() since
   *   MPCBase does not store this).
   * @param mpc The MPC object from which to extract predictions and references
   *   (should be the same one that was provided to the constructor - providing
   *   it again ensures the object still exists in memory).
   * @param solve_time Optional user-calculated solve time (likely to include
   *   setup time). The solve time reported by OSQP is also logged separately.
   */
  void logStep(double t,
               const Eigen::Ref<const Eigen::VectorXd>& x0,
               const MPCBase& mpc,
               double solve_time = -1.0);

  /**
   * @brief Add or overwrite custom metadata to be saved in both NPZ and YAML.
   *
   *   User-added metadata is preserved in the order it was added and appears
   *   after the automatic MPC snapshot in the output files.
   *
   * @param key Unique identifier for the metadata entry.
   * @param value The value to store (int, double, string, or VectorXd).
   * @param precision Optional decimal precision for floating point output in
   *   YAML. -1 uses default precision.
   */
  void addMetadata(const std::string& key,
                   const MetadataValue& value,
                   int precision = -1);

  /**
   * @brief Manually capture a snapshot of an MPC object's current parameters.
   *
   *   This is called automatically in the constructor, but can be re-called if
   *   weights or limits are updated during the simulation.
   *
   * @param mpc The MPC object to snapshot.
   */
  void captureMPCSnapshot(const MPCBase& mpc);

  /**
   * @brief Pack all temporary binary data into the final .npz file and write
   *   the parameter YAML file.
   *
   *   This operation involves file I/O and should be called after the
   *   simulation loop ends. Temporary files are deleted upon successful
   *   completion.
   */
  void finalize();

  /**
   * @brief Write the internal metadata map to a YAML file.
   * @param filename Output filename (should end with .yaml or .yml).
   */
  void writeParamFile(const std::filesystem::path& filename = "params.yaml");

private:
  /**
   * @struct MetadataEntry
   * @brief Internal storage for metadata values and their formatting options.
   */
  struct MetadataEntry
  {
    MetadataValue value;
    int precision = -1; ///< -1 for default.
  };

  /**
   * @brief Opens the temporary binary files for writing.
   */
  void initTempFiles();

  /**
   * @brief Internal raw logger: writes strided arrays directly to binary
   *   streams.
   */
  void writeStep(double t,
                 const Eigen::Ref<const Eigen::VectorXd>& states,
                 const Eigen::Ref<const Eigen::VectorXd>& inputs,
                 const Eigen::Ref<const Eigen::VectorXd>& ref_states,
                 const Eigen::Ref<const Eigen::VectorXd>& ref_inputs,
                 const Eigen::Ref<const Eigen::VectorXd>& solve_times);

  int state_dim_, input_dim_, horizon_steps_, num_ctrl_pts_, spline_degree_;
  double ts_;
  int prediction_stride_;
  bool log_control_points_;

  std::vector<int> strided_k_; ///< Pre-computed indices for strided logging.
  int logged_x_dim_, logged_u_dim_;

  std::filesystem::path save_dir_;
  std::string save_name_;
  bool is_finalized_;
  int num_logged_steps_;

  std::vector<std::string> metadata_keys_; ///< Preserves insertion order.
  std::map<std::string, MetadataEntry> metadata_registry_;

  // Internal buffers to avoid re-allocation during high-frequency logging.
  Eigen::VectorXd x_pred_buf_, u_pred_buf_;
  Eigen::VectorXd states_out_buf_, inputs_out_buf_;
  Eigen::VectorXd ref_states_out_buf_, ref_inputs_out_buf_;

  // Temp binary output streams for raw data.
  std::ofstream time_bin_;
  std::ofstream solve_times_bin_;
  std::ofstream states_bin_;
  std::ofstream inputs_bin_;
  std::ofstream ref_states_bin_;
  std::ofstream ref_inputs_bin_;
};

} // namespace affine_mpc

#endif // AFFINE_MPC_MPC_LOGGER_HPP
