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
 *   Uses a "write-raw, pack-later" strategy to support high logging
 *   frequencies. Per-step data is always written to raw binary payload files
 *   during logging. Finalization then either keeps those raw payloads with
 *   recovery metadata, converts them to standalone `.npy` files, or packages
 *   them into a single `.npz` archive. Simulation metadata is stored in a YAML
 *   file.
 *
 *   The logger is designed to be used within a simulation or control loop. It
 *   provides a convenience method to automatically extract and stride
 *   trajectories from an MPC object.
 */
class MPCLogger
{
public:
  /**
   * @brief Controls how logged payloads are finalized after raw binary capture.
   *
   *   - RawRecoverable: Write each data array to 2 files: a NPY header
   *     (`.npyh`) + binary (`.bin`). Also write a `data_info.yaml` with
   *     readable header information.
   *   - Npy: Write each data array to a NPY (`.npy`) file.
   *   - NpzUncompressed: Write all data arrays into a single uncompressed NPZ
   *     (`.npz`) file.
   *   - NpzCompressed: Write all data arrays into a single compressed NPZ
   *     (`.npz`) file.
   */
  enum class Mode
  {
    RawRecoverable,  ///< Writes `*.bin`, `*.npyh`, and `data_info.yaml`
    Npy,             ///< Writes `*.npy`
    NpzUncompressed, ///< Writes uncompressed `.npz`
    NpzCompressed,   ///< Writes compressed `.npz`
  };

  using MetadataValue = std::
      variant<int, double, std::string, Eigen::VectorXd, std::vector<double>>;

  /**
   * @brief Construct an MPCLogger for a given MPC instance; the logger is
   *   linked to this single MPC instance.
   *
   *   IMPORTANT: The logger does not own the MPC object; the MPC instance must
   *   outlive the logger.
   *
   *   The logger automatically captures a snapshot of the MPC object's current
   *   parameters (dimensions, weights, limits, etc.) at construction. If these
   *   parameters change later, captureMPCSnapshot() should be called manually.
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
   * @param save_name Base name for the output artifact(s) (default: "log").
   * @param mode Finalization output mode. Raw payload files are always staged
   *   under `<save_name>_raw/` during logging.
   *   - RawRecoverable: saves data `*.bin`, NPY header info `*.npyh`, and
   *     readable header info `data_info.yaml` all to `<save_name>_raw/`.
   *   - NPY: saves data arrays to `<save_name>_npy/*.npy`.
   *   - NpzUncompressed: saves all data into an uncompressed `<save_name>.npz`.
   *   - NpzCompressed: saves all data into a compressed `<save_name>.npz`.
   *
   *   NPZ output is limited by ZIP32 size bounds of about 4 GiB. If NPZ
   *   finalization exceeds those limits, the logger falls back to NPY output
   *   and preserves raw recoverable payloads on failure.
   */
  MPCLogger(const MPCBase* const mpc,
            const std::filesystem::path& save_dir,
            double ts,
            int prediction_stride = 1,
            bool log_control_points = false,
            const std::string& save_name = "log",
            Mode mode = Mode::NpzCompressed);

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
   * @param solve_time Optional user-calculated solve time (likely to include
   *   setup time). The solve time reported by OSQP is also logged separately.
   */
  void logStep(double t,
               const Eigen::Ref<const Eigen::VectorXd>& x0,
               double solve_time = -1.0);

  /**
   * @brief Add or overwrite custom metadata to be saved in both the main output
   *   artifact and YAML metadata when supported.
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
   */
  void captureMPCSnapshot();

  /**
   * @brief Finalize the logger output according to the configured logging mode.
   *
   *   This operation involves file I/O and should be called after the
   *   simulation loop ends. Raw payloads are preserved on failure to support
   *   recovery.
   */
  void finalize();

  /**
   * @brief Write the internal metadata map to a YAML file within `save_dir`.
   * @param filename Output filename (should end with .yaml or .yml).
   */
  void writeParamFile(const std::filesystem::path& filename = "params.yaml");

private:
  struct MetadataEntry
  {
    MetadataValue value;
    int precision = -1;
  };

  struct PayloadEntry
  {
    std::string name;
    std::filesystem::path data_path;
    std::filesystem::path header_path;
    std::vector<size_t> shape;
  };

  void initRawFiles();
  void closeRawFiles();
  std::vector<PayloadEntry> buildPayloadEntries() const;
  void cleanupRawDirectory() const;
  void finalizeRawRecoverable(const std::vector<PayloadEntry>& payloads);
  void finalizeNpy(const std::vector<PayloadEntry>& payloads);
  void finalizeNpz(const std::vector<PayloadEntry>& payloads, bool compress);
  void writeNpzFromPayloads(const std::vector<PayloadEntry>& payloads,
                            const std::filesystem::path& npz_path,
                            bool compress) const;
  void writeNpyOutputs(const std::vector<PayloadEntry>& payloads,
                       const std::filesystem::path& npy_dir) const;
  void writeDataInfoFile(const std::vector<PayloadEntry>& payloads,
                         const std::filesystem::path& data_info_path) const;
  void writeParamFileToPath(const std::filesystem::path& path) const;
  void writeStep(double t);

  std::filesystem::path rawDirPath() const;
  std::filesystem::path rawDataPath(const std::string& name) const;
  std::filesystem::path rawHeaderPath(const std::string& name) const;
  std::filesystem::path npyDirPath() const;
  std::filesystem::path npzPath() const;

  const MPCBase* const mpc_;
  const int state_dim_, input_dim_, horizon_steps_, num_ctrls_, spline_degree_;
  const double ts_;
  const int prediction_stride_;
  const bool log_control_points_;
  const Mode logging_mode_;

  std::vector<int> strided_k_;
  Eigen::Index logged_x_dim_, logged_u_dim_;

  std::filesystem::path save_dir_;
  std::string save_name_;
  bool is_finalized_;
  size_t num_logged_steps_;

  std::vector<std::string> metadata_keys_;
  std::map<std::string, MetadataEntry> metadata_registry_;

  Eigen::Vector2d solve_times_buf_;
  Eigen::VectorXd x_traj_buf_, u_traj_buf_;
  Eigen::VectorXd states_out_buf_, inputs_out_buf_;
  Eigen::VectorXd ref_states_out_buf_, ref_inputs_out_buf_;

  std::ofstream time_bin_;
  std::ofstream solve_times_bin_;
  std::ofstream states_bin_;
  std::ofstream inputs_bin_;
  std::ofstream ref_states_bin_;
  std::ofstream ref_inputs_bin_;
};

} // namespace affine_mpc

#endif // AFFINE_MPC_MPC_LOGGER_HPP
