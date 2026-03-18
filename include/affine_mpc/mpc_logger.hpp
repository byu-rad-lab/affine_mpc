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
 * Stores per-step data in an .npz file and human-readable metadata in a .yaml file.
 */
class MPCLogger
{
public:
  using MetadataValue =
      std::variant<int, double, std::string, Eigen::VectorXd, std::vector<double>>;

  struct MetadataEntry
  {
    MetadataValue value;
    int precision = -1; // -1 means use default
  };

  /**
   * @brief Construct an MPCLogger for a given MPC instance (automatic metadata).
   */
  MPCLogger(const MPCBase* const mpc,
            const std::filesystem::path& save_location,
            const std::string& save_name = "log");

  /**
   * @brief Generic constructor for non-MPC use cases (manual metadata).
   */
  MPCLogger(int state_dim,
            int input_dim,
            int horizon_steps,
            const std::filesystem::path& save_location,
            const std::string& save_name = "log");

  virtual ~MPCLogger();

  /**
   * @brief Log a single step of data.
   * @param t Current simulation time.
   * @param x Current state (size state_dim).
   * @param u Current applied input (size input_dim).
   * @param x_pred Predicted state trajectory (size state_dim * horizon_steps or empty).
   * @param u_pred Predicted input trajectory (size input_dim * horizon_steps or empty).
   * @param solve_time User-calculated solve time (optional).
   * @param osqp_solve_time OSQP-calculated solve time (optional).
   */
  void logStep(double t,
               const Eigen::Ref<const Eigen::VectorXd>& x,
               const Eigen::Ref<const Eigen::VectorXd>& u,
               const Eigen::Ref<const Eigen::VectorXd>& x_pred = Eigen::VectorXd{},
               const Eigen::Ref<const Eigen::VectorXd>& u_pred = Eigen::VectorXd{},
               double solve_time = -1.0,
               double osqp_solve_time = -1.0);

  /**
   * @brief Add or overwrite metadata to be saved in both NPZ and YAML.
   */
  void addMetadata(const std::string& key,
                   const MetadataValue& value,
                   int precision = -1);

  /**
   * @brief Pack all data and metadata into the final .npz and .yaml files.
   */
  void finalize();

  /**
   * @brief Write the internal metadata map to a YAML file.
   */
  void writeParamFile(const std::filesystem::path& filename = "params.yaml");

private:
  void initTempFiles();
  void captureMPCSnapshot();

  const MPCBase* const mpc_;
  int state_dim_, input_dim_, horizon_steps_;
  std::filesystem::path save_path_;
  std::string save_name_;
  bool is_finalized_;
  int num_logged_steps_;

  std::vector<std::string> metadata_keys_;
  std::map<std::string, MetadataEntry> metadata_registry_;

  // Temp binary output streams
  std::ofstream time_bin_;
  std::ofstream solve_times_bin_;
  std::ofstream states_bin_;
  std::ofstream inputs_bin_;
  std::ofstream x_pred_bin_;
  std::ofstream u_pred_bin_;
};

} // namespace affine_mpc

#endif // AFFINE_MPC_MPC_LOGGER_HPP
