#include "affine_mpc/mpc_logger.hpp"

#include <cassert>
#include <cnpy.h>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>

namespace affine_mpc {

namespace {
const Eigen::IOFormat kVectorFormat(Eigen::StreamPrecision,
                                    Eigen::DontAlignCols,
                                    ", ",
                                    ", ",
                                    "",
                                    "",
                                    "[",
                                    "]");

std::string vec2Str(const Eigen::VectorXd& vec, int precision)
{
  if (vec.size() == 0)
    return "[]";
  std::stringstream ss;
  if (precision >= 0)
    ss << std::fixed << std::setprecision(precision);
  ss << vec.format(kVectorFormat);
  return ss.str();
}

void writeBinary(std::ofstream& fout, const Eigen::Ref<const Eigen::VectorXd>& vec)
{
  if (vec.size() > 0)
    fout.write(reinterpret_cast<const char*>(vec.data()),
               vec.size() * sizeof(double));
}

std::vector<double> loadBinary(const std::filesystem::path& path)
{
  std::ifstream fin(path, std::ios::binary | std::ios::ate);
  if (!fin.is_open())
    return {};
  auto size = fin.tellg();
  fin.seekg(0, std::ios::beg);
  std::vector<double> data(size / sizeof(double));
  fin.read(reinterpret_cast<char*>(data.data()), size);
  return data;
}
} // namespace

MPCLogger::MPCLogger(const MPCBase* const mpc,
                     const std::filesystem::path& save_location,
                     const std::string& save_name) :
    mpc_{mpc},
    state_dim_{mpc->state_dim_},
    input_dim_{mpc->input_dim_},
    horizon_steps_{mpc->horizon_steps_},
    save_path_{save_location.lexically_normal()},
    save_name_{save_name},
    is_finalized_{false},
    num_logged_steps_{0}
{
  captureMPCSnapshot();
  initTempFiles();
}

MPCLogger::MPCLogger(int state_dim,
                     int input_dim,
                     int horizon_steps,
                     const std::filesystem::path& save_location,
                     const std::string& save_name) :
    mpc_{nullptr},
    state_dim_{state_dim},
    input_dim_{input_dim},
    horizon_steps_{horizon_steps},
    save_path_{save_location.lexically_normal()},
    save_name_{save_name},
    is_finalized_{false},
    num_logged_steps_{0}
{
  addMetadata("state_dim", state_dim);
  addMetadata("input_dim", input_dim);
  addMetadata("horizon_steps", horizon_steps);
  initTempFiles();
}

MPCLogger::~MPCLogger()
{
  try {
    if (!is_finalized_)
      finalize();
  } catch (const std::exception& e) {
    std::cerr << "[MPCLogger Destructor] Error: " << e.what() << std::endl;
  }
}

void MPCLogger::initTempFiles()
{
  if (!std::filesystem::exists(save_path_))
    std::filesystem::create_directories(save_path_);

  auto open_bin = [&](std::ofstream& fout, const std::string& name) {
    fout.open(save_path_ / (save_name_ + "_" + name + ".tmp"),
              std::ios::binary | std::ios::out);
  };

  open_bin(time_bin_, "time");
  open_bin(solve_times_bin_, "solve_times");
  open_bin(states_bin_, "states");
  open_bin(inputs_bin_, "inputs");
  open_bin(x_pred_bin_, "x_pred");
  open_bin(u_pred_bin_, "u_pred");
}

void MPCLogger::captureMPCSnapshot()
{
  if (!mpc_)
    return;
  // User preferred order:
  // state_dim, input_dim, horizon_steps, degree, num_control_points, knots, opts
  addMetadata("state_dim", mpc_->state_dim_);
  addMetadata("input_dim", mpc_->input_dim_);
  addMetadata("horizon_steps", mpc_->horizon_steps_);
  addMetadata("spline_degree", mpc_->spline_degree_);
  addMetadata("num_control_points", mpc_->num_ctrl_pts_);
  
  if (mpc_->spline_knots_.size() > 0)
    addMetadata("knots", mpc_->spline_knots_);

  // Options
  addMetadata("opt_use_input_cost", (int)mpc_->opts_.use_input_cost);
  addMetadata("opt_slew_initial_input", (int)mpc_->opts_.slew_initial_input);
  addMetadata("opt_slew_control_points", (int)mpc_->opts_.slew_control_points);
  addMetadata("opt_saturate_states", (int)mpc_->opts_.saturate_states);
  addMetadata("opt_saturate_input_trajectory", (int)mpc_->opts_.saturate_input_trajectory);

  // Constraints/Weights
  addMetadata("u_min", mpc_->u_min_, 3);
  addMetadata("u_max", mpc_->u_max_, 3);
  addMetadata("Q_diag", mpc_->Q_big_.diagonal().head(mpc_->state_dim_), 3);
  addMetadata("Qf_diag", mpc_->Q_big_.diagonal().tail(mpc_->state_dim_), 3);

  if (mpc_->opts_.use_input_cost)
    addMetadata("R_diag", mpc_->R_big_.diagonal().head(mpc_->input_dim_), 3);
  
  if (mpc_->opts_.slew_control_points)
    addMetadata("u_slew", mpc_->u_slew_, 3);
  
  if (mpc_->opts_.saturate_states) {
    addMetadata("x_min", mpc_->x_min_, 3);
    addMetadata("x_max", mpc_->x_max_, 3);
  }
}

void MPCLogger::addMetadata(const std::string& key,
                            const MetadataValue& value,
                            int precision)
{
  if (metadata_registry_.find(key) == metadata_registry_.end()) {
    metadata_keys_.push_back(key);
  }
  metadata_registry_[key] = {value, precision};
}

void MPCLogger::logStep(double t,
                        const Eigen::Ref<const Eigen::VectorXd>& x,
                        const Eigen::Ref<const Eigen::VectorXd>& u,
                        const Eigen::Ref<const Eigen::VectorXd>& x_pred,
                        const Eigen::Ref<const Eigen::VectorXd>& u_pred,
                        double solve_time,
                        double osqp_solve_time)
{
  time_bin_.write(reinterpret_cast<const char*>(&t), sizeof(double));
  writeBinary(states_bin_, x);
  writeBinary(inputs_bin_, u);
  writeBinary(x_pred_bin_, x_pred);
  writeBinary(u_pred_bin_, u_pred);

  double times[2] = {solve_time, osqp_solve_time};
  solve_times_bin_.write(reinterpret_cast<const char*>(times), 2 * sizeof(double));
  num_logged_steps_++;
}

void MPCLogger::finalize()
{
  if (is_finalized_)
    return;

  time_bin_.close();
  solve_times_bin_.close();
  states_bin_.close();
  inputs_bin_.close();
  x_pred_bin_.close();
  u_pred_bin_.close();

  const std::string npz_path = (save_path_ / (save_name_ + ".npz")).string();
  const size_t N = (size_t)num_logged_steps_;

  auto pack_clean = [&](const std::string& key, const std::string& name,
                        const std::vector<size_t>& shape, bool append) {
    auto path = save_path_ / (save_name_ + "_" + name + ".tmp");
    auto data = loadBinary(path);
    if (!data.empty())
      cnpy::npz_save(npz_path, key, data.data(), shape, append ? "a" : "w");
    std::filesystem::remove(path);
  };

  pack_clean("time", "time", {N}, false);
  pack_clean("solve_times", "solve_times", {N, 2}, true);
  pack_clean("x_curr", "states", {N, (size_t)state_dim_}, true);
  pack_clean("u_curr", "inputs", {N, (size_t)input_dim_}, true);

  auto x_p_data = loadBinary(save_path_ / (save_name_ + "_x_pred.tmp"));
  if (!x_p_data.empty()) {
    size_t Tx = x_p_data.size() / (N * state_dim_);
    cnpy::npz_save(npz_path, "x_pred", x_p_data.data(), {N, Tx, (size_t)state_dim_}, "a");
  }
  std::filesystem::remove(save_path_ / (save_name_ + "_x_pred.tmp"));

  auto u_p_data = loadBinary(save_path_ / (save_name_ + "_u_pred.tmp"));
  if (!u_p_data.empty()) {
    size_t Tu = u_p_data.size() / (N * input_dim_);
    cnpy::npz_save(npz_path, "u_pred", u_p_data.data(), {N, Tu, (size_t)input_dim_}, "a");
  }
  std::filesystem::remove(save_path_ / (save_name_ + "_u_pred.tmp"));

  for (const auto& key : metadata_keys_) {
    const auto& entry = metadata_registry_[key];
    std::visit([&](auto&& arg) {
      using T = std::decay_t<decltype(arg)>;
      if constexpr (std::is_same_v<T, int>) {
        int val = arg;
        cnpy::npz_save(npz_path, "meta_" + key, &val, {1}, "a");
      } else if constexpr (std::is_same_v<T, double>) {
        double val = arg;
        cnpy::npz_save(npz_path, "meta_" + key, &val, {1}, "a");
      } else if constexpr (std::is_same_v<T, Eigen::VectorXd>) {
        cnpy::npz_save(npz_path, "meta_" + key, arg.data(), {(size_t)arg.size()}, "a");
      } else if constexpr (std::is_same_v<T, std::vector<double>>) {
        cnpy::npz_save(npz_path, "meta_" + key, arg.data(), {arg.size()}, "a");
      }
    }, entry.value);
  }

  writeParamFile();
  is_finalized_ = true;
}

void MPCLogger::writeParamFile(const std::filesystem::path& filename)
{
  std::ofstream fout(save_path_ / filename);
  fout << std::boolalpha;
  for (const auto& key : metadata_keys_) {
    const auto& entry = metadata_registry_[key];
    fout << key << ": ";
    std::visit([&](auto&& arg) {
      using T = std::decay_t<decltype(arg)>;
      if constexpr (std::is_same_v<T, int> || std::is_same_v<T, double> || std::is_same_v<T, std::string>) {
        if (entry.precision >= 0 && std::is_same_v<T, double>)
          fout << std::fixed << std::setprecision(entry.precision) << arg << "\n";
        else
          fout << arg << "\n";
      } else if constexpr (std::is_same_v<T, Eigen::VectorXd>) {
        fout << vec2Str(arg, entry.precision) << "\n";
      } else if constexpr (std::is_same_v<T, std::vector<double>>) {
        Eigen::VectorXd v = Eigen::Map<const Eigen::VectorXd>(arg.data(), arg.size());
        fout << vec2Str(v, entry.precision) << "\n";
      }
    }, entry.value);
  }
}

} // namespace affine_mpc
