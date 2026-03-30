#include "affine_mpc/mpc_logger.hpp"

#include <cnpy.h>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace affine_mpc {

namespace {
const Eigen::IOFormat kVectorFormat(
    Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", "[", "]");

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

void writeBinary(std::ofstream& fout,
                 const Eigen::Ref<const Eigen::VectorXd>& vec)
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
                     const std::filesystem::path& save_dir,
                     const double ts,
                     const int prediction_stride,
                     const bool log_control_points,
                     const std::string& save_name) :
    mpc_{mpc},
    state_dim_{mpc_->state_dim_},
    input_dim_{mpc_->input_dim_},
    horizon_steps_{mpc_->horizon_steps_},
    num_ctrls_{mpc_->num_ctrl_pts_},
    spline_degree_{mpc_->spline_degree_},
    ts_{ts},
    prediction_stride_{prediction_stride},
    log_control_points_{log_control_points},
    save_dir_{save_dir.lexically_normal()},
    save_name_{save_name},
    is_finalized_{false},
    num_logged_steps_{0},
    x_traj_buf_{state_dim_ * horizon_steps_},
    u_traj_buf_{log_control_points ? input_dim_ * num_ctrls_
                                   : input_dim_ * horizon_steps_}
{
  if (prediction_stride_ <= 0) {
    strided_k_.push_back(0);
  } else {
    for (int k = 0; k <= horizon_steps_; k += prediction_stride_) {
      strided_k_.push_back(k);
    }
    if (strided_k_.back() != horizon_steps_) {
      strided_k_.push_back(horizon_steps_);
    }
  }

  logged_x_dim_ = state_dim_ * strided_k_.size();
  logged_u_dim_ = log_control_points_ ? input_dim_ * num_ctrls_
                                      : input_dim_ * strided_k_.size();

  states_out_buf_.resize(logged_x_dim_);
  inputs_out_buf_.resize(logged_u_dim_);
  ref_states_out_buf_.resize(logged_x_dim_);
  ref_inputs_out_buf_.resize(logged_u_dim_);

  captureMPCSnapshot();

  std::vector<double> t_pred(strided_k_.size());
  for (size_t i = 0; i < strided_k_.size(); ++i) {
    t_pred[i] = strided_k_[i] * ts_;
  }
  addMetadata("t_pred", t_pred);
  addMetadata("ts", ts_);
  addMetadata("prediction_stride", prediction_stride_);
  addMetadata("log_control_points", int{log_control_points_});

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

void MPCLogger::captureMPCSnapshot()
{
  addMetadata("state_dim", mpc_->state_dim_);
  addMetadata("input_dim", mpc_->input_dim_);
  addMetadata("horizon_steps", mpc_->horizon_steps_);
  addMetadata("spline_degree", mpc_->spline_degree_);
  addMetadata("num_control_points", mpc_->num_ctrl_pts_);

  if (mpc_->spline_knots_.size() > 0)
    addMetadata("knots", mpc_->spline_knots_);

  addMetadata("opt_use_input_cost",
              static_cast<int>(mpc_->opts_.use_input_cost));
  addMetadata("opt_slew_initial_input",
              static_cast<int>(mpc_->opts_.slew_initial_input));
  addMetadata("opt_slew_control_points",
              static_cast<int>(mpc_->opts_.slew_control_points));
  addMetadata("opt_saturate_states",
              static_cast<int>(mpc_->opts_.saturate_states));
  addMetadata("opt_saturate_input_trajectory",
              static_cast<int>(mpc_->opts_.saturate_input_trajectory));

  addMetadata("u_min", mpc_->u_min_);
  addMetadata("u_max", mpc_->u_max_);
  addMetadata("Q_diag", mpc_->Q_big_.diagonal().head(state_dim_));
  addMetadata("Qf_diag", mpc_->Q_big_.diagonal().tail(state_dim_));

  if (mpc_->opts_.use_input_cost)
    addMetadata("R_diag", mpc_->R_big_.diagonal().head(input_dim_));
  if (mpc_->opts_.slew_control_points)
    addMetadata("u_slew", mpc_->u_slew_);
  if (mpc_->opts_.saturate_states) {
    addMetadata("x_min", mpc_->x_min_);
    addMetadata("x_max", mpc_->x_max_);
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
                        const Eigen::Ref<const Eigen::VectorXd>& x0,
                        double solve_time)
{
  solve_times_buf_ << solve_time, mpc_->solver_->getSolveTime();

  mpc_->getPredictedStateTrajectory(x_traj_buf_);
  const bool has_input_ref{mpc_->opts_.use_input_cost};

  if (log_control_points_) {
    mpc_->getParameterizedInputTrajectory(inputs_out_buf_);
    if (has_input_ref)
      ref_inputs_out_buf_ = mpc_->u_ref_;
  } else {
    mpc_->getInputTrajectory(u_traj_buf_);
  }

  // Handle Input References Evaluation if not logging control points
  const int order{mpc_->spline_degree_ + 1};

  for (size_t i = 0; i < strided_k_.size(); ++i) {
    const int k{strided_k_[i]};

    // 1. States
    if (k == 0) {
      states_out_buf_.head(state_dim_) = x0;
      // add 1st ref state for x0 to match size of states & ref_states
      ref_states_out_buf_.head(state_dim_) = mpc_->x_ref_.head(state_dim_);
    } else {
      states_out_buf_.segment(i * state_dim_, state_dim_) =
          x_traj_buf_.segment((k - 1) * state_dim_, state_dim_);
      ref_states_out_buf_.segment(i * state_dim_, state_dim_) =
          mpc_->x_ref_.segment((k - 1) * state_dim_, state_dim_);
    }

    // 2. Inputs
    if (!log_control_points_) {
      const int u_idx{std::min(k, horizon_steps_ - 1)};
      inputs_out_buf_.segment(i * input_dim_, input_dim_) =
          u_traj_buf_.segment(u_idx * input_dim_, input_dim_);

      if (has_input_ref) {
        // Evaluate spline for reference input at step u_idx
        const int seg{mpc_->spline_segment_idxs_(u_idx)};
        const Eigen::Map<const Eigen::MatrixXd> ctrls{mpc_->u_ref_.data(),
                                                      input_dim_, num_ctrls_};
        ref_inputs_out_buf_.segment(i * input_dim_, input_dim_).noalias() =
            ctrls.middleCols(seg, order) * mpc_->spline_weights_.col(u_idx);
      }
    }
  }
  writeStep(t);
}

void MPCLogger::finalize()
{
  if (is_finalized_)
    return;

  time_bin_.close();
  solve_times_bin_.close();
  states_bin_.close();
  inputs_bin_.close();
  ref_states_bin_.close();
  if (ref_inputs_bin_.is_open())
    ref_inputs_bin_.close();

  const std::string npz_path = (save_dir_ / (save_name_ + ".npz")).string();
  const size_t N{static_cast<size_t>(num_logged_steps_)};

  auto pack_clean = [&](const std::string& name,
                        const std::vector<size_t>& shape, bool append) {
    auto path = save_dir_ / (save_name_ + "_" + name + ".tmp");
    auto data = loadBinary(path);
    if (!data.empty())
      cnpy::npz_save(npz_path, name, data.data(), shape, append ? "a" : "w");
    std::filesystem::remove(path);
  };

  pack_clean("time", {N}, false);
  pack_clean("solve_times", {N, 2}, true);

  const size_t K{strided_k_.size()};
  const size_t n{static_cast<size_t>(state_dim_)};
  const size_t m{static_cast<size_t>(input_dim_)};
  const size_t nc{static_cast<size_t>(num_ctrls_)};
  if (K == 1) {
    pack_clean("states", {N, n}, true);
    pack_clean("ref_states", {N, n}, true);
  } else {
    pack_clean("states", {N, K, n}, true);
    pack_clean("ref_states", {N, K, n}, true);
  }

  if (log_control_points_) {
    pack_clean("inputs", {N, nc, m}, true);
    pack_clean("ref_inputs", {N, nc, m}, true);
  } else {
    if (K == 1) {
      pack_clean("inputs", {N, m}, true);
      pack_clean("ref_inputs", {N, m}, true);
    } else {
      pack_clean("inputs", {N, K, m}, true);
      pack_clean("ref_inputs", {N, K, m}, true);
    }
  }

  for (const auto& key : metadata_keys_) {
    const auto& entry = metadata_registry_[key];
    std::visit(
        [&](auto&& arg) {
          using T = std::decay_t<decltype(arg)>;
          if constexpr (std::is_same_v<T, int>) {
            int val = arg;
            cnpy::npz_save(npz_path, "meta_" + key, &val, {1}, "a");
          } else if constexpr (std::is_same_v<T, double>) {
            double val = arg;
            cnpy::npz_save(npz_path, "meta_" + key, &val, {1}, "a");
          } else if constexpr (std::is_same_v<T, Eigen::VectorXd>) {
            cnpy::npz_save(npz_path, "meta_" + key, arg.data(),
                           {(size_t)arg.size()}, "a");
          } else if constexpr (std::is_same_v<T, std::vector<double>>) {
            cnpy::npz_save(npz_path, "meta_" + key, arg.data(), {arg.size()},
                           "a");
          }
        },
        entry.value);
  }

  writeParamFile();
  is_finalized_ = true;
}

void MPCLogger::writeParamFile(const std::filesystem::path& filename)
{
  std::ofstream fout(save_dir_ / filename);
  fout << std::boolalpha;
  for (const auto& key : metadata_keys_) {
    const auto& entry = metadata_registry_[key];
    fout << key << ": ";
    std::visit(
        [&](auto&& arg) {
          using T = std::decay_t<decltype(arg)>;
          if constexpr (std::is_same_v<T, int> || std::is_same_v<T, double>
                        || std::is_same_v<T, std::string>) {
            if (entry.precision >= 0 && std::is_same_v<T, double>)
              fout << std::fixed << std::setprecision(entry.precision) << arg
                   << "\n";
            else
              fout << arg << "\n";
          } else if constexpr (std::is_same_v<T, Eigen::VectorXd>) {
            fout << vec2Str(arg, entry.precision) << "\n";
          } else if constexpr (std::is_same_v<T, std::vector<double>>) {
            Eigen::VectorXd v =
                Eigen::Map<const Eigen::VectorXd>(arg.data(), arg.size());
            fout << vec2Str(v, entry.precision) << "\n";
          }
        },
        entry.value);
  }
}

void MPCLogger::initTempFiles()
{
  if (!std::filesystem::exists(save_dir_))
    std::filesystem::create_directories(save_dir_);

  auto open_bin = [&](std::ofstream& fout, const std::string& name) {
    fout.open(save_dir_ / (save_name_ + "_" + name + ".tmp"),
              std::ios::binary | std::ios::out);
  };

  open_bin(time_bin_, "time");
  open_bin(solve_times_bin_, "solve_times");
  open_bin(states_bin_, "states");
  open_bin(inputs_bin_, "inputs");
  open_bin(ref_states_bin_, "ref_states");
  if (mpc_->opts_.use_input_cost)
    open_bin(ref_inputs_bin_, "ref_inputs");
}

void MPCLogger::writeStep(double t)
{
  time_bin_.write(reinterpret_cast<const char*>(&t), sizeof(double));
  writeBinary(solve_times_bin_, solve_times_buf_);
  writeBinary(states_bin_, states_out_buf_);
  writeBinary(inputs_bin_, inputs_out_buf_);
  writeBinary(ref_states_bin_, ref_states_out_buf_);
  if (mpc_->opts_.use_input_cost)
    writeBinary(ref_inputs_bin_, ref_inputs_out_buf_);
  ++num_logged_steps_;
}

} // namespace affine_mpc
