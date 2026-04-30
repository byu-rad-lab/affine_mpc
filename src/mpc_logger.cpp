#include "affine_mpc/mpc_logger.hpp"

#include <cstdint>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <type_traits>

#include "npz_writer.hpp"

namespace affine_mpc {

namespace { // utilities for this file

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

std::string makeShapeString(const std::vector<size_t>& shape)
{
  std::ostringstream oss;
  oss << "(";
  for (size_t i{0}; i < shape.size(); ++i) {
    if (i > 0)
      oss << ", ";
    oss << shape[i];
  }
  if (shape.size() == 1)
    oss << ",";
  oss << ")";
  return oss.str();
}

void writeNpyHeaderV2(std::ofstream& fout, const std::vector<size_t>& shape)
{
  std::ostringstream header;
  header << "{'descr': '<f8', 'fortran_order': False, 'shape': "
         << makeShapeString(shape) << ", }";
  std::string header_str = header.str();

  constexpr size_t preamble_size{12};
  const size_t padding =
      (16 - ((preamble_size + header_str.size() + 1) % 16)) % 16;
  header_str.append(padding, ' ');
  header_str.push_back('\n');

  if (header_str.size() > std::numeric_limits<std::uint32_t>::max()) {
    throw std::length_error(
        "[MPCLogger] NPY header too large for v2.0 format.");
  }

  fout.write("\x93NUMPY", 6);
  const std::uint8_t version[2]{2, 0};
  fout.write(reinterpret_cast<const char*>(version), 2);

  const std::uint32_t header_len =
      static_cast<std::uint32_t>(header_str.size());
  const std::uint8_t header_len_le[4]{
      static_cast<std::uint8_t>(header_len & 0xffu),
      static_cast<std::uint8_t>((header_len >> 8) & 0xffu),
      static_cast<std::uint8_t>((header_len >> 16) & 0xffu),
      static_cast<std::uint8_t>((header_len >> 24) & 0xffu)};
  fout.write(reinterpret_cast<const char*>(header_len_le), 4);
  fout.write(header_str.data(),
             static_cast<std::streamsize>(header_str.size()));
}

void writeTempBinaryAsNpy(const std::filesystem::path& temp_path,
                          const std::filesystem::path& npy_path,
                          const std::vector<size_t>& shape)
{
  std::ifstream fin(temp_path, std::ios::binary | std::ios::ate);
  if (!fin.is_open()) {
    throw std::runtime_error("[MPCLogger] Failed to open temp payload file: "
                             + temp_path.string());
  }

  const auto size = fin.tellg();
  if (size < std::streamoff{0}) {
    throw std::runtime_error("[MPCLogger] Failed to query temp payload size: "
                             + temp_path.string());
  }

  size_t expected_numel{1};
  for (const size_t dim : shape) {
    if (dim > 0 && expected_numel > std::numeric_limits<size_t>::max() / dim) {
      throw std::overflow_error("[MPCLogger] NPY fallback shape overflow.");
    }
    expected_numel *= dim;
  }

  const auto expected_size =
      static_cast<std::uintmax_t>(expected_numel) * sizeof(double);
  if (static_cast<std::uintmax_t>(size) != expected_size) {
    throw std::runtime_error("[MPCLogger] Temp payload size does not match the "
                             "expected NPY array shape for: "
                             + temp_path.string());
  }

  fin.seekg(0, std::ios::beg);
  std::ofstream fout(npy_path, std::ios::binary | std::ios::out);
  if (!fout.is_open()) {
    throw std::runtime_error("[MPCLogger] Failed to open fallback NPY file: "
                             + npy_path.string());
  }

  writeNpyHeaderV2(fout, shape);

  std::array<char, 1 << 16> buffer{};
  while (fin) {
    fin.read(buffer.data(), static_cast<std::streamsize>(buffer.size()));
    const auto count = fin.gcount();
    if (count > 0)
      fout.write(buffer.data(), count);
  }

  if (!fout) {
    throw std::runtime_error(
        "[MPCLogger] Failed while writing fallback NPY file: "
        + npy_path.string());
  }
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
    addMetadata("control_point_slew", mpc_->ctrls_slew_);
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
    mpc_->getInputControlPoints(inputs_out_buf_);
    if (has_input_ref)
      ref_inputs_out_buf_ = mpc_->ctrls_ref_;
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
      // repeats final input
      const int u_idx{std::min(k, horizon_steps_ - 1)};
      inputs_out_buf_.segment(i * input_dim_, input_dim_) =
          u_traj_buf_.segment(u_idx * input_dim_, input_dim_);

      if (has_input_ref) {
        // Evaluate spline for reference input at step u_idx
        mpc_->getInput(u_idx, mpc_->ctrls_ref_,
                       ref_inputs_out_buf_.segment(i * input_dim_, input_dim_));
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

  struct PayloadEntry
  {
    std::string name;
    std::filesystem::path temp_path;
    std::vector<size_t> shape;
  };

  const std::filesystem::path npz_path = save_dir_ / (save_name_ + ".npz");
  const std::filesystem::path npy_dir = save_dir_ / (save_name_ + "_npy");
  const size_t N{static_cast<size_t>(num_logged_steps_)};
  const size_t K{strided_k_.size()};
  const size_t n{static_cast<size_t>(state_dim_)};
  const size_t m{static_cast<size_t>(input_dim_)};
  const size_t nc{static_cast<size_t>(num_ctrls_)};

  std::vector<PayloadEntry> payloads;
  payloads.push_back({"time", save_dir_ / (save_name_ + "_time.tmp"), {N}});
  payloads.push_back(
      {"solve_times", save_dir_ / (save_name_ + "_solve_times.tmp"), {N, 2}});

  if (K == 1) {
    payloads.push_back(
        {"states", save_dir_ / (save_name_ + "_states.tmp"), {N, n}});
    payloads.push_back(
        {"ref_states", save_dir_ / (save_name_ + "_ref_states.tmp"), {N, n}});
  } else {
    payloads.push_back(
        {"states", save_dir_ / (save_name_ + "_states.tmp"), {N, K, n}});
    payloads.push_back({"ref_states",
                        save_dir_ / (save_name_ + "_ref_states.tmp"),
                        {N, K, n}});
  }

  if (log_control_points_) {
    payloads.push_back(
        {"inputs", save_dir_ / (save_name_ + "_inputs.tmp"), {N, nc, m}});
    if (mpc_->opts_.use_input_cost) {
      payloads.push_back({"ref_inputs",
                          save_dir_ / (save_name_ + "_ref_inputs.tmp"),
                          {N, nc, m}});
    }
  } else if (K == 1) {
    payloads.push_back(
        {"inputs", save_dir_ / (save_name_ + "_inputs.tmp"), {N, m}});
    if (mpc_->opts_.use_input_cost) {
      payloads.push_back(
          {"ref_inputs", save_dir_ / (save_name_ + "_ref_inputs.tmp"), {N, m}});
    }
  } else {
    payloads.push_back(
        {"inputs", save_dir_ / (save_name_ + "_inputs.tmp"), {N, K, m}});
    if (mpc_->opts_.use_input_cost) {
      payloads.push_back({"ref_inputs",
                          save_dir_ / (save_name_ + "_ref_inputs.tmp"),
                          {N, K, m}});
    }
  }

  const auto cleanupTemps = [&]() {
    for (const auto& payload : payloads) {
      if (std::filesystem::exists(payload.temp_path))
        std::filesystem::remove(payload.temp_path);
    }
  };

  const auto writeFallbackNpy = [&]() {
    if (std::filesystem::exists(npy_dir))
      std::filesystem::remove_all(npy_dir);
    std::filesystem::create_directories(npy_dir);

    for (const auto& payload : payloads) {
      writeTempBinaryAsNpy(payload.temp_path, npy_dir / (payload.name + ".npy"),
                           payload.shape);
    }
  };

  try {
    NpzWriter writer(npz_path);
    for (const auto& payload : payloads) {
      auto data = loadBinary(payload.temp_path);
      if (!data.empty())
        writer.addArray(payload.name, data.data(), payload.shape);
    }

    for (const auto& key : metadata_keys_) {
      const auto& entry = metadata_registry_[key];
      std::visit(
          [&](auto&& arg) {
            using T = std::decay_t<decltype(arg)>;
            if constexpr (std::is_same_v<T, int>) {
              writer.addScalar("meta_" + key, static_cast<std::int32_t>(arg));
            } else if constexpr (std::is_same_v<T, double>) {
              writer.addScalar("meta_" + key, arg);
            } else if constexpr (std::is_same_v<T, Eigen::VectorXd>) {
              writer.addArray("meta_" + key, arg.data(),
                              {static_cast<size_t>(arg.size())});
            } else if constexpr (std::is_same_v<T, std::vector<double>>) {
              writer.addArray("meta_" + key, arg.data(), {arg.size()});
            }
          },
          entry.value);
    }

    writer.finalize();
    writeParamFile();
    cleanupTemps();
    is_finalized_ = true;
    return;
  } catch (const std::overflow_error& exc) {
    if (std::filesystem::exists(npz_path))
      std::filesystem::remove(npz_path);

    try {
      writeFallbackNpy();
      writeParamFile();
      cleanupTemps();
      is_finalized_ = true;
    } catch (...) {
      if (std::filesystem::exists(npy_dir))
        std::filesystem::remove_all(npy_dir);
      throw;
    }

    throw std::runtime_error(
        std::string{
            "[MPCLogger::finalize] NPZ packaging exceeded ZIP32 limits: "}
        + exc.what() + ". Wrote fallback NPY files to " + npy_dir.string()
        + " and removed temporary payload files.");
  } catch (...) {
    if (std::filesystem::exists(npz_path))
      std::filesystem::remove(npz_path);
    throw;
  }
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
