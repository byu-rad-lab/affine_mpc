#include "affine_mpc/mpc_logger.hpp"

#include <Eigen/Core>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <type_traits>

#include "npy_writer.hpp"
#include "npz_writer.hpp"

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
  if (vec.size() > 0) {
    fout.write(reinterpret_cast<const char*>(vec.data()),
               static_cast<std::streamsize>(vec.size() * sizeof(double)));
  }
}

std::string loggingModeToString(affine_mpc::MPCLogger::Mode mode)
{
  switch (mode) {
  case MPCLogger::Mode::RawRecoverable:
    return "RawRecoverable";
  case MPCLogger::Mode::Npy:
    return "Npy";
  case MPCLogger::Mode::NpzUncompressed:
    return "NpzUncompressed";
  case MPCLogger::Mode::NpzCompressed:
    return "NpzCompressed";
  }
  return "Unknown";
}

std::string makeShapeString(const std::vector<size_t>& shape)
{
  std::ostringstream oss;
  oss << "[";
  for (size_t i{0}; i < shape.size(); ++i) {
    if (i > 0)
      oss << ", ";
    oss << shape[i];
  }
  oss << "]";
  return oss.str();
}

} // namespace

MPCLogger::MPCLogger(const MPCBase* const mpc,
                     const std::filesystem::path& save_dir,
                     double ts,
                     int prediction_stride,
                     bool log_control_points,
                     const std::string& save_name,
                     MPCLogger::Mode mode) :
    mpc_{mpc},
    state_dim_{mpc_->getStateDim()},
    input_dim_{mpc_->getInputDim()},
    horizon_steps_{mpc_->getHorizonSteps()},
    num_ctrls_{mpc_->getNumControlPoints()},
    spline_degree_{mpc_->getParameterization().degree},
    input_param_{input_dim_, mpc_->getParameterization()},
    ts_{ts},
    prediction_stride_{prediction_stride},
    log_control_points_{log_control_points},
    logging_mode_{mode},
    save_dir_{save_dir.lexically_normal()},
    save_name_{save_name},
    is_finalized_{false},
    num_logged_steps_{0},
    loop_time_{-1.0},
    mpc_solve_time_{0.0},
    qp_solve_time_{0.0},
    prepare_time_{0.0},
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
    if (strided_k_.back() != horizon_steps_)
      strided_k_.push_back(horizon_steps_);
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
  for (size_t i = 0; i < strided_k_.size(); ++i)
    t_pred[i] = strided_k_[i] * ts_;

  addMetadata("t_pred", t_pred);
  addMetadata("ts", ts_);
  addMetadata("prediction_stride", prediction_stride_);
  addMetadata("log_control_points", int{log_control_points_});
  addMetadata("logging_mode", loggingModeToString(logging_mode_));

  initRawFiles();
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
  addMetadata("state_dim", mpc_->getStateDim());
  addMetadata("input_dim", mpc_->getInputDim());
  addMetadata("horizon_steps", mpc_->getHorizonSteps());
  const Parameterization& param{mpc_->getParameterization()};
  const bool bspline{param.type == Parameterization::Type::BSpline};
  addMetadata("parameterization_type",
              std::string{bspline ? "bspline" : "custom"});
  if (bspline)
    addMetadata("spline_degree", param.degree);
  addMetadata("num_control_points", mpc_->getNumControlPoints());

  if (bspline) {
    addMetadata("knots", param.knots);
  } else {
    // the basis row by row (horizon_steps x num_control_points), so control
    // points can be evaluated offline
    addMetadata("basis",
                Eigen::VectorXd{Eigen::Map<const Eigen::VectorXd>{
                    param.band_weights.data(), param.band_weights.size()}});
  }

  addMetadata("opt_use_input_cost",
              static_cast<int>(mpc_->getOptions().use_input_cost));
  addMetadata("opt_slew_initial_input",
              static_cast<int>(mpc_->getOptions().slew_initial_input));
  addMetadata("opt_slew_control_points",
              static_cast<int>(mpc_->getOptions().slew_control_points));
  addMetadata("opt_saturate_states",
              static_cast<int>(mpc_->getOptions().saturate_states));
  addMetadata("opt_saturate_input_trajectory",
              static_cast<int>(mpc_->getOptions().saturate_input_trajectory));
  addMetadata("opt_use_input_rate_cost",
              static_cast<int>(mpc_->getOptions().use_input_rate_cost));
  addMetadata("opt_soft_state_bounds",
              static_cast<int>(mpc_->getOptions().soft_state_bounds));
  addMetadata("opt_num_general_constraints",
              mpc_->getOptions().num_general_constraints);
  addMetadata("opt_num_terminal_constraints",
              mpc_->getOptions().num_terminal_constraints);

  addMetadata("u_min", mpc_->getInputMin());
  addMetadata("u_max", mpc_->getInputMax());
  addMetadata("Q_diag", mpc_->getStateWeights());
  addMetadata("Qf_diag", mpc_->getTerminalStateWeights());

  if (mpc_->getOptions().use_input_cost)
    addMetadata("R_diag", mpc_->getInputWeights());
  if (mpc_->getOptions().use_input_rate_cost)
    addMetadata("R_rate_diag", mpc_->getInputRateWeights());
  if (mpc_->getOptions().slew_control_points)
    addMetadata("control_point_slew", mpc_->getSlewRate());
  if (mpc_->getOptions().saturate_states) {
    // x_min and x_max have one entry per bounded state
    const std::vector<int>& bounded{mpc_->getBoundedStates()};
    addMetadata("bounded_states",
                std::vector<double>(bounded.begin(), bounded.end()));
    addMetadata("x_min", mpc_->getStateMin());
    addMetadata("x_max", mpc_->getStateMax());
  }
  if (mpc_->getOptions().soft_state_bounds) {
    addMetadata("x_soft_l1", mpc_->getStateBoundL1Penalties());
    addMetadata("x_soft_l2", mpc_->getStateBoundL2Penalties());
  }
  // constraint matrices are stored row-major (reshape to rows x state_dim or
  // rows x input_dim)
  auto rowMajor = [](const Eigen::MatrixXd& mat) {
    std::vector<double> values;
    values.reserve(static_cast<size_t>(mat.size()));
    for (Eigen::Index i{0}; i < mat.rows(); ++i)
      for (Eigen::Index j{0}; j < mat.cols(); ++j)
        values.push_back(mat(i, j));
    return values;
  };
  if (mpc_->getOptions().num_general_constraints > 0) {
    addMetadata("Hx", rowMajor(mpc_->getGeneralConstraintStateMatrix()));
    addMetadata("Hu", rowMajor(mpc_->getGeneralConstraintInputMatrix()));
    addMetadata("h_min", mpc_->getGeneralConstraintMin());
    addMetadata("h_max", mpc_->getGeneralConstraintMax());
  }
  if (mpc_->getOptions().num_terminal_constraints > 0) {
    addMetadata("Hf", rowMajor(mpc_->getTerminalConstraintMatrix()));
    addMetadata("hf_min", mpc_->getTerminalConstraintMin());
    addMetadata("hf_max", mpc_->getTerminalConstraintMax());
  }
}

void MPCLogger::addMetadata(const std::string& key,
                            const MetadataValue& value,
                            int precision)
{
  if (metadata_registry_.find(key) == metadata_registry_.end())
    metadata_keys_.push_back(key);
  metadata_registry_[key] = {value, precision};
}

void MPCLogger::logStep(double t,
                        const Eigen::Ref<const Eigen::VectorXd>& x0,
                        double loop_time)
{
  if (!mpc_->isSolverInitialized())
    throw std::logic_error("[MPCLogger::logStep] MPC solver must be "
                           "initialized before logging.");
  if (x0.size() != state_dim_)
    throw std::invalid_argument("[MPCLogger::logStep] x0 must have size "
                                + std::to_string(state_dim_) + ", got "
                                + std::to_string(x0.size()) + ".");
  const SolveInfo info{mpc_->getSolveInfo()};
  loop_time_ = loop_time;
  mpc_solve_time_ = info.mpc_solve_time;
  qp_solve_time_ = info.qp_solve_time;
  prepare_time_ = info.prepare_time;

  mpc_->getPredictedStateTrajectory(x_traj_buf_);
  const bool has_input_ref{mpc_->getOptions().use_input_cost};

  if (log_control_points_) {
    mpc_->getInputControlPoints(inputs_out_buf_);
    if (has_input_ref)
      ref_inputs_out_buf_ = mpc_->getReferenceInputControlPoints();
  } else {
    mpc_->getInputTrajectory(u_traj_buf_);
  }

  for (size_t i = 0; i < strided_k_.size(); ++i) {
    const int k{strided_k_[i]};
    const auto i_idx{static_cast<Eigen::Index>(i)};

    if (k == 0) {
      states_out_buf_.head(state_dim_) = x0;
      ref_states_out_buf_.head(state_dim_) =
          mpc_->getReferenceStateTrajectory().head(state_dim_);
    } else {
      states_out_buf_.segment(i_idx * state_dim_, state_dim_) =
          x_traj_buf_.segment((k - 1) * state_dim_, state_dim_);
      ref_states_out_buf_.segment(i_idx * state_dim_, state_dim_) =
          mpc_->getReferenceStateTrajectory().segment((k - 1) * state_dim_,
                                                      state_dim_);
    }

    if (!log_control_points_) {
      const Eigen::Index u_idx{std::min(k, horizon_steps_ - 1)};
      inputs_out_buf_.segment(i_idx * input_dim_, input_dim_) =
          u_traj_buf_.segment(u_idx * input_dim_, input_dim_);

      if (has_input_ref) {
        input_param_.evaluate(
            u_idx, mpc_->getReferenceInputControlPoints(),
            ref_inputs_out_buf_.segment(i_idx * input_dim_, input_dim_));
      }
    }
  }

  writeStep(t);
}

void MPCLogger::finalize()
{
  if (is_finalized_)
    return;

  closeRawFiles();
  const std::vector<PayloadEntry> payloads = buildPayloadEntries();

  switch (logging_mode_) {
  case MPCLogger::Mode::RawRecoverable:
    finalizeRawRecoverable(payloads);
    return;
  case MPCLogger::Mode::Npy:
    finalizeNpy(payloads);
    return;
  case MPCLogger::Mode::NpzUncompressed:
    finalizeNpz(payloads, false);
    return;
  case MPCLogger::Mode::NpzCompressed:
    finalizeNpz(payloads, true);
    return;
  }
}

void MPCLogger::initRawFiles()
{
  std::filesystem::create_directories(save_dir_);

  const std::filesystem::path raw_dir = rawDirPath();
  if (std::filesystem::exists(raw_dir))
    std::filesystem::remove_all(raw_dir);
  std::filesystem::create_directories(raw_dir);

  auto open_bin = [&](std::ofstream& fout, const std::string& name) {
    fout.open(rawDataPath(name), std::ios::binary | std::ios::out);
    if (!fout.is_open()) {
      throw std::runtime_error("[MPCLogger] Failed to open raw payload file: "
                               + rawDataPath(name).string());
    }
  };

  open_bin(time_bin_, "time");
  open_bin(loop_time_bin_, "loop_time");
  open_bin(mpc_solve_time_bin_, "mpc_solve_time");
  open_bin(qp_solve_time_bin_, "qp_solve_time");
  open_bin(prepare_time_bin_, "prepare_time");
  open_bin(states_bin_, "states");
  open_bin(inputs_bin_, "inputs");
  open_bin(ref_states_bin_, "ref_states");
  if (mpc_->getOptions().use_input_cost)
    open_bin(ref_inputs_bin_, "ref_inputs");
}

void MPCLogger::closeRawFiles()
{
  time_bin_.close();
  loop_time_bin_.close();
  mpc_solve_time_bin_.close();
  qp_solve_time_bin_.close();
  prepare_time_bin_.close();
  states_bin_.close();
  inputs_bin_.close();
  ref_states_bin_.close();
  if (ref_inputs_bin_.is_open())
    ref_inputs_bin_.close();
}

std::vector<MPCLogger::PayloadEntry> MPCLogger::buildPayloadEntries() const
{
  const size_t N{num_logged_steps_};
  const size_t K{strided_k_.size()};
  const size_t n{static_cast<size_t>(state_dim_)};
  const size_t m{static_cast<size_t>(input_dim_)};
  const size_t nc{static_cast<size_t>(num_ctrls_)};

  std::vector<PayloadEntry> payloads;
  payloads.push_back({"time", rawDataPath("time"), rawHeaderPath("time"), {N}});
  for (const char* name :
       {"loop_time", "mpc_solve_time", "qp_solve_time", "prepare_time"})
    payloads.push_back({name, rawDataPath(name), rawHeaderPath(name), {N}});

  if (K == 1) {
    payloads.push_back(
        {"states", rawDataPath("states"), rawHeaderPath("states"), {N, n}});
    payloads.push_back({"ref_states",
                        rawDataPath("ref_states"),
                        rawHeaderPath("ref_states"),
                        {N, n}});
  } else {
    payloads.push_back(
        {"states", rawDataPath("states"), rawHeaderPath("states"), {N, K, n}});
    payloads.push_back({"ref_states",
                        rawDataPath("ref_states"),
                        rawHeaderPath("ref_states"),
                        {N, K, n}});
  }

  if (log_control_points_) {
    payloads.push_back(
        {"inputs", rawDataPath("inputs"), rawHeaderPath("inputs"), {N, nc, m}});
    if (mpc_->getOptions().use_input_cost) {
      payloads.push_back({"ref_inputs",
                          rawDataPath("ref_inputs"),
                          rawHeaderPath("ref_inputs"),
                          {N, nc, m}});
    }
  } else if (K == 1) {
    payloads.push_back(
        {"inputs", rawDataPath("inputs"), rawHeaderPath("inputs"), {N, m}});
    if (mpc_->getOptions().use_input_cost) {
      payloads.push_back({"ref_inputs",
                          rawDataPath("ref_inputs"),
                          rawHeaderPath("ref_inputs"),
                          {N, m}});
    }
  } else {
    payloads.push_back(
        {"inputs", rawDataPath("inputs"), rawHeaderPath("inputs"), {N, K, m}});
    if (mpc_->getOptions().use_input_cost) {
      payloads.push_back({"ref_inputs",
                          rawDataPath("ref_inputs"),
                          rawHeaderPath("ref_inputs"),
                          {N, K, m}});
    }
  }

  return payloads;
}

void MPCLogger::cleanupRawDirectory() const
{
  const std::filesystem::path raw_dir = rawDirPath();
  if (std::filesystem::exists(raw_dir))
    std::filesystem::remove_all(raw_dir);
}

void MPCLogger::finalizeRawRecoverable(
    const std::vector<PayloadEntry>& payloads)
{
  const std::filesystem::path raw_dir = rawDirPath();
  for (const auto& payload : payloads)
    NpyWriter::writeDoubleHeader(payload.header_path, payload.shape);

  writeParamFileToPath(raw_dir / "params.yaml");
  writeDataInfoFile(payloads, raw_dir / "data_info.yaml");
  is_finalized_ = true;
}

void MPCLogger::finalizeNpy(const std::vector<PayloadEntry>& payloads)
{
  const std::filesystem::path npy_dir = npyDirPath();
  try {
    writeNpyOutputs(payloads, npy_dir);
    writeParamFileToPath(save_dir_ / "params.yaml");
    cleanupRawDirectory();
    is_finalized_ = true;
  } catch (...) {
    if (std::filesystem::exists(npy_dir))
      std::filesystem::remove_all(npy_dir);
    throw;
  }
}

void MPCLogger::finalizeNpz(const std::vector<PayloadEntry>& payloads,
                            bool compress)
{
  const std::filesystem::path archive_path = npzPath();
  const std::filesystem::path npy_dir = npyDirPath();

  try {
    writeNpzFromPayloads(payloads, archive_path, compress);
    writeParamFileToPath(save_dir_ / "params.yaml");
    cleanupRawDirectory();
    is_finalized_ = true;
    return;
  } catch (const std::overflow_error& exc) {
    if (std::filesystem::exists(archive_path))
      std::filesystem::remove(archive_path);

    try {
      writeNpyOutputs(payloads, npy_dir);
      writeParamFileToPath(save_dir_ / "params.yaml");
      cleanupRawDirectory();
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
        + " and removed staged raw payload files.");
  } catch (...) {
    if (std::filesystem::exists(archive_path))
      std::filesystem::remove(archive_path);
    throw;
  }
}

void MPCLogger::writeNpzFromPayloads(const std::vector<PayloadEntry>& payloads,
                                     const std::filesystem::path& npz_path,
                                     bool compress) const
{
  NpzWriter writer(npz_path, compress ? NpzWriter::CompressionMode::Deflated
                                      : NpzWriter::CompressionMode::Stored);
  for (const auto& payload : payloads)
    writer.addDoubleArrayFromFile(payload.name, payload.data_path,
                                  payload.shape);

  for (const auto& key : metadata_keys_) {
    const auto& entry = metadata_registry_.at(key);
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
}

void MPCLogger::writeNpyOutputs(const std::vector<PayloadEntry>& payloads,
                                const std::filesystem::path& npy_dir) const
{
  if (std::filesystem::exists(npy_dir))
    std::filesystem::remove_all(npy_dir);
  std::filesystem::create_directories(npy_dir);

  for (const auto& payload : payloads) {
    NpyWriter::writeDoubleArrayFromFile(npy_dir / (payload.name + ".npy"),
                                        payload.data_path, payload.shape);
  }
}

void MPCLogger::writeDataInfoFile(
    const std::vector<PayloadEntry>& payloads,
    const std::filesystem::path& data_info_path) const
{
  const std::filesystem::path tmp_path = data_info_path.string() + ".tmp";
  std::ofstream fout{tmp_path};
  if (!fout.is_open()) {
    throw std::runtime_error("[MPCLogger] Failed to open data info file: "
                             + tmp_path.string());
  }

  fout << "format_version: 1\n";
  fout << "mode: " << loggingModeToString(logging_mode_) << "\n";
  fout << "save_name: " << save_name_ << "\n";
  fout << "num_logged_steps: " << num_logged_steps_ << "\n";
  fout << "payloads:\n";
  for (const auto& payload : payloads) {
    fout << "  - name: " << payload.name << "\n";
    fout << "    data_file: " << payload.data_path.filename().string() << "\n";
    fout << "    header_file: " << payload.header_path.filename().string()
         << "\n";
    fout << "    dtype: float64\n";
    fout << "    shape: " << makeShapeString(payload.shape) << "\n";
    fout << "    endianness: little\n";
    fout << "    fortran_order: false\n";
  }

  fout.close();
  if (!fout) {
    throw std::runtime_error("[MPCLogger] Failed while writing data info file: "
                             + tmp_path.string());
  }

  if (std::filesystem::exists(data_info_path))
    std::filesystem::remove(data_info_path);
  std::filesystem::rename(tmp_path, data_info_path);
}

void MPCLogger::writeParamFile(const std::filesystem::path& filename)
{
  writeParamFileToPath(save_dir_ / filename);
}

void MPCLogger::writeParamFileToPath(const std::filesystem::path& path) const
{
  if (path.has_parent_path())
    std::filesystem::create_directories(path.parent_path());

  std::ofstream fout{path};
  fout << std::boolalpha;
  for (const auto& key : metadata_keys_) {
    const auto& entry = metadata_registry_.at(key);
    fout << key << ": ";
    std::visit(
        [&](auto&& arg) {
          using T = std::decay_t<decltype(arg)>;
          if constexpr (std::is_same_v<T, int> || std::is_same_v<T, double>
                        || std::is_same_v<T, std::string>) {
            if (entry.precision >= 0 && std::is_same_v<T, double>) {
              fout << std::fixed << std::setprecision(entry.precision) << arg
                   << "\n";
            } else {
              fout << arg << "\n";
            }
          } else if constexpr (std::is_same_v<T, Eigen::VectorXd>) {
            fout << vec2Str(arg, entry.precision) << "\n";
          } else if constexpr (std::is_same_v<T, std::vector<double>>) {
            const Eigen::VectorXd v =
                Eigen::Map<const Eigen::VectorXd>(arg.data(), arg.size());
            fout << vec2Str(v, entry.precision) << "\n";
          }
        },
        entry.value);
  }
}

void MPCLogger::writeStep(double t)
{
  time_bin_.write(reinterpret_cast<const char*>(&t), sizeof(double));
  loop_time_bin_.write(reinterpret_cast<const char*>(&loop_time_),
                       sizeof(double));
  mpc_solve_time_bin_.write(reinterpret_cast<const char*>(&mpc_solve_time_),
                            sizeof(double));
  qp_solve_time_bin_.write(reinterpret_cast<const char*>(&qp_solve_time_),
                           sizeof(double));
  prepare_time_bin_.write(reinterpret_cast<const char*>(&prepare_time_),
                          sizeof(double));
  writeBinary(states_bin_, states_out_buf_);
  writeBinary(inputs_bin_, inputs_out_buf_);
  writeBinary(ref_states_bin_, ref_states_out_buf_);
  if (mpc_->getOptions().use_input_cost)
    writeBinary(ref_inputs_bin_, ref_inputs_out_buf_);
  ++num_logged_steps_;
}

std::filesystem::path MPCLogger::rawDirPath() const
{
  return save_dir_ / (save_name_ + "_raw");
}

std::filesystem::path MPCLogger::rawDataPath(const std::string& name) const
{
  return rawDirPath() / (name + ".bin");
}

std::filesystem::path MPCLogger::rawHeaderPath(const std::string& name) const
{
  return rawDirPath() / (name + ".npyh");
}

std::filesystem::path MPCLogger::npyDirPath() const
{
  return save_dir_ / (save_name_ + "_npy");
}

std::filesystem::path MPCLogger::npzPath() const
{
  return save_dir_ / (save_name_ + ".npz");
}

} // namespace affine_mpc
