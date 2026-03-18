#include "affine_mpc/mpc_logger.hpp"

#include <cassert>
#include <cnpy.h>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace affine_mpc {

namespace {
// Define a standard format for Eigen vectors to string conversion for YAML
const Eigen::IOFormat kVectorFormat(Eigen::StreamPrecision,
                                    Eigen::DontAlignCols,
                                    ", ",
                                    ", ",
                                    "",
                                    "",
                                    "[",
                                    "]");

std::string eig2Str(const Eigen::Ref<const Eigen::VectorXd>& vec)
{
  if (vec.size() == 0)
    return "[]";

  std::stringstream ss;
  ss << vec.format(kVectorFormat);
  return ss.str();
}

/**
 * @brief Write raw binary data from an Eigen vector to a stream.
 */
void writeBinary(std::ofstream& fout,
                 const Eigen::Ref<const Eigen::VectorXd>& vec)
{
  fout.write(reinterpret_cast<const char*>(vec.data()),
             vec.size() * sizeof(double));
}

/**
 * @brief Helper to load raw binary data into a vector.
 */
std::vector<double> loadBinary(const std::filesystem::path& path)
{
  std::ifstream fin(path, std::ios::binary | std::ios::ate);
  if (!fin.is_open()) {
    return {};
  }
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
    save_path_{save_location.lexically_normal()},
    save_name_{save_name},
    wrote_params_{false},
    is_finalized_{false},
    num_logged_steps_{0},
    x_traj_{mpc->state_dim_ * mpc->horizon_steps_},
    u_traj_{mpc_->input_dim_ * mpc_->horizon_steps_}
{
  if (std::filesystem::exists(save_path_)
      && !std::filesystem::is_directory(save_path_)) {
    throw std::invalid_argument("[MPCLogger Constructor] "
                                "save_location exists but is not a directory: "
                                + save_path_.string());
  }

  if (!std::filesystem::exists(save_path_)) {
    std::filesystem::create_directories(save_path_);
  }

  auto open_bin = [&](std::ofstream& fout, const std::string& name) {
    fout.open(save_path_ / (save_name_ + "_" + name + ".tmp"),
              std::ios::binary | std::ios::out);
    if (!fout.is_open()) {
      throw std::runtime_error("[MPCLogger Constructor] Failed to open temp "
                               + name + " for writing in "
                               + save_path_.string());
    }
  };

  open_bin(time_bin_, "time");
  open_bin(solve_times_bin_, "solve_times");
  open_bin(states_bin_, "states");
  open_bin(refs_bin_, "refs");
  open_bin(inputs_bin_, "inputs");
}

MPCLogger::~MPCLogger()
{
  try {
    if (!is_finalized_) {
      finalize();
    }
  } catch (const std::exception& e) {
    std::cerr << "[MPCLogger Destructor] Error finalizing log: " << e.what()
              << std::endl;
  }
}

void MPCLogger::logPreviousSolve(double t0,
                                 double ts,
                                 const Eigen::Ref<const Eigen::VectorXd>& x0,
                                 const double solve_time,
                                 const int write_every)
{
  if (!mpc_->solver_initialized_) {
    throw std::logic_error("[MPCLogger::logPreviousSolve] "
                           "Solver must be initialized before logging data.");
  }

  const int n{mpc_->state_dim_};
  const int m{mpc_->input_dim_};
  const int T{mpc_->horizon_steps_};

  double time{t0};

  // 1. Log current time and state
  time_bin_.write(reinterpret_cast<const char*>(&time), sizeof(double));
  writeBinary(states_bin_, x0);

  // 2. Log predicted trajectories
  mpc_->getPredictedStateTrajectory(x_traj_);
  mpc_->getInputTrajectory(u_traj_);

  // Note: We log the WHOLE horizon at each step (N-first in Python).
  // This is slightly different from the old text logger which logged
  // points within the horizon as separate rows.
  // This matches the logger_plan.md requirement for (N, Tx, n) shapes.
  writeBinary(states_bin_, x_traj_);
  writeBinary(refs_bin_, mpc_->x_goal_);
  writeBinary(inputs_bin_, u_traj_);

  // Log solve times
  double times[2] = {solve_time, mpc_->solver_->getSolveTime()};
  solve_times_bin_.write(reinterpret_cast<const char*>(times),
                         2 * sizeof(double));

  num_logged_steps_++;
}

void MPCLogger::finalize()
{
  if (is_finalized_)
    return;

  // Close all binary streams
  time_bin_.close();
  solve_times_bin_.close();
  states_bin_.close();
  refs_bin_.close();
  inputs_bin_.close();

  const std::string npz_path = (save_path_ / (save_name_ + ".npz")).string();

  auto pack_and_clean = [&](const std::string& key, const std::string& suffix,
                            const std::vector<size_t>& shape,
                            bool append = true) {
    const std::filesystem::path path =
        save_path_ / (save_name_ + "_" + suffix + ".tmp");
    std::vector<double> data = loadBinary(path);
    if (!data.empty()) {
      if (append)
        cnpy::npz_save(npz_path, key, data.data(), shape, "a");
      else
        cnpy::npz_save(npz_path, key, data.data(), shape, "w");
    }
    std::filesystem::remove(path);
  };

  const size_t N = static_cast<size_t>(num_logged_steps_);
  const size_t n = static_cast<size_t>(mpc_->state_dim_);
  const size_t m = static_cast<size_t>(mpc_->input_dim_);
  const size_t T = static_cast<size_t>(mpc_->horizon_steps_);

  // Save each dataset. First one uses "w" to create/overwrite the file.
  pack_and_clean("time", "time", {N}, false);
  pack_and_clean("solve_times", "solve_times", {N, 2}, true);

  // x_pred: The old text logger logged (T+1) points including x0.
  // Our binary logger wrote x0 (n) + x_traj (T*n) = (T+1)*n doubles per step.
  pack_and_clean("x_pred", "states", {N, T + 1, n}, true);
  pack_and_clean("x_ref", "refs", {N, T, n}, true);
  pack_and_clean("u_pred", "inputs", {N, T, m}, true);

  // Also save spline knots if available
  if (mpc_->spline_knots_.size() > 0) {
    std::vector<double> knots(mpc_->spline_knots_.data(),
                              mpc_->spline_knots_.data()
                                  + mpc_->spline_knots_.size());
    cnpy::npz_save(npz_path, "spline_knots", knots.data(),
                   {static_cast<size_t>(knots.size())}, "a");
  }

  if (!wrote_params_) {
    writeParamFile();
  }

  is_finalized_ = true;
}

void MPCLogger::writeParamFile(const std::filesystem::path& filename)
{
  std::ofstream param_fout(save_path_ / filename);
  if (!param_fout.is_open()) {
    throw std::runtime_error("[MPCLogger::writeParamFile] Failed to open "
                             + filename.string() + " for writing in "
                             + save_path_.string());
  }

  param_fout << std::boolalpha;
  param_fout << "n: " << mpc_->state_dim_ << "\n"
             << "m: " << mpc_->input_dim_ << "\n"
             << "T: " << mpc_->horizon_steps_ << "\n"
             << "mu: " << mpc_->num_ctrl_pts_ << "\n"
             << "p: " << mpc_->spline_degree_ << "\n"
             << "use_input_cost: " << mpc_->opts_.use_input_cost << "\n"
             << "slew_initial_input: " << mpc_->opts_.slew_initial_input << "\n"
             << "slew_control_points: " << mpc_->opts_.slew_control_points
             << "\n"
             << "saturate_states: " << mpc_->opts_.saturate_states << "\n"
             << "u_min: " << eig2Str(mpc_->u_min_) << "\n"
             << "u_max: " << eig2Str(mpc_->u_max_) << "\n"
             << "Q: " << eig2Str(mpc_->Q_big_.diagonal().head(mpc_->state_dim_))
             << "\n"
             << "Qf: " << eig2Str(mpc_->Q_big_.diagonal().tail(mpc_->state_dim_))
             << "\n";

  param_fout << "R: ";
  if (mpc_->opts_.use_input_cost) {
    param_fout << eig2Str(mpc_->R_big_.diagonal().head(mpc_->input_dim_))
               << "\n";
  } else {
    param_fout << "None\n";
  }

  auto log_opt_vec = [&](const std::string& label, bool condition,
                         const Eigen::VectorXd& vec) {
    param_fout << label << ": ";
    if (condition) {
      param_fout << eig2Str(vec) << "\n";
    } else {
      param_fout << "None\n";
    }
  };

  log_opt_vec("u_slew", mpc_->opts_.slew_control_points, mpc_->u_slew_);
  log_opt_vec("x_min", mpc_->opts_.saturate_states, mpc_->x_min_);
  log_opt_vec("x_max", mpc_->opts_.saturate_states, mpc_->x_max_);

  wrote_params_ = true;
}

} // namespace affine_mpc
