#include "affine_mpc/mpc_logger.hpp"

#include <Eigen/Core>
#include <cassert>
#include <iostream>
#include <sstream>
#include <stdexcept>

namespace affine_mpc {


std::string eig2Str(const Eigen::Ref<const Eigen::VectorXd>& vec)
{
  // Define a standard format for Eigen vectors to string conversion
  const static Eigen::IOFormat kVectorFormat{Eigen::StreamPrecision,
                                             Eigen::DontAlignCols,
                                             ", ",
                                             ", ",
                                             "",
                                             "",
                                             "[",
                                             "]"};

  if (vec.size() == 0)
    return "[]";

  std::stringstream ss;
  ss << vec.format(kVectorFormat);
  return ss.str();
}

MPCLogger::MPCLogger(const MPCBase* const mpc,
                     const std::filesystem::path& save_location) :
    mpc_{mpc},
    save_path_{save_location.lexically_normal()},
    wrote_params_{false},
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

  auto open_file = [&](std::ofstream& fout, const std::string& name) {
    fout.open(save_path_ / name);
    if (!fout.is_open()) {
      throw std::runtime_error("[MPCLogger Constructor] Failed to open " + name
                               + " for writing in " + save_path_.string());
    }
  };

  open_file(time_fout_, "time.txt");
  open_file(solve_time_fout_, "solve_times.txt");
  open_file(states_fout_, "states.txt");
  open_file(refs_fout_, "ref_states.txt");
  open_file(inputs_fout_, "inputs.txt");
  open_file(spline_knots_fout_, "spline_knots.txt");

  spline_knots_fout_ << mpc_->spline_knots_.transpose() << "\n";
  spline_knots_fout_.close();
}

MPCLogger::~MPCLogger()
{
  try {
    if (!wrote_params_) {
      writeParamFile();
    }
  } catch (const std::exception& e) {
    std::cerr << "[MPCLogger Destructor] Error writing parameter file: "
              << e.what() << std::endl;
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

  time_fout_ << time << " ";
  states_fout_ << x0.transpose() << " ";

  mpc_->getPredictedStateTrajectory(x_traj_);
  mpc_->getInputTrajectory(u_traj_);

  for (int k{0}; k < T - 1; ++k) {
    time += ts;
    if (k % write_every == 0) {
      time_fout_ << time << " ";
      states_fout_ << x_traj_.segment(k * n, n).transpose() << " ";
      refs_fout_ << mpc_->x_goal_.segment(k * n, n).transpose() << " ";
      inputs_fout_ << u_traj_.segment(k * m, m).transpose() << " ";
    }
  }

  time += ts;
  time_fout_ << time << "\n";
  solve_time_fout_ << solve_time << " " << mpc_->solver_->getSolveTime()
                   << "\n";
  states_fout_ << x_traj_.tail(n).transpose() << "\n";
  refs_fout_ << mpc_->x_goal_.tail(n).transpose() << "\n";
  inputs_fout_ << u_traj_.tail(m).transpose() << "\n";
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
             << "saturate_input_trajectory: "
             << mpc_->opts_.saturate_input_trajectory << "\n"
             << "u_min: " << eig2Str(mpc_->u_min_) << "\n"
             << "u_max: " << eig2Str(mpc_->u_max_) << "\n"
             << "Q: " << eig2Str(mpc_->Q_big_.diagonal().head(mpc_->state_dim_))
             << "\n"
             << "Qf: "
             << eig2Str(mpc_->Q_big_.diagonal().tail(mpc_->state_dim_)) << "\n";

  auto log_opt_vec = [&](const std::string& label, const bool condition,
                         const Eigen::Ref<const Eigen::VectorXd>& vec) {
    param_fout << label << ": ";
    if (condition) {
      param_fout << eig2Str(vec) << "\n";
    } else {
      param_fout << "None\n";
    }
  };

  log_opt_vec("R", mpc_->opts_.use_input_cost,
              mpc_->R_big_.diagonal().head(mpc_->input_dim_));
  log_opt_vec("u_slew", mpc_->opts_.slew_control_points, mpc_->u_slew_);
  log_opt_vec("x_min", mpc_->opts_.saturate_states, mpc_->x_min_);
  log_opt_vec("x_max", mpc_->opts_.saturate_states, mpc_->x_max_);

  wrote_params_ = true;
}

} // namespace affine_mpc
