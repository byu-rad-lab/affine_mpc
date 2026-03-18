#include "affine_mpc/mpc_logger.hpp"

#include <cassert>
#include <cstdlib>
#include <sstream>
#include <stdexcept>

namespace affine_mpc {

std::string eig2Str(const Eigen::Ref<const Eigen::VectorXd>& vec)
{
  std::stringstream ss;
  ss << vec.transpose();
  std::string pretty_vec{ss.str()};

  if (pretty_vec.empty())
    return "[]";

  while (!pretty_vec.empty() && pretty_vec.front() == ' ')
    pretty_vec.erase(0, 1);

  std::size_t pos{pretty_vec.find(' ')};
  while (pos != std::string::npos) {
    pretty_vec.replace(pos, 1, ", ");
    pos = pretty_vec.find(' ', pos + 2);
  }

  return "[" + pretty_vec + "]";
}

MPCLogger::MPCLogger(const MPCBase* const mpc,
                     const std::filesystem::path& save_location) :
    mpc_{mpc},
    save_path_{save_location},
    x_traj_{mpc->state_dim_ * mpc->horizon_steps_},
    u_traj_{mpc_->input_dim_ * mpc_->horizon_steps_},
    wrote_params_{false}
{
  handlePathSubstitutions();

  if (!std::filesystem::exists(save_path_))
    std::filesystem::create_directories(save_path_);

  time_fout_.open(save_path_ / "time.txt");
  solve_time_fout_.open(save_path_ / "solve_times.txt");
  states_fout_.open(save_path_ / "states.txt");
  refs_fout_.open(save_path_ / "ref_states.txt");
  inputs_fout_.open(save_path_ / "inputs.txt");
  spline_knots_fout_.open(save_path_ / "spline_knots.txt");

  if (!time_fout_.is_open())
    throw std::runtime_error("[MPCLogger Constructor] "
                             "Failed to open time file for writing.");
  if (!solve_time_fout_.is_open())
    throw std::runtime_error("[MPCLogger Constructor] "
                             "Failed to open solve_times file for writing.");
  if (!states_fout_.is_open())
    throw std::runtime_error("[MPCLogger Constructor] "
                             "Failed to open states file for writing.");
  if (!refs_fout_.is_open())
    throw std::runtime_error("[MPCLogger Constructor] "
                             "Failed to open ref_states file for writing.");
  if (!inputs_fout_.is_open())
    throw std::runtime_error("[MPCLogger Constructor] "
                             "Failed to open inputs file for writing.");
  if (!spline_knots_fout_.is_open())
    throw std::runtime_error("[MPCLogger Constructor] "
                             "Failed to open spline_knots file for writing.");

  spline_knots_fout_ << mpc_->spline_knots_.transpose() << std::endl;
  spline_knots_fout_.close();
}

MPCLogger::~MPCLogger()
{
  time_fout_.close();
  solve_time_fout_.close();
  states_fout_.close();
  refs_fout_.close();
  inputs_fout_.close();
  spline_knots_fout_.close();
  if (!wrote_params_)
    writeParamFile();
}

// Writes a single line of data for each time step in the horizon
void MPCLogger::logPreviousSolve(double t0,
                                 double ts,
                                 const Eigen::Ref<const Eigen::VectorXd>& x0,
                                 const double solve_time,
                                 const int write_every)
{
  if (!mpc_->solver_initialized_)
    throw std::logic_error("[MPCLogger::logPreviousSolve] "
                           "Solver must be initialized before logging data.");
  const int n{mpc_->state_dim_};
  const int m{mpc_->input_dim_};
  const int T{mpc_->horizon_steps_};

  double time{t0};

  time_fout_ << time << " ";
  states_fout_ << x0.transpose() << " ";

  mpc_->getPredictedStateTrajectory(x_traj_);
  mpc_->getInputTrajectory(u_traj_);

  const int Tm1{T - 1};
  for (int k{0}; k < Tm1; ++k) {
    time += ts;
    if (k % write_every == 0) {
      time_fout_ << time << " ";
      states_fout_ << x_traj_.segment(k * n, n).transpose() << " ";
      refs_fout_ << mpc_->x_goal_.segment(k * n, n).transpose() << " ";
      inputs_fout_ << u_traj_.segment(k * m, m).transpose() << " ";
    }
  }

  time += ts;
  time_fout_ << time << std::endl;
  solve_time_fout_ << solve_time << " " << mpc_->solver_->getSolveTime()
                   << std::endl;
  states_fout_ << x_traj_.tail(n).transpose() << std::endl;
  refs_fout_ << mpc_->x_goal_.tail(n).transpose() << std::endl;
  inputs_fout_ << u_traj_.tail(m).transpose() << std::endl;
}

void MPCLogger::writeParamFile(const std::filesystem::path& filename)
{
  std::ofstream param_fout;
  param_fout.open(save_path_ / filename);
  if (!param_fout.is_open()) {
    throw std::runtime_error("[MPCLogger::writeParamFile] "
                             "Failed to open parameter file for writing.");
  }
  param_fout << std::boolalpha << "n: " << mpc_->state_dim_ << std::endl
             << "m: " << mpc_->input_dim_ << std::endl
             << "T: " << mpc_->horizon_steps_ << std::endl
             << "mu: " << mpc_->num_ctrl_pts_ << std::endl
             << "p: " << mpc_->spline_degree_ << std::endl
             << "use_input_cost: " << mpc_->opts_.use_input_cost << std::endl
             << "slew_initial_input: " << mpc_->opts_.slew_initial_input
             << std::endl
             << "slew_control_points: " << mpc_->opts_.slew_control_points
             << std::endl
             << "saturate_states: " << mpc_->opts_.saturate_states << std::endl
             << "u_min: " << eig2Str(mpc_->u_min_) << std::endl
             << "u_max: " << eig2Str(mpc_->u_max_) << std::endl
             << "Q: " << eig2Str(mpc_->Q_big_.diagonal().head(mpc_->state_dim_))
             << std::endl
             << "Qf: "
             << eig2Str(mpc_->Q_big_.diagonal().tail(mpc_->state_dim_))
             << std::endl;

  param_fout << "R: ";
  if (mpc_->opts_.use_input_cost)
    param_fout << eig2Str(mpc_->R_big_.diagonal().head(mpc_->input_dim_))
               << std::endl;
  else
    param_fout << "None" << std::endl;

  param_fout << "u_slew: ";
  if (mpc_->opts_.slew_control_points)
    param_fout << eig2Str(mpc_->u_slew_) << std::endl;
  else
    param_fout << "None" << std::endl;

  param_fout << "x_min: ";
  if (mpc_->opts_.saturate_states)
    param_fout << eig2Str(mpc_->x_min_) << std::endl;
  else
    param_fout << "None" << std::endl;

  param_fout << "x_max: ";
  if (mpc_->opts_.saturate_states)
    param_fout << eig2Str(mpc_->x_max_) << std::endl;
  else
    param_fout << "None" << std::endl;

  param_fout.close();
  wrote_params_ = true;
}

void MPCLogger::handlePathSubstitutions()
{
  const std::string raw = save_path_.string();
  if (raw.empty())
    return;

  const char* home_cstr = std::getenv("HOME");
  if (home_cstr == nullptr)
    return;

  const std::string home{home_cstr};

  if (raw.front() == '~') {
    save_path_ = std::filesystem::path{home} / raw.substr(1);
    return;
  }

  if (raw.rfind("$HOME", 0) == 0) {
    save_path_ = std::filesystem::path{home} / raw.substr(5);
    return;
  }

  if (raw.rfind("${HOME}", 0) == 0) {
    save_path_ = std::filesystem::path{home} / raw.substr(7);
    return;
  }
}

} // namespace affine_mpc
