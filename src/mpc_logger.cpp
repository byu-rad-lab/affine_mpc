#include "affine_mpc/mpc_logger.hpp"

#include <filesystem>
#include <cassert>

namespace fs = std::filesystem;
namespace affine_mpc {


std::string eig2Str(const Eigen::Ref<const Eigen::VectorXd>& vec)
{
  std::stringstream ss;
  ss << vec.transpose();
  std::string pretty_vec{ss.str()};

  while (pretty_vec.front() == std::string(" ").front())
    pretty_vec.replace(0,1,"");

  std::size_t pos{pretty_vec.find(" ")};
  while (pos != std::string::npos)
  {
    pretty_vec.replace(pos, 1, ", ");
    pos = pretty_vec.find(" ", pos+2);
  }

  return "[" + pretty_vec + "]";
}


MPCLogger::MPCLogger(const MPCBase* const mpc,
                     const std::string& save_location) :
    mpc_{mpc},
    save_dir_{save_location},
    x_traj_{mpc->num_states_*mpc->len_horizon_},
    u_traj_{mpc_->num_inputs_*mpc_->len_horizon_},
    wrote_params_{false}
{
  handleStringSubstitutions();

  if (!fs::exists(save_dir_))
    fs::create_directories(save_dir_);

  time_fout_.open(save_dir_ + "time.txt");
  solve_time_fout_.open(save_dir_ + "solve_times.txt");
  states_fout_.open(save_dir_ + "states.txt");
  refs_fout_.open(save_dir_ + "ref_states.txt");
  inputs_fout_.open(save_dir_ + "inputs.txt");

  assert(time_fout_.is_open());
  assert(solve_time_fout_.is_open());
  assert(states_fout_.is_open());
  assert(refs_fout_.is_open());
  assert(inputs_fout_.is_open());
}

MPCLogger::~MPCLogger()
{
  time_fout_.close();
  solve_time_fout_.close();
  states_fout_.close();
  refs_fout_.close();
  inputs_fout_.close();
  if (!wrote_params_)
    writeParamFile();
}

// Writes a single line of data for each time step in the horizon
void MPCLogger::logPreviousSolve(double t0, double ts,
                                 const Eigen::Ref<const Eigen::VectorXd>& x0,
                                 double solve_time, int write_every)
{
  assert(mpc_->solver_initialized_);
  const static int n{mpc_->num_states_}, m{mpc_->num_inputs_};
  const static int T{mpc_->len_horizon_}, p{mpc_->num_ctrl_pts_};

  double time{t0};

  time_fout_ << time << " ";
  states_fout_ << x0.transpose() << " ";

  mpc_->getPredictedStateTrajectory(x_traj_);
  mpc_->getInputTrajectory(u_traj_);

  int Tm1{T-1};
  for (int k{0}; k < Tm1; ++k)
  {
    time += ts;
    if (k % write_every == 0)
    {
      time_fout_ << time << " ";
      states_fout_ << x_traj_.segment(k*n,n).transpose() << " ";
      refs_fout_ << mpc_->x_goal_.segment(k*n,n).transpose() << " ";
      inputs_fout_ << u_traj_.segment(k*m,m).transpose() << " ";
    }
  }
  // always write data from last time step
  time += ts;
  time_fout_ << time << std::endl;
  solve_time_fout_ << solve_time << " " << mpc_->solver_->getSolveTime() << std::endl;
  states_fout_ << x_traj_.tail(n).transpose() << std::endl;
  refs_fout_ << mpc_->x_goal_.tail(n).transpose() << std::endl;
  inputs_fout_ << u_traj_.tail(m).transpose() << std::endl;
}

void MPCLogger::writeParamFile(const std::string& filename)
{
  std::ofstream param_fout;
  param_fout.open(save_dir_ + filename);
  assert(param_fout.is_open());
  param_fout << std::boolalpha
    << "n: " << mpc_->num_states_ << std::endl
    << "m: " << mpc_->num_inputs_ << std::endl
    << "T: " << mpc_->len_horizon_ << std::endl
    << "p: " << mpc_->num_ctrl_pts_ << std::endl
    << "use_input_cost: " << mpc_->use_input_cost_ << std::endl
    << "use_slew_rate: " << mpc_->use_slew_rate_ << std::endl
    << "saturate_states: " << mpc_->saturate_states_ << std::endl
    << "u_min: " << eig2Str(mpc_->u_min_) << std::endl
    << "u_max: " << eig2Str(mpc_->u_max_) << std::endl
    << "Q: " << eig2Str(mpc_->Q_big_.diagonal().head(mpc_->num_states_)) << std::endl
    << "Qf: " << eig2Str(mpc_->Q_big_.diagonal().tail(mpc_->num_states_)) << std::endl;

  param_fout << "R: ";
  if (mpc_->use_input_cost_)
    param_fout << eig2Str(mpc_->R_big_.diagonal().head(mpc_->num_inputs_)) << std::endl;
  else
    param_fout << "None" << std::endl;

  param_fout << "u_slew: ";
  if (mpc_->use_slew_rate_)
    param_fout << eig2Str(mpc_->u_slew_) << std::endl;
  else
    param_fout << "None" << std::endl;

  param_fout << "x_min: ";
  if (mpc_->saturate_states_)
    param_fout << eig2Str(mpc_->x_min_) << std::endl;
  else
    param_fout << "None" << std::endl;

  param_fout << "x_max: ";
  if (mpc_->saturate_states_)
    param_fout << eig2Str(mpc_->x_max_) << std::endl;
  else
    param_fout << "None" << std::endl;

  param_fout.close();
  wrote_params_ = true;
}

void MPCLogger::handleStringSubstitutions()
{
  std::string slash{"/"};
  if (save_dir_.back() != slash.back())
    save_dir_ += slash;

  if (save_dir_.front() == std::string("~").front()
      || save_dir_.substr(0,5) == std::string("$HOME")
      || save_dir_.substr(0,7) == std::string("${HOME}"))
  {
    std::string env_home{std::getenv("HOME")};
    int pos = save_dir_.find_first_of("/");
    save_dir_.replace(0, pos, env_home);
  }
}

} // namespace affine_mpc
