#include "affine_mpc/mpc_logger.hpp"


MPCLogger::MPCLogger(const MPCBase* const mpc,
                     const std::string& time_file,
                     const std::string& states_file,
                     const std::string& ref_states_file,
                     const std::string& inputs_file) :
    mpc_{mpc},
    x_traj_{mpc->getNumStates()*mpc->getHorizonLength()},
    write_x_{bool(states_file.length())},
    write_r_{bool(ref_states_file.length())},
    write_u_{bool(inputs_file.length())}
{
  time_fout_.open(time_file);
  if (write_x_)
    states_fout_.open(states_file);
  if (write_r_)
    refs_fout_.open(ref_states_file);
  if (write_u_)
    inputs_fout_.open(inputs_file);
}

MPCLogger::~MPCLogger()
{
  time_fout_.close();
  if (write_x_)
    states_fout_.close();
  if (write_r_)
    refs_fout_.close();
  if (write_u_)
    inputs_fout_.close();
}

void MPCLogger::writeData(double t0, double ts, const Ref<const VectorXd>& x0,
                          const Ref<const VectorXd>& u_traj,
                          const Ref<const VectorXd>& x_traj_des, int write_every)
{
  static int n{mpc_->getNumStates()}, m{mpc_->getNumInputs()};
  static int T{mpc_->getHorizonLength()}, p{mpc_->getNumKnotPoints()};

  VectorXd uk{m};
  double Tp{(T-1)/double(p-1)};
  int p1, p2;
  double c, time{t0};
  int count{0};

  for (int k{0}; k < T-1; ++k)
  {
    p1 = int(k/Tp);
    p2 = p1 + 1;
    c = k/Tp - p1;
    uk = (1-c)*u_traj.segment(p1*m,m) + c*u_traj.segment(p2*m,m);
    time += ts;

    if (++count == write_every)
    {
      count = 0;
      time_fout_ << time << " ";
      time_fout_.flush();
      if (write_u_)
      {
        inputs_fout_ << uk.transpose() << " ";
        inputs_fout_.flush();
      }
    }
  }

  time += ts;
  uk = u_traj.tail(m);
  time_fout_ << time << std::endl;
  if (write_u_)
    inputs_fout_ << uk.transpose() << std::endl;

  if (write_x_)
  {
    mpc_->getPredictedStateTrajectory(x_traj_);
    states_fout_ << x_traj_.transpose() << std::endl;
  }

  if (write_r_)
    refs_fout_ << mpc_->getDesiredStateTrajectory().transpose() << std::endl;
    // refs_fout_ << x_traj_des.transpose() << std::endl;
}


// void MPCLogger::writeData(double t0, double ts, const Ref<const VectorXd>& x0,
//                           const Ref<const VectorXd>& u_traj,
//                           const Ref<const VectorXd>& x_traj_des, int write_every)
// {
//   static int n{mpc_->getNumStates()}, m{mpc_->getNumInputs()};
//   static int T{mpc_->getHorizonLength()}, p{mpc_->getNumKnotPoints()};
//   VectorXd xk{n}, uk{m};
//   xk = x0;
//   double Tp{(T-1)/double(p-1)};
//   int p1, p2;
//   double c, time{t0};
//   int count{0};

//   for (int k{0}; k < T-1; ++k)
//   {
//     p1 = int(k/Tp);
//     p2 = p1 + 1;
//     c = k/Tp - p1;
//     uk = (1-c)*u_traj.segment(p1*m,m) + c*u_traj.segment(p2*m,m);
//     mpc_->propagateModel(xk, uk, xk);
//     time += ts;

//     if (++count == write_every)
//     {
//       count = 0;
//       time_fout_ << time << " ";
//       time_fout_.flush();
//       if (write_x_)
//       {
//         states_fout_ << xk.transpose() << " ";
//         states_fout_.flush();
//       }
//       if (write_u_)
//       {
//         inputs_fout_ << uk.transpose() << " ";
//         inputs_fout_.flush();
//       }
//     }
//   }

//   time += ts;
//   uk = u_traj.tail(m);
//   mpc_->propagateModel(xk, uk, xk);
//   time_fout_ << time << std::endl;
//   if (write_x_)
//     states_fout_ << xk.transpose() << std::endl;
//   if (write_r_)
//     refs_fout_ << x_traj_des.transpose() << std::endl;
//   if (write_u_)
//     inputs_fout_ << uk.transpose() << std::endl;
// }
