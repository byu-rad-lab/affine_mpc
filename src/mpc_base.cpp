#include "affine_mpc/mpc_base.hpp"

MPCBase::MPCBase(const int n, const int m, const int T, const int p,
                 const bool use_input_cost, const bool use_slew_rate,
                 const bool saturate_states) :
    n_{n}, m_{m}, T_{T}, p_{p},
    use_input_cost_{use_input_cost},
    use_slew_rate_{use_slew_rate},
    saturate_states_{saturate_states},
    model_set_{false},
    input_limits_set_{false},
    slew_rate_set_{false},
    state_limits_set_{false},
    initialized_{false},
    solver_{nullptr},
    Ad_{n,n},
    Bd_{n,m},
    wd_{n},
    Q_big_{n*T},
    x_goal_{n*T},
    u_min_{m},
    u_max_{m},
    solution_map_{nullptr,0}
{
  assert(n > 0 && m > 0 && T > 0 && p > 0);
  Q_big_.setIdentity();
  if (use_input_cost)
  {
    R_big_.setZero(m*p);
    u_goal_.setZero(m*p);
  }
  if (use_slew_rate)
    u_slew_.setZero(m);
  if (saturate_states)
  {
    x_min_.setZero(n);
    x_max_.setZero(n);
  }
}

MPCBase::~MPCBase()
{
  if (solver_)
    delete solver_;
}

bool MPCBase::calcNextInput(const Ref<const VectorXd>& x0, Ref<VectorXd> u)
{
  assert(u.size() == m_);
  bool solved{solve(x0)};
  if (solved)
    u = solution_map_.head(m_);
  return solved;
}

bool MPCBase::calcInputTrajectory(const Ref<const VectorXd>& x0, Ref<VectorXd> u_traj)
{
  assert(u_traj.size() == m_*p_);
  bool solved{solve(x0)};
  if (solved)
    u_traj = solution_map_;
  return solved;
}

void MPCBase::unparameterizeSolution(const Ref<const VectorXd>& sol, Ref<VectorXd> u_traj) const
{
  assert(u_traj.size() == m_*T_);

  VectorXd uk{m_};
  double Tp{(T_-1)/double(p_-1)};
  int p1, p2;
  double c;

  for (int k{0}; k < T_-1; ++k)
  {
    p1 = int(k/Tp);
    p2 = p1 + 1;
    c = k/Tp - p1;
    uk = (1-c)*solution_map_.segment(p1*m_,m_) + c*solution_map_.segment(p2*m_,m_);
    u_traj.segment(k*m_,m_) = uk;
  }
  u_traj.tail(m_) = solution_map_.tail(m_);
}

void MPCBase::getPredictedStateTrajectory(Ref<VectorXd> x_traj) const
{
  assert(x_traj.size() == n_*T_);

  // CAN'T DO WITHOUT X0

  // VectorXd uk{m_};
  // double Tp{(T_-1)/double(p_-1)};
  // int p1, p2;
  // double c;

  // for (int k{0}; k < T_-1; ++k)
  // {
  //   p1 = int(k/Tp);
  //   p2 = p1 + 1;
  //   c = k/Tp - p1;
  //   uk = (1-c)*solution_map_.segment(p1*m_,m_) + c*solution_map_.segment(p2*m_,m_);
  // }
  // uk = mpc_->solution_map_.tail(m);
}

bool MPCBase::initSolver(const OSQPSettings* solver_settings)
{
  assert(model_set_ && input_limits_set_);
  if (use_slew_rate_)
    assert(slew_rate_set_);
  if (saturate_states_)
    assert(state_limits_set_);

  initialized_ = solver_->initialize(P_, A_, q_, l_, u_, solver_settings);
  new (&solution_map_) Map<const VectorXd>(solver_->getSolutionPtr(), m_*p_);
  return initialized_;
}

void MPCBase::propagateModel(const Ref<const VectorXd>& x0, const Ref<const VectorXd>& u,
                             Ref<VectorXd> x_next) const
{
  assert(model_set_);
  assert(x0.size() == n_ && u.size() == m_ && x_next.size() == n_);
  x_next = Ad_*x0 + Bd_*u + wd_;
}

void MPCBase::setModelDiscrete(const Ref<const MatrixXd>& Ad,
                               const Ref<const MatrixXd>& Bd,
                               const Ref<const VectorXd>& wd)
{
  assert(Ad.rows() == n_ && Ad.cols() == n_);
  assert(Bd.rows() == n_ && Bd.cols() == m_);
  assert(wd.size() == n_);

  Ad_ = Ad;
  Bd_ = Bd;
  wd_ = wd;
  model_set_ = true;
}

void MPCBase::setModelContinuous2Discrete(const Ref<const MatrixXd>& Ac,
                                          const Ref<const MatrixXd>& Bc,
                                          const Ref<const VectorXd>& wc,
                                          double dt, double tol)
{
  assert(Ac.rows() == n_ && Ac.cols() == n_);
  assert(Bc.rows() == n_ && Bc.cols() == m_);
  assert(wc.size() == n_);

  static MatrixXd G{n_,n_}, At{n_,n_}, At_i{n_,n_};

  At.noalias() = Ac * dt;
  At_i.setIdentity(n_,n_);
  Ad_.setIdentity(n_,n_);
  G.setIdentity(n_,n_);

  int i{1};
  double factorial{1};
  for (double t_i{dt}; t_i/factorial > tol; t_i*=dt)
  {
    At_i *= At;
    Ad_ += At_i / factorial;
    factorial *= ++i;
    G += At_i / factorial;
  }
  G *= dt;

  Bd_.noalias() = G * Bc;
  wd_.noalias() = G * wc;
  model_set_ = true;
}

void MPCBase::setWeights(const Ref<const VectorXd>& Q_diag,
                         const Ref<const VectorXd>& R_diag)
{
  setStateWeights(Q_diag);
  setInputWeights(R_diag);
}

void MPCBase::setStateWeights(const Ref<const VectorXd>& Q_diag)
{
  assert(Q_diag.size() == n_);
  for (int i{0}; i < T_; ++i)
    Q_big_.diagonal().segment(n_*i,n_) = Q_diag;
}

void MPCBase::setStateWeightsTerminal(const Ref<const VectorXd>& Qf_diag)
{
  assert(Qf_diag.size() == n_);
  Q_big_.diagonal().segment(n_*(T_-1),n_) = Qf_diag;
}

void MPCBase::setInputWeights(const Ref<const VectorXd>& R_diag)
{
  assert(use_input_cost_);
  assert(R_diag.size() == m_);
  for (int i{0}; i < p_; ++i)
    R_big_.diagonal().segment(m_*i,m_) = R_diag;
}

void MPCBase::setDesiredState(const Ref<const VectorXd>& x_step)
{
  assert(x_step.size() == n_);
  for (int i{0}; i < T_; ++i)
    x_goal_.segment(n_*i,n_) = x_step;
}

void MPCBase::setDesiredInput(const Ref<const VectorXd>& u_step)
{
  assert(use_input_cost_);
  assert(u_step.size() == m_);
  for (int i{0}; i < p_; ++i)
    u_goal_.segment(m_*i,m_) = u_step;
}

void MPCBase::setDesiredStateTrajectory(const Ref<const VectorXd>& x_traj)
{
  assert(x_traj.size() == n_*T_);
  x_goal_ = x_traj;
}

void MPCBase::setDesiredInputTrajectory(const Ref<const VectorXd>& u_traj)
{
  assert(use_input_cost_);
  assert(u_traj.size() == m_*p_);
  u_goal_ = u_traj;
}

void MPCBase::setInputLimits(const Ref<const VectorXd>& u_min,
                             const Ref<const VectorXd>& u_max)
{
  assert(u_min.size() == m_ && u_max.size() == m_);
  assert((u_max-u_min).all() >= 0.0);
  u_min_ = u_min;
  u_max_ = u_max;
  input_limits_set_ = true;
}

void MPCBase::setStateLimits(const Ref<const VectorXd>& x_min,
                             const Ref<const VectorXd>& x_max)
{
  assert(saturate_states_);
  assert(x_min.size() == n_ && x_max.size() == n_);
  assert((x_max-x_min).all() >= 0);
  x_min_ = x_min;
  x_max_ = x_max;
  state_limits_set_ = true;
}

void MPCBase::setSlewRate(const Ref<const VectorXd>& u_slew)
{
  assert(use_slew_rate_);
  assert(u_slew.size() == m_);
  u_slew_ = u_slew;
  slew_rate_set_ = true;
}

bool MPCBase::solve(const Ref<const VectorXd>& x0)
{
  convertToQP(x0);
  return solver_->solve();
}
