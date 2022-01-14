#include "osqp_mpc/implicit_mpc.hpp"

ImplicitMPC::ImplicitMPC(const int n, const int m, const int T, const int p,
                         const bool use_input_cost, const bool use_slew_rate,
                         const bool saturate_states) :
    MPCBase(n,m,T,p,use_input_cost,use_slew_rate,saturate_states),
    x_sat_idx_{m*p + m*(p-1)*use_slew_rate},
    num_constraints_{x_sat_idx_ + n*T*saturate_states},
    initialized_{false},
    solver_{m*p, num_constraints_},
    P_{m*p, m*p},
    A_{num_constraints_, m*p},
    S_{n*T, m*p},
    v_{n*T},
    q_{m*p},
    l_{num_constraints_},
    u_{num_constraints_}
{
  A_.setIdentity(); // input saturation conststraint
  if (use_slew_rate)
  {
    A_.block(m_*p_, 0, m_*p_-m_, m_*p_).diagonal().setConstant(-1);
    A_.block(m_*p_, m_, m_*p_-m_, m_*p_-m_).diagonal().setOnes();
  }
}

ImplicitMPC::~ImplicitMPC() {}

bool ImplicitMPC::calcNextInput(const Ref<const VectorXd>& x0, Ref<VectorXd> u)
{
  bool solved{solve(x0)};
  if (solved)
    u = Map<const VectorXd>{solver_.getSolutionPtr(), m_, 1};
  return solved;
}

bool ImplicitMPC::calcInputTrajectory(const Ref<const VectorXd>& x0, Ref<VectorXd> u_traj)
{
  bool solved{solve(x0)};
  if (solved)
    u_traj = Map<const VectorXd>{solver_.getSolutionPtr(), m_*p_, 1};
  return solved;
}

void ImplicitMPC::initSolver(const OSQPSettings* solver_settings)
{
  assert(model_set_ && input_limits_set_);
  if (use_slew_rate_)
    assert(slew_rate_set_);
  if (saturate_states_)
    assert(state_limits_set_);

  VectorXd x_full{n_};
  x_full.setOnes();
  convertToQP(x_full);

  initialized_ = solver_.initialize(P_, A_, q_, l_, u_, solver_settings);
}

void ImplicitMPC::setInputLimits(const Ref<const VectorXd>& u_min,
                                 const Ref<const VectorXd>& u_max)
{
  MPCBase::setInputLimits(u_min, u_max);

  for (int k{0}; k < p_; ++k)
  {
    l_.segment(m_*k, m_) = u_min_;
    u_.segment(m_*k, m_) = u_max_;
  }
  if (initialized_)
    solver_.updateBounds(l_, u_);
}

void ImplicitMPC::setStateLimits(const Ref<const VectorXd>& x_min,
                                 const Ref<const VectorXd>& x_max)
{
  MPCBase::setStateLimits(x_min, x_max);

  A_.block(x_sat_idx_, 0, n_*T_, m_*p_) = S_;
  for (int k{0}; k < T_; ++k)
  {
    l_.segment(x_sat_idx_+n_*k, n_) = x_min_;
    u_.segment(x_sat_idx_+n_*k, n_) = x_max_;
  }
  if (initialized_)
    solver_.updateBounds(l_, u_);
}

void ImplicitMPC::setSlewRate(const Ref<const VectorXd>& u_slew)
{
  MPCBase::setSlewRate(u_slew);

  int mp{m_*p_};
  int max{p_-1};
  for (int i{0}; i < max; ++i)
  {
    l_.segment(mp+m_*i, m_) = -u_slew_;
    u_.segment(mp+m_*i, m_) = u_slew_;
  }
  if (initialized_)
    solver_.updateBounds(l_, u_);
}

bool ImplicitMPC::solve(const Ref<const VectorXd>& x0)
{
  convertToQP(x0);
  solver_.updateCostMatrix(P_);
  solver_.updateCostVector(q_);
  if (saturate_states_)
    solver_.updateConstraintMatrix(A_);
  return solver_.solve();
}

void ImplicitMPC::convertToQP(const Ref<const VectorXd>& x0)
{
  calcSAndV(x0);
  calcPandQ();
  if (saturate_states_)
    A_.block(x_sat_idx_, 0, n_*T_, m_*p_) = S_;
}

void ImplicitMPC::calcSAndV(const Ref<const VectorXd>& x0)
{
  double Tp{(T_-1)/double(p_-1)};

  S_.block(0,0,n_,m_) = Bd_;
  v_.segment(0,n_) = Ad_*x0 + wd_;

  int i,j;
  double c;
  for (int k{1}; k < T_; ++k)
  {
    i = k / Tp;
    j = i + 1;
    c = k/Tp - i;

    S_.block(n_*k,0,n_,m_*p_) = Ad_ * S_.block(n_*(k-1),0,n_,m_*p_);
    S_.block(n_*k,m_*i,n_,m_) += (1-c) * Bd_;

    if (j < p_)
      S_.block(n_*k,m_*j,n_,m_) += c * Bd_;

    v_.segment(n_*k,n_) = Ad_*v_.segment(n_*(k-1),n_) + wd_;
  }
}

void ImplicitMPC::calcPandQ()
{
  P_ = S_.transpose() * Q_big_ * S_;
  q_ = S_.transpose() * Q_big_ * (v_ - x_goal_);
  if (use_input_cost_)
  {
    P_ += R_big_;
    q_ -= R_big_*u_goal_;
  }
}
