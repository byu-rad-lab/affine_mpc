#include "affine_mpc/mpc_base.hpp"

#include <cassert>   // for assert
#include <iostream>  // for std::cerr
#include <stdexcept> // for exceptions

// #include <Eigen/Core>  // revert back to this once Eigen 3.5 is required
#include "eigen_compat.hpp"  // revmove this once Eigen 3.5 is required
#include <unsupported/Eigen/Splines> // for B-spline support
#include <osqp.h>                    // for OSQPSettings

using namespace Eigen;
// namespace ph = Eigen::placeholders;  // revert back to this once Eigen 3.5 is required

namespace affine_mpc {


MPCBase::MPCBase(const int state_dim,
                 const int input_dim,
                 const int horizon_steps,
                 const int num_control_points,
                 const int spline_degree,
                 const Ref<const VectorXd>& spline_knots,
                 const bool use_input_cost,
                 const bool use_slew_rate,
                 const bool saturate_states) :
    state_dim_{state_dim},
    input_dim_{input_dim},
    horizon_steps_{horizon_steps},
    num_ctrl_pts_{num_control_points},
    spline_degree_{spline_degree},
    use_input_cost_{use_input_cost},
    use_slew_rate_{use_slew_rate},
    saturate_states_{saturate_states},
    model_set_{false},
    input_limits_set_{false},
    slew_rate_set_{false},
    state_limits_set_{false},
    solver_initialized_{false},
    solver_{nullptr},
    spline_segment_idxs_{horizon_steps},
    spline_knots_{num_control_points + spline_degree + 1},
    spline_weights_{spline_degree + 1, horizon_steps},
    Ad_{state_dim, state_dim},
    Bd_{state_dim, input_dim},
    wd_{state_dim},
    Q_big_{state_dim * horizon_steps},
    x_goal_{state_dim * horizon_steps},
    u_min_{input_dim},
    u_max_{input_dim},
    solution_map_{nullptr, 0}
{
  if (state_dim <= 0)
    throw std::invalid_argument("state_dim must be greater than zero.");
  if (input_dim <= 0)
    throw std::invalid_argument("input_dim must be greater than zero.");
  if (horizon_steps <= 0)
    throw std::invalid_argument("horizon_steps must be greater than zero.");
  if (num_control_points <= 0 || num_control_points > horizon_steps)
    throw std::invalid_argument(
        "num_control_points must be between zero and horizon_steps.");

  // set defaults
  Q_big_.setIdentity();
  u_min_.setConstant(-std::numeric_limits<double>::infinity());
  u_max_.setConstant(std::numeric_limits<double>::infinity());

  // allocate memory needed based on options
  if (use_input_cost) {
    R_big_.setZero(input_dim * num_control_points);
    u_goal_.setZero(input_dim * num_control_points);
  }
  if (use_slew_rate) {
    // u_slew_.setZero(input_dim);
    u_slew_.resize(input_dim);
    u_slew_.setConstant(std::numeric_limits<double>::infinity());
  }
  if (saturate_states) {
    // x_min_.setZero(state_dim);
    // x_max_.setZero(state_dim);
    x_min_.resize(state_dim);
    x_max_.resize(state_dim);
    x_min_.setConstant(-std::numeric_limits<double>::infinity());
    x_max_.setConstant(std::numeric_limits<double>::infinity());
  }

  initializeSplineKnots(spline_knots);
  calcSplineParams();
}

MPCBase::~MPCBase()
{
  // Derived classes create solver_ but do not need to delete it.
  // This deletes the solver, derived classes can use default destructor.
  if (solver_)
    delete solver_;
}

bool MPCBase::initializeSolver(const OSQPSettings* solver_settings)
{
  if (!model_set_)
    throw std::runtime_error("Model must be set before initializing solver.");
  if (!input_limits_set_)
    throw std::runtime_error(
        "Input limits must be set before initializing solver.");

  if (use_slew_rate_ && !slew_rate_set_)
    std::cerr
        << "Warning: Slew rate enabled but not set, "
        << "using default value of infinity."
        << "This is a waste of optimization effort and memory."
        << "If you do not actually want a slew rate, you should disable it."
        << std::endl;
  if (saturate_states_ && !state_limits_set_)
    std::cerr
        << "Warning: State saturation enabled but not set, "
        << "using default values of +/- infinity"
        << "This is a waste of optimization effort and memory."
        << "If you do not actually want a slew rate, you should disable it."
        << std::endl;

  if (solver_initialized_)
    return true;

  // Avoid zeros in initial cost/constraint matrices. OSQP is a sparse solver
  // that only tracks elements that are initially non-zero.
  VectorXd x_full{state_dim_}; // TODO: decide if user should provide this
  x_full.setOnes();             // may need something other than ones
  convertToQP(x_full);

  solver_initialized_ =
      solver_->initialize(P_, A_, q_, l_, u_, solver_settings);
  // According to Eigen's documentation, use "placement new" syntax to update
  // solution_map_ with the new pointer to the solution vector. This does not
  // allocate new memory that needs to be deleted, it just updates the pointer.
  // This location in memory is managed by the solver_ object and is unchanged
  // while the solver_ is in scope. solution_map_ will function like an Eigen
  // VectorXd object that automatically updates when solve() is called. Using
  // Eigen Map allows the solution to be accessed without copying memory.
  new (&solution_map_) Map<const VectorXd>(solver_->getSolutionPtr(),
                                           input_dim_ * num_ctrl_pts_);
  return solver_initialized_;
}

bool MPCBase::solve(const Ref<const VectorXd>& x0)
{
  assert(x0.size() == state_dim_);
  convertToQP(x0);
  return solver_->solve();
}

void MPCBase::getNextInput(Ref<VectorXd> u0) const
{
  assert(u0.size() == input_dim_);
  u0 = solution_map_.head(input_dim_);
}

void MPCBase::getParameterizedInputTrajectory(
    Ref<VectorXd> u_traj_ctrl_pts) const
{
  assert(u_traj_ctrl_pts.size() == input_dim_ * num_ctrl_pts_);
  u_traj_ctrl_pts = solution_map_;
  // u_traj_ctrl_pts = solution_map_.head(input_dim_ * num_ctrl_pts_);
}

void MPCBase::getInputTrajectory(Ref<VectorXd> u_traj) const
{
  assert(u_traj.size() == input_dim_ * horizon_steps_);

  int seg;
  Map<const MatrixXd> ctrls{solution_map_.data(), input_dim_, num_ctrl_pts_};
  VectorXd weights;

  for (int k{0}; k < horizon_steps_; ++k) {
    seg = spline_segment_idxs_(k);
    u_traj(seqN(k, input_dim_)) =
        ctrls(ph::all, seqN(seg, spline_degree_ + 1)) * spline_weights_.col(k);
  }
}

void MPCBase::getPredictedStateTrajectory(Ref<VectorXd> x_traj) const
{
  assert(x_traj.size() == state_dim_ * horizon_steps_);
}

void MPCBase::propagateModel(const Ref<const VectorXd>& x0,
                             const Ref<const VectorXd>& u,
                             Ref<VectorXd> x_next) const
{
  if (!model_set_) {
    throw std::runtime_error("Model must be set before propagation.");
  }
  assert(u.size() == input_dim_);
  assert(x0.size() == state_dim_ && x_next.size() == state_dim_);
  x_next = Ad_ * x0 + Bd_ * u + wd_;
}

void MPCBase::setModelDiscrete(const Ref<const MatrixXd>& Ad,
                               const Ref<const MatrixXd>& Bd,
                               const Ref<const VectorXd>& wd)
{
  assert(Ad.rows() == state_dim_ && Ad.cols() == state_dim_);
  assert(Bd.rows() == state_dim_ && Bd.cols() == input_dim_);
  assert(wd.size() == state_dim_);

  Ad_ = Ad;
  Bd_ = Bd;
  wd_ = wd;
  model_set_ = true;
}

void MPCBase::setModelContinuous2Discrete(const Ref<const MatrixXd>& Ac,
                                          const Ref<const MatrixXd>& Bc,
                                          const Ref<const VectorXd>& wc,
                                          double dt,
                                          double tol)
{
  assert(Ac.rows() == state_dim_ && Ac.cols() == state_dim_);
  assert(Bc.rows() == state_dim_ && Bc.cols() == input_dim_);
  assert(wc.size() == state_dim_);

  static MatrixXd G{state_dim_, state_dim_};
  static MatrixXd At{state_dim_, state_dim_};
  static MatrixXd At_i{state_dim_, state_dim_};

  At.noalias() = Ac * dt;
  At_i.setIdentity(state_dim_, state_dim_);
  Ad_.setIdentity(state_dim_, state_dim_);
  G.setIdentity(state_dim_, state_dim_);

  int i{1};
  double factorial{1};
  for (double t_i{dt}; t_i / factorial > tol; t_i *= dt) {
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
  if (Q_diag.minCoeff() < 0.0)
    throw std::invalid_argument("State weights must be non-negative.");
  assert(Q_diag.size() == state_dim_);
  for (int i{0}; i < horizon_steps_; ++i)
    Q_big_.diagonal().segment(state_dim_ * i, state_dim_) = Q_diag;
}

void MPCBase::setStateWeightsTerminal(const Ref<const VectorXd>& Qf_diag)
{
  if (Qf_diag.minCoeff() < 0.0)
    throw std::invalid_argument("Terminal state weights must be non-negative.");
  assert(Qf_diag.size() == state_dim_);
  Q_big_.diagonal().segment(state_dim_ * (horizon_steps_ - 1), state_dim_) =
      Qf_diag;
}

void MPCBase::setInputWeights(const Ref<const VectorXd>& R_diag)
{
  if (!use_input_cost_)
    throw std::runtime_error("Input cost is not enabled.");
  if (R_diag.minCoeff() < 0.0)
    throw std::invalid_argument("Input weights must be non-negative.");
  assert(R_diag.size() == input_dim_);
  for (int i{0}; i < num_ctrl_pts_; ++i)
    R_big_.diagonal().segment(input_dim_ * i, input_dim_) = R_diag;
}

void MPCBase::setReferenceState(const Ref<const VectorXd>& x_step)
{
  assert(x_step.size() == state_dim_);
  for (int k{0}; k < horizon_steps_; ++k)
    x_goal_.segment(state_dim_ * k, state_dim_) = x_step;
}

void MPCBase::setReferenceInput(const Ref<const VectorXd>& u_step)
{
  if (!use_input_cost_)
    throw std::runtime_error("Input cost is not enabled.");
  assert(u_step.size() == input_dim_);
  for (int i{0}; i < num_ctrl_pts_; ++i)
    u_goal_.segment(input_dim_ * i, input_dim_) = u_step;
}

void MPCBase::setReferenceStateTrajectory(const Ref<const VectorXd>& x_traj)
{
  assert(x_traj.size() == state_dim_ * horizon_steps_);
  x_goal_ = x_traj;
}

void MPCBase::setReferenceParameterizedInputTrajectory(
    const Ref<const VectorXd>& u_traj_ctrl_pts)
{
  if (!use_input_cost_)
    throw std::runtime_error("Input cost is not enabled.");
  assert(u_traj_ctrl_pts.size() == input_dim_ * num_ctrl_pts_);
  u_goal_ = u_traj_ctrl_pts;
}

void MPCBase::setInputLimits(const Ref<const VectorXd>& u_min,
                             const Ref<const VectorXd>& u_max)
{
  if ((u_max - u_min).minCoeff() < 0.0)
    throw std::invalid_argument("u_min cannot be greater than u_max.");
  assert(u_min.size() == input_dim_ && u_max.size() == input_dim_);
  u_min_ = u_min;
  u_max_ = u_max;
  input_limits_set_ = true;
}

void MPCBase::setStateLimits(const Ref<const VectorXd>& x_min,
                             const Ref<const VectorXd>& x_max)
{
  if (!saturate_states_)
    throw std::runtime_error("State saturation is not enabled.");
  if ((x_max - x_min).minCoeff() < 0.0)
    throw std::invalid_argument("x_min cannot be greater than x_max.");
  assert(x_min.size() == state_dim_ && x_max.size() == state_dim_);
  x_min_ = x_min;
  x_max_ = x_max;
  state_limits_set_ = true;
}

void MPCBase::setSlewRate(const Ref<const VectorXd>& u_slew)
{
  if (!use_slew_rate_)
    throw std::runtime_error("Slew rate is not enabled.");
  if (u_slew.minCoeff() < 0.0)
    throw std::invalid_argument("Slew rate must be non-negative.");
  assert(u_slew.size() == input_dim_);
  u_slew_ = u_slew;
  slew_rate_set_ = true;
}

void MPCBase::initializeSplineKnots(const Ref<const VectorXd>& spline_knots)
{
  spline_knots_.head(spline_degree_).setZero();
  spline_knots_.tail(spline_degree_).setConstant(horizon_steps_ - 1);

  const int num_internal_knots = spline_knots_.size() - 2 * spline_degree_;
  const size_t knots_size = spline_knots.size();
  if (knots_size > 0) {
    if (knots_size != num_internal_knots) {
      std::string err_msg =
          "spline_knots size must be equal to num_control_points - spline_degree + 1 (" +
          std::to_string(num_internal_knots) + "), got " +
          std::to_string(knots_size) + ".";
      throw std::invalid_argument(err_msg);
    }
    if ((spline_knots(seq(1, ph::last)) - spline_knots(seq(0, ph::last - 1))).minCoeff() < 0)
      throw std::invalid_argument("spline_knots must be non-decreasing.");

    spline_knots_(seq(spline_degree_, ph::last - spline_degree_)) = spline_knots;
  } else {
    spline_knots_(seq(spline_degree_, ph::last - spline_degree_)) =
        ArrayXd::LinSpaced(num_internal_knots, 0, horizon_steps_ - 1);
  }
}

void MPCBase::calcSplineParams()
{
  using Spline1d = Spline<double, 1>;
  for (int k{0}; k < horizon_steps_; ++k) {
    const double t = k;
    const int span = Spline1d::Span(t, spline_degree_, spline_knots_);
    spline_segment_idxs_(k) = span - spline_degree_;

    spline_weights_.col(k) =
        Spline1d::BasisFunctions(t, spline_degree_, spline_knots_);
  }
}

} // namespace affine_mpc
