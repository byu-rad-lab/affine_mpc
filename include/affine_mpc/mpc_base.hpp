#ifndef MPC_BASE_HPP
#define MPC_BASE_HPP

#include <Eigen/Core>
#include <memory>

#include "affine_mpc/options.hpp"
#include "affine_mpc/osqp_solver.hpp"
#include "affine_mpc/parameterization.hpp"

namespace affine_mpc {


class MPCBase
{
  friend class MPCLogger; // allow access to member variables for logging
protected:
  MPCBase(const int state_dim,
          const int input_dim,
          const Parameterization& parameterization,
          const Options& opts,
          const int num_design_vars,
          const int num_custom_constraints);
  virtual ~MPCBase() = default;

public:
  // Must be called before solve()
  [[nodiscard]] bool
  initializeSolver(const OSQPSettings& solver_settings =
                       OSQPSolver::getRecommendedSettings(true));
  [[nodiscard]] bool
  initializeSolver(const Eigen::Ref<const Eigen::VectorXd>& x_full,
                   const OSQPSettings& solver_settings =
                       OSQPSolver::getRecommendedSettings(true));

  // Must be called before accesing optimized variables
  [[nodiscard]] bool solve(const Eigen::Ref<const Eigen::VectorXd>& x0);

  // Getters for optimized variables
  void getNextInput(Eigen::Ref<Eigen::VectorXd> u0) const noexcept;
  void getParameterizedInputTrajectory(
      Eigen::Ref<Eigen::VectorXd> u_traj_ctrl_pts) const noexcept;
  void getInputTrajectory(Eigen::Ref<Eigen::VectorXd> u_traj) const noexcept;
  virtual void getPredictedStateTrajectory(
      Eigen::Ref<Eigen::VectorXd> x_traj) const noexcept;

  void propagateModel(const Eigen::Ref<const Eigen::VectorXd>& x,
                      const Eigen::Ref<const Eigen::VectorXd>& u,
                      Eigen::Ref<Eigen::VectorXd> x_next) const;
  bool setModelDiscrete(const Eigen::Ref<const Eigen::MatrixXd>& Ad,
                        const Eigen::Ref<const Eigen::MatrixXd>& Bd,
                        const Eigen::Ref<const Eigen::VectorXd>& wd);
  bool setModelContinuous2Discrete(const Eigen::Ref<const Eigen::MatrixXd>& Ac,
                                   const Eigen::Ref<const Eigen::MatrixXd>& Bc,
                                   const Eigen::Ref<const Eigen::VectorXd>& wc,
                                   double dt,
                                   double tol = 1e-6);

  void setWeights(const Eigen::Ref<const Eigen::VectorXd>& Q_diag,
                  const Eigen::Ref<const Eigen::VectorXd>& R_diag);
  void setWeights(const Eigen::Ref<const Eigen::VectorXd>& Q_diag,
                  const Eigen::Ref<const Eigen::VectorXd>& Qf_diag,
                  const Eigen::Ref<const Eigen::VectorXd>& R_diag);
  void setStateWeights(const Eigen::Ref<const Eigen::VectorXd>& Q_diag);
  void setStateWeights(const Eigen::Ref<const Eigen::VectorXd>& Q_diag,
                       const Eigen::Ref<const Eigen::VectorXd>& Qf_diag);
  void setInputWeights(const Eigen::Ref<const Eigen::VectorXd>& R_diag);

  bool setReferenceState(const Eigen::Ref<const Eigen::VectorXd>& x_step);
  bool
  setReferenceStateTrajectory(const Eigen::Ref<const Eigen::VectorXd>& x_traj);
  bool setReferenceInput(const Eigen::Ref<const Eigen::VectorXd>& u_step);
  bool setReferenceParameterizedInputTrajectory(
      const Eigen::Ref<const Eigen::VectorXd>& u_traj_ctrl_pts);

  bool setInputLimits(const Eigen::Ref<const Eigen::VectorXd>& u_min,
                      const Eigen::Ref<const Eigen::VectorXd>& u_max);
  bool setStateLimits(const Eigen::Ref<const Eigen::VectorXd>& x_min,
                      const Eigen::Ref<const Eigen::VectorXd>& x_max);
  bool setSlewRate(const Eigen::Ref<const Eigen::VectorXd>& u_slew);
  bool setSlewRateInitial(const Eigen::Ref<const Eigen::VectorXd>& u0_slew);
  bool setPreviousInput(const Eigen::Ref<const Eigen::VectorXd>& u_prev);

  constexpr int getStateDim() const noexcept { return state_dim_; };
  constexpr int getInputDim() const noexcept { return input_dim_; };
  constexpr int getHorizonSteps() const noexcept { return horizon_steps_; };
  constexpr int getNumControlPoints() const noexcept { return num_ctrl_pts_; };

protected:
  const int state_dim_, input_dim_;
  const int horizon_steps_, num_ctrl_pts_, spline_degree_;
  const int x_traj_dim_, u_traj_dim_, ctrls_dim_;
  // const bool use_input_cost_, use_slew_rate_, saturate_states_;
  const Options opts_;
  const int u_sat_dim_, slew_dim_, x_sat_dim_;
  const int u_sat_idx_, slew0_idx_, slew_idx_, x_sat_idx_;
  bool model_set_, u_lims_set_, slew0_rate_set_, slew_rate_set_, x_lims_set_;
  bool solver_initialized_;
  bool weights_changed_;

  // MPC variables
  Eigen::MatrixXd Ad_, Bd_;
  Eigen::VectorXd wd_;

  // Working matrices for model discretization
  Eigen::MatrixXd G_, At_, At_pow_;

  Eigen::DiagonalMatrix<double, Eigen::Dynamic> Q_big_, R_big_;
  Eigen::VectorXd x_goal_, u_goal_, u_min_, u_max_, x_min_, x_max_;
  Eigen::VectorXd u_slew_, u0_slew_, u_prev_;
  Eigen::Map<const Eigen::VectorXd> solution_map_;

  // OSQP variables - see https://osqp.org/docs/solver/index.html
  std::unique_ptr<OSQPSolver> solver_;
  Eigen::MatrixXd P_; // osqp cost matrix
  Eigen::MatrixXd A_; // osqp constraint matrix
  Eigen::VectorXd q_; // osqp cost vector
  Eigen::VectorXd l_; // osqp constraint lower bound
  Eigen::VectorXd u_; // osqp constraint upper bound

  // Input parameterization variables
  Eigen::VectorXi spline_segment_idxs_;
  Eigen::VectorXd spline_knots_;
  Eigen::MatrixXd spline_weights_;

private:
  void calcSplineParams();

  // need to be implemented by derived classes
  virtual void qpUpdateX0(const Eigen::Ref<const Eigen::VectorXd>& x0) = 0;
  virtual bool qpUpdateModel() = 0;
  virtual bool qpUpdateReferences() = 0;
  virtual bool qpUpdateInputLimits() = 0;
  virtual bool qpUpdateStateLimits() = 0;
  virtual bool qpUpdateSlewRate() = 0;
};

} // namespace affine_mpc

#endif // MPC_BASE_HPP
