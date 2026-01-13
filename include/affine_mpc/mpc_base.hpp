#ifndef MPC_BASE_HPP
#define MPC_BASE_HPP

#include <Eigen/Core>

#include "affine_mpc/osqp_solver.hpp"

namespace affine_mpc {


class MPCBase
{
  friend class MPCLogger; // allow acces to member variables for logging
public:
  // MPCBase(const Eigen::Ref<const Eigen::MatrixXd>& A,
  //         const Eigen::Ref<const Eigen::MatrixXd>& B,
  //         const Eigen::Ref<const Eigen::VectorXd>& w, const int len_horizon,
  //         const int num_control_points, const int degree,
  //         const Eigen::Ref<const Eigen::VectorXd>& Q_diag,
  //         const Eigen::Ref<const Eigen::VectorXd>& R_diag,
  //         const Eigen::Ref<const Eigen::VectorXd>& Qf_diag =
  //         Eigen::VectorXd(0), const Eigen::Ref<const Eigen::VectorXd>& knots
  //         = Eigen::VectorXd(0), const bool use_input_cost = false, const bool
  //         use_slew_rate = false, const bool saturate_states = false);
  MPCBase(const int num_states, const int num_inputs, const int len_horizon,
          const int num_control_points, const int degree,
          const Eigen::Ref<const Eigen::VectorXd>& knots = Eigen::VectorXd(0),
          const bool use_input_cost = false, const bool use_slew_rate = false,
          const bool saturate_states = false);
  virtual ~MPCBase();

  // Must be called before solve()
  bool initializeSolver(const OSQPSettings* solver_settings = nullptr);

  // Must be called before accesing optimized variables
  bool solve(const Eigen::Ref<const Eigen::VectorXd>& x0);

  // Getters for optimized variables
  void getNextInput(Eigen::Ref<Eigen::VectorXd> u0) const;
  void getParameterizedInputTrajectory(
      Eigen::Ref<Eigen::VectorXd> u_traj_ctrl_pts) const;
  virtual void getInputTrajectory(Eigen::Ref<Eigen::VectorXd> u_traj) const;
  virtual void
  getPredictedStateTrajectory(Eigen::Ref<Eigen::VectorXd> x_traj) const;

  void propagateModel(const Eigen::Ref<const Eigen::VectorXd>& x,
                      const Eigen::Ref<const Eigen::VectorXd>& u,
                      Eigen::Ref<Eigen::VectorXd> x_next) const;
  void setModelDiscrete(const Eigen::Ref<const Eigen::MatrixXd>& Ad,
                        const Eigen::Ref<const Eigen::MatrixXd>& Bd,
                        const Eigen::Ref<const Eigen::VectorXd>& wd);
  void setModelContinuous2Discrete(const Eigen::Ref<const Eigen::MatrixXd>& Ac,
                                   const Eigen::Ref<const Eigen::MatrixXd>& Bc,
                                   const Eigen::Ref<const Eigen::VectorXd>& wc,
                                   double dt, double tol = 1e-6);

  void setWeights(const Eigen::Ref<const Eigen::VectorXd>& Q_diag,
                  const Eigen::Ref<const Eigen::VectorXd>& R_diag);
  void setStateWeights(const Eigen::Ref<const Eigen::VectorXd>& Q_diag);
  void
  setStateWeightsTerminal(const Eigen::Ref<const Eigen::VectorXd>& Qf_diag);
  void setInputWeights(const Eigen::Ref<const Eigen::VectorXd>& R_diag);

  void setReferenceState(const Eigen::Ref<const Eigen::VectorXd>& x_step);
  void setReferenceInput(const Eigen::Ref<const Eigen::VectorXd>& u_step);
  void
  setReferenceStateTrajectory(const Eigen::Ref<const Eigen::VectorXd>& x_traj);
  void setReferenceParameterizedInputTrajectory(
      const Eigen::Ref<const Eigen::VectorXd>& u_traj_ctrl_pts);

  void setInputLimits(const Eigen::Ref<const Eigen::VectorXd>& u_min,
                      const Eigen::Ref<const Eigen::VectorXd>& u_max);
  void setStateLimits(const Eigen::Ref<const Eigen::VectorXd>& x_min,
                      const Eigen::Ref<const Eigen::VectorXd>& x_max);
  void setSlewRate(const Eigen::Ref<const Eigen::VectorXd>& u_slew);

  inline int getNumStates() const { return num_states_; };
  inline int getNumInputs() const { return num_inputs_; };
  inline int getHorizonLength() const { return len_horizon_; };
  inline int getNumControlPoints() const { return num_ctrl_pts_; };

private:
  void initializeKnots(const Eigen::Ref<const Eigen::VectorXd>& knots);
  void calcSplineParams();

protected:
  // need to be implemented by derived classes
  virtual void convertToQP(const Eigen::Ref<const Eigen::VectorXd>& x0) = 0;

  const int num_states_, num_inputs_;
  const int len_horizon_, num_ctrl_pts_, degree_;
  const bool use_input_cost_, use_slew_rate_, saturate_states_;
  bool model_set_, input_limits_set_, slew_rate_set_, state_limits_set_;
  bool solver_initialized_;

  // OSQP variables - see https://osqp.org/docs/solver/index.html
  OSQPSolver* solver_;
  Eigen::MatrixXd P_; // osqp cost matrix
  Eigen::MatrixXd A_; // osqp constraint matrix
  Eigen::VectorXd q_; // osqp cost vector
  Eigen::VectorXd l_; // osqp constraint lower bound
  Eigen::VectorXd u_; // osqp constraint upper bound

  // Input parameterization variables
  Eigen::VectorXd spline_segment_idxs_;
  Eigen::VectorXd spline_knots_;
  Eigen::MatrixXd spline_weights_;

  // MPC variables
  Eigen::MatrixXd Ad_;
  Eigen::MatrixXd Bd_;
  Eigen::VectorXd wd_;
  Eigen::DiagonalMatrix<double, Eigen::Dynamic> Q_big_;
  Eigen::DiagonalMatrix<double, Eigen::Dynamic> R_big_;
  Eigen::VectorXd x_goal_;
  Eigen::VectorXd u_goal_;
  Eigen::VectorXd u_min_;
  Eigen::VectorXd u_max_;
  Eigen::VectorXd x_min_;
  Eigen::VectorXd x_max_;
  Eigen::VectorXd u_slew_;
  Eigen::Map<const Eigen::VectorXd> solution_map_;
};

} // namespace affine_mpc

#endif // MPC_BASE_HPP
