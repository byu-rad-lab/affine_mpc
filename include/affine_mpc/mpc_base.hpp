#ifndef AFFINE_MPC_MPC_BASE_HPP
#define AFFINE_MPC_MPC_BASE_HPP

#include <Eigen/Core>
#include <memory>

#include "affine_mpc/options.hpp"
#include "affine_mpc/osqp_solver.hpp"
#include "affine_mpc/parameterization.hpp"
#include "affine_mpc/solve_status.hpp"

/**
 * @file mpc_base.hpp
 * @brief Defines the abstract MPCBase class.
 */

namespace affine_mpc {

/**
 * @class MPCBase
 * @brief Abstract base class for affine MPC formulations.
 *
 * Defines the common interface and shared logic for condensed and sparse MPC
 * variants. Manages OSQP solver, input parameterization via B-splines, and
 * runtime configuration.
 *
 * Derived classes must implement QP update routines for state, model,
 * references, and constraints.
 */
class MPCBase
{
  friend class MPCLogger; // allow access to member variables for logging
protected:
  /**
   * @brief Construct MPCBase with dimensions, parameterization, options, and
   *   constraint counts.
   * @param state_dim State vector dimension.
   * @param input_dim Input vector dimension.
   * @param parameterization Input trajectory parameterization.
   * @param opts Optional MPC configuration features to enable.
   * @param num_design_vars Number of QP decision variables.
   * @param num_custom_constraints Number of custom constraints (other than
   *   input saturation, slew rate constraints, and state saturation).
   */
  MPCBase(int state_dim,
          int input_dim,
          const Parameterization& parameterization,
          const Options& opts,
          int num_design_vars,
          int num_custom_constraints);

public:
  virtual ~MPCBase() = default;

  /**
   * @brief Initialize OSQP solver after configuring MPC setup. Calling this
   *   method after a successful initialization will simply return true without
   *   doing anything.
   *
   *   Prior to calling this function, you must have set the model and input
   *   limits. You must also provide parameters for all enabled options before
   *   calling this function. For example, if state saturation is enabled then
   *   `setStateLimits()` must be called beforehand.
   * @param solver_settings OSQP solver settings (optional). Defaults to
   *   recommended settings.
   * @return True if initialization succeeds, false otherwise.
   */
  [[nodiscard]] bool initializeSolver(const OSQPSettings& solver_settings =
                                          OSQPSolver::getRecommendedSettings());

  /**
   * @brief Solve the optimization problem for the given initial state.
   *
   *   Must have previously called initializeSolver() prior to calling this.
   *   Call getNextInput(), getInputControlPoints(), getInputTrajectory(), or
   *   getPredictedStateTrajectory() after calling this function to access the
   *   results.
   * @param x0 Initial (current) state vector.
   * @return SolveStatus Result indication. Generally expected to be `Success`
   *   unless the solver has not been initialized, then `NotInitialized`. Verify
   *   your problem setup and consult OSQP documentation for any other value.
   */
  [[nodiscard]] SolveStatus solve(const Eigen::Ref<const Eigen::VectorXd>& x0);

  /**
   * @brief Get the next input to apply (initial input from optimized
   *   trajectory) from the previous solve.
   * @param u0 Output vector for input.
   */
  void getNextInput(Eigen::Ref<Eigen::VectorXd> u0) const noexcept;

  /**
   * @brief Get the control points that parameterize the input trajectory from
   *   the previous solve.
   * @param control_points Output vector for stacked control points.
   */
  void getInputControlPoints(
      Eigen::Ref<Eigen::VectorXd> control_points) const noexcept;

  /**
   * @brief Get the full input trajectory from the previous solve.
   * @param u_traj Output vector for input trajectory.
   */
  void getInputTrajectory(Eigen::Ref<Eigen::VectorXd> u_traj) const noexcept;

  /**
   * @brief Get the predicted state trajectory from the previous solve.
   * @param x_traj Output vector for state trajectory.
   */
  virtual void getPredictedStateTrajectory(
      Eigen::Ref<Eigen::VectorXd> x_traj) const noexcept;

  /**
   * @brief Propagate the internal discrete-time model for one step.
   *
   *   Model must be set prior to calling this method.
   * @param x Current state.
   * @param u Current input.
   * @param x_next Output vector for next state.
   */
  void propagateModel(const Eigen::Ref<const Eigen::VectorXd>& x,
                      const Eigen::Ref<const Eigen::VectorXd>& u,
                      Eigen::Ref<Eigen::VectorXd> x_next) const;

  /**
   * @brief Set the internal discrete-time model directly from a discrete model.
   *   The model being `x_next = Ax + Bu + w`.
   * @param Ad Discrete-time state matrix A.
   * @param Bd Discrete-time input matrix B.
   * @param wd Discrete-time affine/bias vector w.
   * @return True if internal QP was updated properly. Unlikely to be false.
   */
  bool setModelDiscrete(const Eigen::Ref<const Eigen::MatrixXd>& Ad,
                        const Eigen::Ref<const Eigen::MatrixXd>& Bd,
                        const Eigen::Ref<const Eigen::VectorXd>& wd);

  /**
   * @brief Set the internal discrete-time model from a continuous-time model,
   *   which will be discretized. The model being `x_next = Ax + Bu + w`.
   *
   *   The discretization assumes the input u is constant over the time step
   *   (true for discrete controllers like MPC) and uses a matrix exponential,
   *   which is an exact discretization (theoretically). The matrix exponential
   *   involves a Taylor series expansion $\sum_{i=0}^\inf (A*dt)^i / i!$. Thus
   *   the scalar term is `dt^i / i!`. The infinite summation is stopped when
   *   this term becomes smaller than `tol`.
   * @param Ac Continuous-time state matrix A.
   * @param Bc Continuous-time input matrix B.
   * @param wc Continuous-time affine/bias term w.
   * @param dt Discretization time step in seconds. Usually should be much
   *   smaller than 1s for numeric stability reasons. This usually matches the
   *   control rate, and the input is held constant for this duration.
   * @param tol Tolerance for the matrix exponential. Taylor series expansion
   *   stops once scalar multiplier becomes smaller than this value.
   * @return True if internal QP was updated properly. Unlikely to be false.
   */
  bool setModelContinuous2Discrete(const Eigen::Ref<const Eigen::MatrixXd>& Ac,
                                   const Eigen::Ref<const Eigen::MatrixXd>& Bc,
                                   const Eigen::Ref<const Eigen::VectorXd>& wc,
                                   double dt,
                                   double tol = 1e-6);

  /**
   * @brief Set state and input weights for the cost function.
   *
   *   This method can only be called if `use_input_cost` is enabled.
   * @param Q_diag Vector for diagonal of state weight matrix (non-negative).
   * @param R_diag Vector for diagonal of input weight matrix (non-negative).
   */
  void setWeights(const Eigen::Ref<const Eigen::VectorXd>& Q_diag,
                  const Eigen::Ref<const Eigen::VectorXd>& R_diag);

  /**
   * @brief Set state, terminal state, and input weights for the cost function.
   *
   *   This method can only be called if `use_input_cost` is enabled.
   * @param Q_diag Vector for diagonal of state weight matrix (non-negative).
   * @param Qf_diag Vector for diagonal of terminal state weight matrix
   *   (non-negative).
   * @param R_diag Vector for diagonal of input weight matrix (non-negative).
   */
  void setWeights(const Eigen::Ref<const Eigen::VectorXd>& Q_diag,
                  const Eigen::Ref<const Eigen::VectorXd>& Qf_diag,
                  const Eigen::Ref<const Eigen::VectorXd>& R_diag);

  /**
   * @brief Set only the state weights for the cost function.
   * @param Q_diag Vector for diagonal of state weight matrix (non-negative).
   */
  void setStateWeights(const Eigen::Ref<const Eigen::VectorXd>& Q_diag);

  /**
   * @brief Set only the state and terminal state weights for the cost function.
   * @param Q_diag Vector for diagonal of state weight matrix (non-negative).
   * @param Qf_diag Vector for diagonal of terminal state weight matrix
   *   (non-negative).
   */
  void setStateWeights(const Eigen::Ref<const Eigen::VectorXd>& Q_diag,
                       const Eigen::Ref<const Eigen::VectorXd>& Qf_diag);

  /**
   * @brief Set only the input weights for the cost function.
   *
   *   This method can only be called if `use_input_cost` is enabled.
   * @param R_diag Vector for diagonal of input weight matrix (non-negative).
   */
  void setInputWeights(const Eigen::Ref<const Eigen::VectorXd>& R_diag);

  /**
   * @brief Set reference state trajectory as a step command.
   * @param x_step Reference state vector to use for entire trajectory.
   * @return True if internal QP was updated properly. Unlikely to be false.
   */
  bool setReferenceState(const Eigen::Ref<const Eigen::VectorXd>& x_step);

  /**
   * @brief Set reference state trajectory for tracking.
   * @param x_traj Reference state trajectory as a vector of stacked states.
   *   Should have length of state_dim * horizon_steps. If you have a matrix
   *   where each column is a state at index k in the horizon, then you can pass
   *   in `x_traj.reshaped()` to convert it to a vector.
   * @return True if internal QP was updated properly. Unlikely to be false.
   */
  bool
  setReferenceStateTrajectory(const Eigen::Ref<const Eigen::VectorXd>& x_traj);

  /**
   * @brief Set reference input trajectory as a step command (reference for all
   *   control points set to this value).
   *
   *   This method can only be called if `use_input_cost` is enabled.
   * @param u_step Reference input vector to use for entire trajectory.
   * @return True if internal QP was updated properly. Unlikely to be false.
   */
  bool setReferenceInput(const Eigen::Ref<const Eigen::VectorXd>& u_step);

  /**
   * @brief Set reference control points that parameterize the reference input
   *   trajectory.
   *
   *   This method can only be called if `use_input_cost` is enabled.
   * @param control_points Reference input control points as a stacked vector.
   *   Should have length of input_dim * num_control_points. If you have a
   *   matrix where each column is a control point, then you can pass in
   *   `control_points.reshaped()` to convert it to a vector.
   * @return True if internal QP was updated properly. Unlikely to be false.
   */
  bool setReferenceInputControlPoints(
      const Eigen::Ref<const Eigen::VectorXd>& control_points);

  /**
   * @brief Set input saturation limits.
   *
   *   This function must be called prior to `initializeSolver()`, but can also
   *   be called after if limits change between solves.
   * @param u_min Lower bound on input vector.
   * @param u_max Upper bound on input vector.
   * @return True if internal QP was updated properly. Unlikely to be false.
   */
  bool setInputLimits(const Eigen::Ref<const Eigen::VectorXd>& u_min,
                      const Eigen::Ref<const Eigen::VectorXd>& u_max);

  /**
   * @brief Set state saturation limits.
   *
   *    This function can only be called if `saturate_states` is enabled. Must
   *    be called prior to `initializeSolver()`, but can also be called after if
   *    limits change between solves.
   * @param x_min Lower bound on state vector.
   * @param x_max Upper bound on state vector.
   * @return True if internal QP was updated properly. Unlikely to be false.
   */
  bool setStateLimits(const Eigen::Ref<const Eigen::VectorXd>& x_min,
                      const Eigen::Ref<const Eigen::VectorXd>& x_max);

  /**
   * @brief Set slew-rate constraint limits for control points. Control points
   *   can vary by up to this magnitude from one index to the next.
   *
   *   This function can only be called if `slew_control_points` is enabled.
   *   Must be called prior to `initializeSolver()`, but can also be called
   *   after if limits change between solves.
   * @param control_point_slew Slew-rate vector.
   * @return True if internal QP was updated properly. Unlikely to be false.
   */
  bool setSlewRate(const Eigen::Ref<const Eigen::VectorXd>& control_point_slew);

  /**
   * @brief Set initial slew-rate constraint limits for control points. Initial
   *   input can vary by up to this magnitude from the previous input. See
   *   `setPreviousInput()`.
   *
   *   This function can only be called if `slew_initial_input` is enabled.
   *   Must be called prior to `initializeSolver()`, but can also be called
   *   after if limits change between solves.
   * @param u0_slew Initial slew-rate vector.
   * @return True if internal QP was updated properly. Unlikely to be false.
   */
  bool setSlewRateInitial(const Eigen::Ref<const Eigen::VectorXd>& u0_slew);

  /**
   * @brief Set the previous input, which is only used with the
   *   `slew_initial_input` constraint option. Defaults to zeros, but should be
   *   set prior to first solve if a different value is desired.
   *
   *   This value is automatically updated after each solve, so you really only
   *   need to call this function once before the first solve, or potentially if
   *   a solve fails and you want to set a custom strategy for handling it.
   *
   *   This function can only be called if `slew_initial_input` is enabled.
   *   Must be called prior to `initializeSolver()`, but can also be called
   *   after if limits change between solves.
   * @param u_prev Previous input vector.
   * @return True if previous input is valid and set.
   */
  bool setPreviousInput(const Eigen::Ref<const Eigen::VectorXd>& u_prev);

  /// @brief Get state dimension.
  constexpr int getStateDim() const noexcept { return state_dim_; };
  /// @brief Get input dimension.
  constexpr int getInputDim() const noexcept { return input_dim_; };
  /// @brief Get number of horizon steps.
  constexpr int getHorizonSteps() const noexcept { return horizon_steps_; };
  /// @brief Get number of control points.
  constexpr int getNumControlPoints() const noexcept { return num_ctrl_pts_; };

protected:
  const int state_dim_, input_dim_;
  const int horizon_steps_, num_ctrl_pts_, spline_degree_;
  const int x_traj_dim_, u_traj_dim_, ctrls_dim_;
  const Options opts_;
  const int num_u_sat_cons_, u_sat_dim_, slew_dim_, x_sat_dim_;
  const int u_sat_idx_, slew0_idx_, slew_idx_, x_sat_idx_;
  bool model_set_, u_lims_set_, x_lims_set_;
  bool slew0_rate_set_, ctrls_slew_rate_set_;
  bool solver_initialized_;
  bool weights_changed_;

  // MPC variables
  Eigen::MatrixXd Ad_, Bd_;
  Eigen::VectorXd wd_;

  // Working matrices for model discretization
  Eigen::MatrixXd G_, At_, At_pow_;

  Eigen::DiagonalMatrix<double, Eigen::Dynamic> Q_big_, R_big_;
  Eigen::VectorXd x_ref_, ctrls_ref_, u_min_, u_max_, x_min_, x_max_;
  Eigen::VectorXd ctrls_slew_, u0_slew_, u_prev_;
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
  void evaluateControlPoints(const Eigen::Ref<const Eigen::VectorXd>& ctrl_pts,
                             Eigen::Ref<Eigen::VectorXd> u_traj) const noexcept;
  void getInput(int k,
                const Eigen::Ref<const Eigen::VectorXd>& ctrl_pts,
                Eigen::Ref<Eigen::VectorXd> uk) const noexcept;

  // need to be implemented by derived classes (they define how to update the
  // QP matrices based on changes to the MPC problem)
  virtual void qpUpdateX0(const Eigen::Ref<const Eigen::VectorXd>& x0) = 0;
  virtual bool qpUpdateModel() = 0;
  virtual bool qpUpdateReferences() = 0;
  virtual bool qpUpdateInputLimits() = 0;
  virtual bool qpUpdateStateLimits() = 0;
  virtual bool qpUpdateSlewRate() = 0;
};

} // namespace affine_mpc

#endif // AFFINE_MPC_MPC_BASE_HPP
