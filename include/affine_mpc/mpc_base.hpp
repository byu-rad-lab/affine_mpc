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
   * @param parameterization Input parameterization (B-spline, etc.).
   * @param opts Optional features to enable.
   * @param num_design_vars Number of QP decision variables.
   * @param num_custom_constraints Number of custom constraints (other than
   *   input saturation, slew rate constraints, and state saturation).
   */
  MPCBase(const int state_dim,
          const int input_dim,
          const Parameterization& parameterization,
          const Options& opts,
          const int num_design_vars,
          const int num_custom_constraints);

public:
  virtual ~MPCBase() = default;

  /**
   * @brief Initialize the OSQP solver with current model and constraints.
   * @param solver_settings OSQP solver settings (optional).
   * @return True if initialization succeeds, false otherwise.
   */
  [[nodiscard]] bool initializeSolver(const OSQPSettings& solver_settings =
                                          OSQPSolver::getRecommendedSettings());

  /**
   * @brief Solve the MPC QP for the given initial state.
   * @param x0 Initial state vector.
   * @return SolveStatus indicating result.
   */
  [[nodiscard]] SolveStatus solve(const Eigen::Ref<const Eigen::VectorXd>& x0);

  /**
   * @brief Get the next input to apply (first step of optimized trajectory).
   * @param u0 Output vector for input.
   */
  void getNextInput(Eigen::Ref<Eigen::VectorXd> u0) const noexcept;

  /**
   * @brief Get the optimized control points for the parameterized input
   *   trajectory.
   * @param u_traj_ctrl_pts Output vector for control points.
   */
  void getParameterizedInputTrajectory(
      Eigen::Ref<Eigen::VectorXd> u_traj_ctrl_pts) const noexcept;

  /**
   * @brief Get the full input trajectory over the horizon (after
   *   parameterization).
   * @param u_traj Output vector for input trajectory.
   */
  void getInputTrajectory(Eigen::Ref<Eigen::VectorXd> u_traj) const noexcept;

  /**
   * @brief Get the predicted state trajectory over the horizon.
   * @param x_traj Output vector for state trajectory.
   */
  virtual void getPredictedStateTrajectory(
      Eigen::Ref<Eigen::VectorXd> x_traj) const noexcept;

  /**
   * @brief Propagate the discrete-time model for one step.
   * @param x Current state.
   * @param u Current input.
   * @param x_next Output vector for next state.
   */
  void propagateModel(const Eigen::Ref<const Eigen::VectorXd>& x,
                      const Eigen::Ref<const Eigen::VectorXd>& u,
                      Eigen::Ref<Eigen::VectorXd> x_next) const;

  /**
   * @brief Set the discrete-time model matrices.
   * @param Ad Discrete-time state matrix.
   * @param Bd Discrete-time input matrix.
   * @param wd Discrete-time affine/bias term.
   * @return True if model is valid and set.
   */
  bool setModelDiscrete(const Eigen::Ref<const Eigen::MatrixXd>& Ad,
                        const Eigen::Ref<const Eigen::MatrixXd>& Bd,
                        const Eigen::Ref<const Eigen::VectorXd>& wd);

  /**
   * @brief Set the model by discretizing continuous-time matrices.
   * @param Ac Continuous-time state matrix.
   * @param Bc Continuous-time input matrix.
   * @param wc Continuous-time affine/bias term.
   * @param dt Discretization timestep.
   * @param tol Tolerance for matrix exponential (stop Taylor series expansion
   *   when the scalar multiplier is less than this value).
   * @return True if model is valid and set.
   */
  bool setModelContinuous2Discrete(const Eigen::Ref<const Eigen::MatrixXd>& Ac,
                                   const Eigen::Ref<const Eigen::MatrixXd>& Bc,
                                   const Eigen::Ref<const Eigen::VectorXd>& wc,
                                   const double dt,
                                   const double tol = 1e-6);

  /**
   * @brief Set state and input weights for the cost function.
   * @param Q_diag Vector for diagonal of state weight matrix.
   * @param R_diag Vector for diagonal of input weight matrix.
   */
  void setWeights(const Eigen::Ref<const Eigen::VectorXd>& Q_diag,
                  const Eigen::Ref<const Eigen::VectorXd>& R_diag);

  /**
   * @brief Set state, terminal state, and input weights for the cost function.
   * @param Q_diag Vector for diagonal of state weight matrix.
   * @param Qf_diag Vector for diagonal of terminal state weight matrix.
   * @param R_diag Vector for diagonal of input weight matrix.
   */
  void setWeights(const Eigen::Ref<const Eigen::VectorXd>& Q_diag,
                  const Eigen::Ref<const Eigen::VectorXd>& Qf_diag,
                  const Eigen::Ref<const Eigen::VectorXd>& R_diag);

  /**
   * @brief Set state weights only.
   * @param Q_diag Vector for diagonal of state weight matrix.
   */
  void setStateWeights(const Eigen::Ref<const Eigen::VectorXd>& Q_diag);

  /**
   * @brief Set state and terminal state weights only.
   * @param Q_diag Vector for diagonal of state weight matrix.
   * @param Qf_diag Vector for diagonal of terminal state weight matrix.
   */
  void setStateWeights(const Eigen::Ref<const Eigen::VectorXd>& Q_diag,
                       const Eigen::Ref<const Eigen::VectorXd>& Qf_diag);

  /**
   * @brief Set input weights only.
   * @param R_diag Vector for diagonal of input weight matrix.
   */
  void setInputWeights(const Eigen::Ref<const Eigen::VectorXd>& R_diag);

  /**
   * @brief Set reference state for tracking.
   * @param x_step Reference state vector.
   * @return True if reference is valid and set.
   */
  bool setReferenceState(const Eigen::Ref<const Eigen::VectorXd>& x_step);

  /**
   * @brief Set reference state trajectory for tracking.
   * @param x_traj Reference state trajectory vector.
   * @return True if reference is valid and set.
   */
  bool
  setReferenceStateTrajectory(const Eigen::Ref<const Eigen::VectorXd>& x_traj);

  /**
   * @brief Set reference input for tracking.
   * @param u_step Reference input vector.
   * @return True if reference is valid and set.
   */
  bool setReferenceInput(const Eigen::Ref<const Eigen::VectorXd>& u_step);

  /**
   * @brief Set reference parameterized input trajectory (control points).
   * @param u_traj_ctrl_pts Reference control points vector.
   * @return True if reference is valid and set.
   */
  bool setReferenceParameterizedInputTrajectory(
      const Eigen::Ref<const Eigen::VectorXd>& u_traj_ctrl_pts);

  /**
   * @brief Set input box constraints.
   * @param u_min Input lower bounds.
   * @param u_max Input upper bounds.
   * @return True if limits are valid and set.
   */
  bool setInputLimits(const Eigen::Ref<const Eigen::VectorXd>& u_min,
                      const Eigen::Ref<const Eigen::VectorXd>& u_max);

  /**
   * @brief Set state box constraints.
   * @param x_min State lower bounds.
   * @param x_max State upper bounds.
   * @return True if limits are valid and set.
   */
  bool setStateLimits(const Eigen::Ref<const Eigen::VectorXd>& x_min,
                      const Eigen::Ref<const Eigen::VectorXd>& x_max);

  /**
   * @brief Set slew-rate constraint for control points.
   * @param u_slew Slew-rate vector.
   * @return True if constraint is valid and set.
   */
  bool setSlewRate(const Eigen::Ref<const Eigen::VectorXd>& u_slew);

  /**
   * @brief Set initial slew-rate constraint (reserved/future use).
   * @param u0_slew Initial slew-rate vector.
   * @return True if constraint is valid and set.
   */
  bool setSlewRateInitial(const Eigen::Ref<const Eigen::VectorXd>& u0_slew);

  /**
   * @brief Set previous input for slew-rate constraints.
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
  // const bool use_input_cost_, use_slew_rate_, saturate_states_;
  const Options opts_;
  const int num_u_sat_cons_, u_sat_dim_, slew_dim_, x_sat_dim_;
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
