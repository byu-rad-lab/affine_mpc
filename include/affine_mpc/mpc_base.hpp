#ifndef AFFINE_MPC_MPC_BASE_HPP
#define AFFINE_MPC_MPC_BASE_HPP

/**
 * @file mpc_base.hpp
 * @brief Defines the abstract MPCBase class.
 */

#include <Eigen/Core>
#include <iosfwd>
#include <memory>
#include <vector>

#include "affine_mpc/discretizer.hpp"
#include "affine_mpc/options.hpp"
#include "affine_mpc/osqp_solver.hpp"
#include "affine_mpc/parameterization.hpp"
#include "affine_mpc/problem_data.hpp"
#include "affine_mpc/solve_status.hpp"
#include "affine_mpc/vector_parameterization.hpp"

namespace affine_mpc {

/**
 * @class MPCBase
 * @brief Abstract base class for affine MPC formulations.
 *
 * Defines the common interface and shared logic for condensed and sparse MPC
 * variants. Manages OSQP solver, input parameterization via B-splines, and
 * runtime configuration.
 *
 * Setters only validate and store their values and mark a change category
 * (model offset, model matrices, weights, references, limits, previous input).
 * All QP work happens in solve(), or earlier in the optional prepare(), once
 * per changed category. Changes of the model matrices or the weights rebuild QP
 * matrices and make OSQP refactor; the other categories only update vectors.
 *
 * Derived classes read the problem data through data() and implement how the
 * QP depends on each change category and on the initial state.
 */
class MPCBase
{
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
   *
   *   The setters only store their values, so solve() first applies every
   *   change since the last solve() or prepare() to the QP (rebuilding QP
   *   matrices and refactoring for model or weight changes), then the terms
   *   that depend on x0, then runs the solver.
   * @param x0 Initial (current) state vector.
   * @throws std::invalid_argument if x0 does not have size state_dim.
   * @return SolveStatus Result indication. Generally expected to be `Success`
   *   unless the solver has not been initialized, then `NotInitialized`.
   *   `UpdateFailed` means OSQP rejected a data update (often from non-finite
   *   model or weight values), so the QP was not solved and the getters still
   *   return the previous solution. The update is retried at the next solve,
   *   which usually recovers once the values are valid again (infinite weights
   *   can leave OSQP unable to recover).
   *   Verify your problem setup and consult OSQP documentation for any other
   *   value.
   */
  [[nodiscard]] SolveStatus solve(const Eigen::Ref<const Eigen::VectorXd>& x0);

  /**
   * @brief Optional: apply every pending change that does not depend on the
   *   initial state, so the next solve() only does the x0-dependent work and
   *   the solver iterations.
   *
   *   Setters only store their values; the QP work happens in solve(). For
   *   changes of the model matrices or the weights, that work (rebuilding QP
   *   matrices and refactoring) is most of the cost of a solve. Calling
   *   prepare() after these setters and before the next state measurement
   *   (e.g. right after applying the previous input) moves that work out of
   *   the time between measurement and input. Calling it is never required:
   *   solve() applies anything still pending. It saves little when only the
   *   model offset, references, limits, or the previous input changed (they
   *   only update QP vectors), and has nothing to do when nothing changed
   *   (e.g. an LTI problem with fixed weights).
   *
   *   Its duration is added to the `prepare_time` that the next solve()
   *   reports in getSolveInfo().
   * @return `Success` if the pending changes were applied, `NotInitialized`
   *   before initializeSolver(), or `UpdateFailed` if OSQP rejected a data
   *   update (the update stays pending, so the next solve() retries it and
   *   reports `UpdateFailed` again while the data is invalid).
   */
  [[nodiscard]] SolveStatus prepare();

  /**
   * @brief Get the next input to apply (initial input from optimized
   *   trajectory) from the previous solve.
   *
   *   Precondition: initializeSolver() succeeded (not checked in Release).
   * @param u0 Output vector for input.
   * @throws std::invalid_argument if u0 does not have size
   *   input_dim.
   */
  void getNextInput(Eigen::Ref<Eigen::VectorXd> u0) const;

  /**
   * @brief Get the control points that parameterize the input trajectory from
   *   the previous solve.
   *
   *   Precondition: initializeSolver() succeeded (not checked in Release).
   * @param control_points Output vector for stacked control points.
   * @throws std::invalid_argument if control_points does not have size
   *   input_dim * num_control_points.
   */
  void getInputControlPoints(Eigen::Ref<Eigen::VectorXd> control_points) const;

  /**
   * @brief Get the full input trajectory from the previous solve.
   *
   *   Precondition: initializeSolver() succeeded (not checked in Release).
   * @param u_traj Output vector for input trajectory.
   * @throws std::invalid_argument if u_traj does not have size
   *   input_dim * horizon_steps.
   */
  void getInputTrajectory(Eigen::Ref<Eigen::VectorXd> u_traj) const;

  /**
   * @brief Get the predicted state trajectory from the previous solve.
   *
   *   Precondition: initializeSolver() succeeded (not checked in Release).
   *   CondensedMPC computes it from the model, so after a prepare() that
   *   applied a model change, call it before prepare() to get the prediction
   *   of the previous solve.
   * @param x_traj Output vector for state trajectory.
   * @throws std::invalid_argument if x_traj does not have size
   *   state_dim * horizon_steps.
   */
  virtual void
  getPredictedStateTrajectory(Eigen::Ref<Eigen::VectorXd> x_traj) const;

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
   * @param wd Discrete-time model offset w.
   * @note The QP matrices are rebuilt (and the solver refactors) in the next
   *   prepare() or solve(), which report update failures.
   */
  void setModelDiscrete(const Eigen::Ref<const Eigen::MatrixXd>& Ad,
                        const Eigen::Ref<const Eigen::MatrixXd>& Bd,
                        const Eigen::Ref<const Eigen::VectorXd>& wd);

  /**
   * @brief Set the internal discrete-time model from a continuous-time model,
   *   which will be discretized. The model being `x_next = Ax + Bu + w`.
   *
   *   The discretization assumes the input u is constant over the time step
   *   (true for discrete controllers like MPC) and uses a matrix exponential,
   *   which is an exact discretization (theoretically). See Discretizer, which
   *   computes it and can also be used on its own.
   *
   *   The discretization runs inside this setter, so its cost is not part of
   *   solve() or prepare() (time it as part of your control loop). The QP
   *   matrices are rebuilt in the next prepare() or solve(). When only the
   *   offset changes, setModelOffsetContinuous2Discrete() is much cheaper.
   * @param Ac Continuous-time state matrix A.
   * @param Bc Continuous-time input matrix B.
   * @param wc Continuous-time model offset w.
   * @param dt Discretization time step in seconds (positive). This usually
   *   matches the control rate, and the input is held constant for this
   *   duration.
   * @param tol Truncation tolerance for the Taylor series (positive). The
   *   actual error in the discrete model is typically two to three orders of
   *   magnitude smaller than `tol`. The default gives results close to machine
   *   precision. Larger values (e.g., 1e-6) save a few matrix products, which
   *   can help when the affine approximation is updated every time step.
   * @note QP update failures are reported by the next prepare() or
   *   solve().
   */
  void setModelContinuous2Discrete(const Eigen::Ref<const Eigen::MatrixXd>& Ac,
                                   const Eigen::Ref<const Eigen::MatrixXd>& Bc,
                                   const Eigen::Ref<const Eigen::VectorXd>& wc,
                                   double dt,
                                   double tol = 1e-10);

  /**
   * @brief Set only the model offset `wd` of the discrete-time model
   *   `x_next = Ad x + Bd u + wd`, keeping `Ad` and `Bd`.
   *
   *   Much cheaper than setModelDiscrete(): the offset only enters vectors of
   *   the QP, so the next solve() does not rebuild the QP matrices and the
   *   solver does not refactor. Use it when only the offset changes between
   * solves, e.g. a disturbance estimate for offset-free tracking or a measured
   * exogenous input.
   * @param wd Discrete-time model offset.
   * @throws std::invalid_argument if wd does not have size state_dim.
   * @throws std::logic_error if the model has not been set.
   */
  void setModelOffsetDiscrete(const Eigen::Ref<const Eigen::VectorXd>& wd);

  /**
   * @brief Set only the model offset from a continuous-time offset `wc`,
   *   discretized with the state matrix and time step of the last
   *   setModelContinuous2Discrete() call (`wd = G wc`, see Discretizer).
   *
   *   Costs one matrix-vector product (no matrix exponential), and like
   *   setModelOffsetDiscrete() does not cause a QP matrix rebuild.
   * @param wc Continuous-time model offset.
   * @throws std::invalid_argument if wc does not have size state_dim.
   * @throws std::logic_error if the model was not last set with
   *   setModelContinuous2Discrete().
   */
  void setModelOffsetContinuous2Discrete(
      const Eigen::Ref<const Eigen::VectorXd>& wc);

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
   *
   *   The terminal state weights are also set to `Q_diag`.
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
   * @note The QP is updated in the next prepare() or solve(), which report
   *   update failures.
   */
  void setReferenceState(const Eigen::Ref<const Eigen::VectorXd>& x_step);

  /**
   * @brief Set reference state trajectory for tracking.
   * @param x_traj Reference state trajectory as a vector of stacked states.
   *   Should have length of state_dim * horizon_steps. If you have a matrix
   *   where each column is a state at index k in the horizon, then you can pass
   *   in `x_traj.reshaped()` to convert it to a vector.
   * @note The QP is updated in the next prepare() or solve(), which report
   *   update failures.
   */
  void
  setReferenceStateTrajectory(const Eigen::Ref<const Eigen::VectorXd>& x_traj);

  /**
   * @brief Set reference input trajectory as a step command (reference for all
   *   control points set to this value).
   *
   *   This method can only be called if `use_input_cost` is enabled.
   * @param u_step Reference input vector to use for entire trajectory.
   * @note The QP is updated in the next prepare() or solve(), which report
   *   update failures.
   */
  void setReferenceInput(const Eigen::Ref<const Eigen::VectorXd>& u_step);

  /**
   * @brief Set reference control points that parameterize the reference input
   *   trajectory.
   *
   *   This method can only be called if `use_input_cost` is enabled.
   * @param control_points Reference input control points as a stacked vector.
   *   Should have length of input_dim * num_control_points. If you have a
   *   matrix where each column is a control point, then you can pass in
   *   `control_points.reshaped()` to convert it to a vector.
   * @note The QP is updated in the next prepare() or solve(), which report
   *   update failures.
   */
  void setReferenceInputControlPoints(
      const Eigen::Ref<const Eigen::VectorXd>& control_points);

  /**
   * @brief Set input saturation limits.
   *
   *   This function must be called prior to `initializeSolver()`, but can also
   *   be called after if limits change between solves.
   * @param u_min Lower bound on input vector.
   * @param u_max Upper bound on input vector.
   * @note The QP is updated in the next prepare() or solve(), which report
   *   update failures.
   */
  void setInputLimits(const Eigen::Ref<const Eigen::VectorXd>& u_min,
                      const Eigen::Ref<const Eigen::VectorXd>& u_max);

  /**
   * @brief Set state saturation limits.
   *
   *    This function can only be called if `saturate_states` is enabled. Must
   *    be called prior to `initializeSolver()`, but can also be called after if
   *    limits change between solves.
   * @param x_min Lower bound on state vector.
   * @param x_max Upper bound on state vector.
   * @note The QP is updated in the next prepare() or solve(), which report
   *   update failures.
   */
  void setStateLimits(const Eigen::Ref<const Eigen::VectorXd>& x_min,
                      const Eigen::Ref<const Eigen::VectorXd>& x_max);

  /**
   * @brief Set slew-rate constraint limits for control points. Control points
   *   can vary by up to this magnitude from one index to the next.
   *
   *   This function can only be called if `slew_control_points` is enabled.
   *   Must be called prior to `initializeSolver()`, but can also be called
   *   after if limits change between solves.
   * @param control_point_slew Slew-rate vector.
   * @note The QP is updated in the next prepare() or solve(), which report
   *   update failures.
   */
  void setSlewRate(const Eigen::Ref<const Eigen::VectorXd>& control_point_slew);

  /**
   * @brief Set initial slew-rate constraint limits for control points. Initial
   *   input can vary by up to this magnitude from the previous input. See
   *   `setPreviousInput()`.
   *
   *   This function can only be called if `slew_initial_input` is enabled.
   *   Must be called prior to `initializeSolver()`, but can also be called
   *   after if limits change between solves.
   * @param u0_slew Initial slew-rate vector.
   * @note The QP is updated in the next prepare() or solve(), which report
   *   update failures.
   */
  void setSlewRateInitial(const Eigen::Ref<const Eigen::VectorXd>& u0_slew);

  /**
   * @brief Set the previous input, which is only used with the
   *   `slew_initial_input` constraint option. Defaults to zeros, but should be
   *   set prior to first solve if a different value is desired.
   *
   *   This value is automatically set to the next input after each solve that
   *   returns `Success` or `SolvedInaccurate`, so you really only need to call
   *   this function once before the first solve, or after a failed solve (which
   *   leaves it unchanged) if you apply an input other than the stored one.
   *
   *   This function can only be called if `slew_initial_input` is enabled.
   *   Must be called prior to `initializeSolver()`, but can also be called
   *   after if limits change between solves.
   * @param u_prev Previous input vector.
   * @note The QP is updated in the next prepare() or solve().
   */
  void setPreviousInput(const Eigen::Ref<const Eigen::VectorXd>& u_prev);

  /// @brief Get state dimension.
  constexpr int getStateDim() const noexcept { return state_dim_; };
  /// @brief Get input dimension.
  constexpr int getInputDim() const noexcept { return input_dim_; };
  /// @brief Get number of horizon steps.
  constexpr int getHorizonSteps() const noexcept { return horizon_steps_; };
  /// @brief Get number of control points.
  constexpr int getNumControlPoints() const noexcept { return num_ctrl_pts_; };
  /// @brief Whether initializeSolver() has succeeded.
  bool isSolverInitialized() const noexcept { return solver_initialized_; }

  /// @brief Get the input trajectory parameterization.
  const Parameterization& getParameterization() const noexcept
  {
    return input_param_.parameterization();
  }

  /// @brief Get the options enabled at construction.
  const Options& getOptions() const noexcept { return opts_; }

  /**
   * @brief Get the diagonal of the state weight matrix Q.
   * @return Vector of size state_dim.
   */
  Eigen::VectorXd getStateWeights() const;

  /**
   * @brief Get the diagonal of the terminal state weight matrix Q_f.
   * @return Vector of size state_dim.
   */
  Eigen::VectorXd getTerminalStateWeights() const;

  /**
   * @brief Get the diagonal of the input weight matrix R.
   * @throws std::logic_error if `use_input_cost` is not enabled.
   * @return Vector of size input_dim.
   */
  Eigen::VectorXd getInputWeights() const;

  /// @brief Get the lower input limit (-inf until setInputLimits()).
  const Eigen::VectorXd& getInputMin() const noexcept { return data_.u_min; }
  /// @brief Get the upper input limit (+inf until setInputLimits()).
  const Eigen::VectorXd& getInputMax() const noexcept { return data_.u_max; }

  /**
   * @brief Get the lower state limit (-inf until setStateLimits()).
   * @throws std::logic_error if `saturate_states` is not enabled.
   */
  const Eigen::VectorXd& getStateMin() const;

  /**
   * @brief Get the upper state limit (+inf until setStateLimits()).
   * @throws std::logic_error if `saturate_states` is not enabled.
   */
  const Eigen::VectorXd& getStateMax() const;

  /**
   * @brief Get the control point slew rate (+inf until setSlewRate()).
   * @throws std::logic_error if `slew_control_points` is not enabled.
   */
  const Eigen::VectorXd& getSlewRate() const;

  /**
   * @brief Get the initial input slew rate (+inf until setSlewRateInitial()).
   * @throws std::logic_error if `slew_initial_input` is not enabled.
   */
  const Eigen::VectorXd& getSlewRateInitial() const;

  /**
   * @brief Get the previous input used by the initial slew rate constraint.
   *   Updated automatically after each solve that returns `Success` or
   *   `SolvedInaccurate`.
   * @throws std::logic_error if `slew_initial_input` is not enabled.
   */
  const Eigen::VectorXd& getPreviousInput() const;

  /**
   * @brief Get the reference state trajectory.
   * @return Stacked states of size state_dim * horizon_steps.
   */
  const Eigen::VectorXd& getReferenceStateTrajectory() const noexcept
  {
    return data_.x_ref;
  }

  /**
   * @brief Get the reference input control points.
   * @throws std::logic_error if `use_input_cost` is not enabled.
   * @return Stacked control points of size input_dim * num_control_points.
   */
  const Eigen::VectorXd& getReferenceInputControlPoints() const;

  /**
   * @brief Get solver diagnostics from the last solve() (status, iterations,
   *   objective, and timing).
   *
   *   After a solve() that returned `UpdateFailed`, the solver fields still
   *   describe the previous solve, while `mpc_solve_time` covers the failed
   *   call. `prepare_time` covers the prepare() calls between the previous
   *   solve() and the last one.
   * @return SolveInfo with default values (status `NotInitialized`) before
   *   initializeSolver().
   */
  SolveInfo getSolveInfo() const noexcept
  {
    SolveInfo info{solver_->getSolveInfo()};
    info.mpc_solve_time = mpc_solve_time_;
    info.prepare_time = prepare_time_;
    return info;
  }

  /**
   * @brief Make the next solve() start from zero instead of warm starting
   *   from the previous solution (an OSQP cold start).
   *
   *   Useful after a failed solve, whose iterates may have diverged and would
   *   slow down or prevent convergence of the next solve. Does not change the
   *   problem data, the previous input used by `slew_initial_input`, or the
   *   results returned by the solution getters. No-op before
   *   initializeSolver().
   */
  void resetWarmStart() noexcept { solver_->coldStart(); }

  /**
   * @brief Get the QP cost matrix P (the Hessian of the QP objective), for
   *   analysis and debugging.
   *
   *   Reflects the matrix used by the last solve() or prepare(): setters do
   *   not change it, model or weight changes are applied by the next prepare()
   *   or solve(), and SparseMPC holds unit placeholder weights between
   *   initializeSolver() and the first prepare() or solve(). The return type
   *   may change if the internal QP storage changes.
   * @return Reference to the internal QP cost matrix (full symmetric matrix),
   *   valid for the lifetime of this object.
   */
  const Eigen::MatrixXd& getQPCostMatrix() const noexcept { return P_; }

  /**
   * @brief Debugging aid: check whether the current QP matrices fit the
   *   sparsity pattern fixed by initializeSolver().
   *
   *   Model or weight updates that add nonzeros outside that pattern are
   *   silently dropped by the solver, so the MPC solves a different problem
   *   than the one you set. Call this after prepare() or solve() (setters do
   *   not update the QP) to detect that. It scans every entry of the
   *   QP matrices, so avoid calling it in time-critical loops.
   * @return True if the QP matrices fit the pattern, or if the solver is not
   *   initialized yet. False otherwise.
   */
  [[nodiscard]] bool isWithinSparsityPattern() const;

  /// @brief Name of the MPC class (e.g. "CondensedMPC"), as printed by print().
  virtual const char* getClassName() const noexcept { return "MPCBase"; }

protected:
  const int state_dim_, input_dim_;
  const int horizon_steps_, num_ctrl_pts_;
  const int x_traj_dim_, u_traj_dim_, ctrls_dim_;
  const Options opts_;
  /// Maps the control points (first in the QP solution) to the inputs
  const VectorParameterization input_param_;
  /// Sample indices with input saturation rows (empty: control points instead)
  const std::vector<int> u_sat_samples_;
  const int num_u_sat_cons_, u_sat_dim_, slew_dim_, x_sat_dim_;
  const int u_sat_idx_, slew0_idx_, slew_idx_, x_sat_idx_;
  Eigen::Map<const Eigen::VectorXd> solution_map_;

  // OSQP variables - see https://osqp.org/docs/solver/index.html
  std::unique_ptr<OSQPSolver> solver_;
  Eigen::MatrixXd P_; // osqp cost matrix
  Eigen::MatrixXd A_; // osqp constraint matrix
  Eigen::VectorXd q_; // osqp cost vector
  // osqp constraint bounds: MPCBase writes the input saturation, slew, and
  // state saturation rows; formulations write their own rows
  Eigen::VectorXd l_; // osqp constraint lower bound
  Eigen::VectorXd u_; // osqp constraint upper bound

  /// @brief The problem data set through the setters (read-only).
  const ProblemData& data() const noexcept { return data_; }

  /// @brief Change categories not yet applied to the QP.
  ChangeSet pendingChanges() const noexcept { return pending_; }

  /**
   * @brief Bring the whole QP up to date for x0: apply the pending changes,
   *   then the x0-dependent terms, and push them to the solver. Before
   *   initializeSolver() this only assembles the QP.
   * @return False if a solver update failed (the failed changes stay pending).
   */
  bool qpUpdateX0(const Eigen::Ref<const Eigen::VectorXd>& x0);

private:
  // Apply the pending changes (x0-independent work). Returns false if a solver
  // update failed, in which case the failed categories stay pending.
  bool applyPendingChanges();
  // Write the bound rows owned by MPCBase for the given changes
  void writeBounds(ChangeSet changes);

  // need to be implemented by derived classes (they define how the QP depends
  // on the problem data)

  // Write the parts of the QP that depend on the changed categories but not on
  // x0 (e.g. P and A after model or weight changes), and push the changed
  // matrices to the solver (and any changed vectors that qpApplyX0() does not
  // push anyway).
  // MPCBase has already written its own bound rows. Before initializeSolver()
  // the QP is only assembled. Returns the categories that are still pending:
  // those whose solver update failed (retried at the next call), plus any the
  // formulation applies only after initialization.
  virtual ChangeSet qpApplyChanges(ChangeSet changes) = 0;
  // Write the x0-dependent parts of the QP and push them to the solver.
  // Returns false if a solver update failed. Must not do matrix work, so that
  // solve() after prepare() only updates vectors.
  virtual bool qpApplyX0(const Eigen::Ref<const Eigen::VectorXd>& x0) = 0;

  ProblemData data_;
  ChangeSet pending_; // categories changed since they were applied to the QP
  bool model_set_, model_from_continuous_, u_lims_set_, x_lims_set_;
  bool slew0_rate_set_, ctrls_slew_rate_set_;
  bool solver_initialized_;
  double mpc_solve_time_; ///< duration of the last solve() call (seconds)
  double prepare_time_;   ///< prepare() time reported by the last solve() (s)
  double unsolved_prepare_time_; ///< prepare() time since the last solve() (s)
  Discretizer discretizer_;      // for setModelContinuous2Discrete()
};

/**
 * @brief Stream a detailed multiline summary of the MPC configuration.
 * @param os Output stream.
 * @param mpc MPC object to print.
 * @return Reference to the output stream.
 */
std::ostream& operator<<(std::ostream& os, const MPCBase& mpc);

/**
 * @brief Stream a detailed multiline summary of the MPC configuration.
 * @param os Output stream.
 * @param mpc MPC object to print.
 * @param capitalize_bools If true, print booleans as `True`/`False`.
 * @return Reference to the output stream.
 */
std::ostream&
print(std::ostream& os, const MPCBase& mpc, bool capitalize_bools = false);

/**
 * @brief Stream a compact one-line summary of the MPC configuration.
 * @param os Output stream.
 * @param mpc MPC object to print.
 * @param capitalize_bools If true, print booleans as `True`/`False`.
 * @return Reference to the output stream.
 */
std::ostream& printInline(std::ostream& os,
                          const MPCBase& mpc,
                          bool capitalize_bools = false);

} // namespace affine_mpc

#endif // AFFINE_MPC_MPC_BASE_HPP
