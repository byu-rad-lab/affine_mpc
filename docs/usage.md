# Usage

This page shows the intended workflow for both the C++ and Python interfaces.
The solver concepts and setup order are the same in both languages; the tabs below only change the interface syntax.

For the shared mathematical background, see [Concepts](concepts.md).

## Main Types

=== "Python"

    The most important names exported by `affine_mpc` are:

    - `Parameterization`
    - `Options`
    - `OSQPSettings`
    - `CondensedMPC`
    - `SparseMPC`
    - `MPCLogger`
    - `SolveStatus`

=== "C++"

    Public headers are in `include/affine_mpc/`.

    The most important types are:

    - `affine_mpc::Parameterization`
    - `affine_mpc::Options`
    - `affine_mpc::MPCBase`
    - `affine_mpc::CondensedMPC`
    - `affine_mpc::SparseMPC`
    - `affine_mpc::MPCLogger`
    - `affine_mpc::SolveStatus`

## Typical Workflow

### 1. Choose a parameterization

Other common choices are `moveBlocking(...)` and `bspline(...)`.

=== "Python"

    ```python
    param = ampc.Parameterization.linearInterp(horizon_steps=T, num_control_points=nc)
    ```

=== "C++"

    ```cpp
    auto param = affine_mpc::Parameterization::linearInterp(horizon_steps,
                                                            num_control_points);
    ```

### 2. Configure options if needed

=== "Python"

    ```python
    opts = ampc.Options(
        use_input_cost=True,
        slew_initial_input=False,
        slew_control_points=True,
        saturate_states=False,
        saturate_input_trajectory=False,
    )
    ```

=== "C++"

    ```cpp
    affine_mpc::Options opts;
    opts.use_input_cost = true;
    opts.slew_initial_input = false;
    opts.slew_control_points = true;
    opts.saturate_states = false;
    opts.saturate_input_trajectory = false;
    ```

### 3. Construct the MPC object

`CondensedMPC` is usually the best starting point.
`SparseMPC` can be worth trying for larger or more structured problems.

=== "Python"

    ```python
    mpc = ampc.CondensedMPC(state_dim=n, input_dim=m, param=param, opts=opts)
    ```

    Or:

    ```python
    mpc = ampc.SparseMPC(state_dim=n, input_dim=m, param=param, opts=opts)
    ```

=== "C++"

    ```cpp
    affine_mpc::CondensedMPC mpc(state_dim, input_dim, param, opts);
    ```

    Or:

    ```cpp
    affine_mpc::SparseMPC mpc(state_dim, input_dim, param, opts);
    ```

### 4. Set the model

Either provide a discrete model directly or discretize from continuous time.

=== "Python"

    ```python
    mpc.setModelDiscrete(Ad, Bd, wd)
    mpc.setModelContinuous2Discrete(Ac, Bc, wc, dt)
    ```

=== "C++"

    ```cpp
    mpc.setModelDiscrete(Ad, Bd, wd);
    mpc.setModelContinuous2Discrete(Ac, Bc, wc, dt);
    ```

### 5. Set limits and enabled optional constraints

=== "Python"

    ```python
    mpc.setInputLimits(u_min, u_max)

    if opts.slew_initial_input:
        mpc.setSlewRateInitial(u0_slew)
        mpc.setPreviousInput(u_prev)  # defaults to zeros
    if opts.slew_control_points:
        mpc.setSlewRate(u_slew)
    if opts.saturate_states:
        mpc.setStateLimits(x_min, x_max)
    ```

=== "C++"

    ```cpp
    mpc.setInputLimits(u_min, u_max);
    if (opts.slew_initial_input) {
      mpc.setSlewRateInitial(u0_slew);
      mpc.setPreviousInput(u_prev); // defaults to zeros
    }
    if (opts.slew_control_points) {
      mpc.setSlewRate(u_slew);
    }
    if (opts.saturate_states) {
      mpc.setStateLimits(x_min, x_max);
    }
    ```

### 6. Set weights

Without input cost:

=== "Python"

    ```python
    mpc.setStateWeights(Q_diag, Qf_diag)
    mpc.setStateWeights(Q_diag)  # Qf = Q
    ```

=== "C++"

    ```cpp
    mpc.setStateWeights(Q_diag, Qf_diag);
    mpc.setStateWeights(Q_diag); // Qf = Q
    ```

With input cost:

=== "Python"

    ```python
    # set together
    mpc.setWeights(Q_diag, Qf_diag, R_diag)
    mpc.setWeights(Q_diag, R_diag)  # Qf = Q

    # OR set individually
    mpc.setStateWeights(Q_diag, Qf_diag)
    mpc.setInputWeights(R_diag)
    ```

=== "C++"

    ```cpp
    // set together
    mpc.setWeights(Q_diag, Qf_diag, R_diag);
    mpc.setWeights(Q_diag, R_diag); // Qf = Q

    // OR set individually
    mpc.setStateWeights(Q_diag, Qf_diag);
    mpc.setInputWeights(R_diag);
    ```

### 7. Set references

State step reference:

=== "Python"

    ```python
    mpc.setReferenceState(x_step)
    ```

=== "C++"

    ```cpp
    mpc.setReferenceState(x_step);
    ```

State trajectory reference:

=== "Python"

    ```python
    mpc.setReferenceStateTrajectory(x_traj)
    ```

=== "C++"

    ```cpp
    mpc.setReferenceStateTrajectory(x_traj);
    ```

If input cost is enabled, input references can also be configured:

=== "Python"

    ```python
    mpc.setReferenceInput(u_step)
    mpc.setReferenceParameterizedInputTrajectory(u_traj_ctrl_pts)
    ```

=== "C++"

    ```cpp
    mpc.setReferenceInput(u_step);
    mpc.setReferenceParameterizedInputTrajectory(u_traj_ctrl_pts);
    ```

### 8. Initialize the solver

=== "Python"

    ```python
    if not mpc.initializeSolver():
        raise RuntimeError("Failed to initialize solver")
    ```

=== "C++"

    ```cpp
    if (!mpc.initializeSolver()) {
      throw std::runtime_error("Failed to initialize MPC solver");
    }
    ```

### 9. Solve in the control loop

=== "Python"

    ```python
    status = mpc.solve(xk)
    if status != ampc.SolveStatus.Success:
        # handle how you want
        pass
    ```

=== "C++"

    ```cpp
    affine_mpc::SolveStatus status = mpc.solve(xk);
    if (status != affine_mpc::SolveStatus::Success) {
      // handle how you want
    }
    ```

### 10. Retrieve results in the control loop

=== "Python"

    ```python
    uk = mpc.getNextInput()
    u_traj = mpc.getInputTrajectory()
    u_traj_ctrl_pts = mpc.getParameterizedInputTrajectory()
    x_pred = mpc.getPredictedStateTrajectory()
    ```

=== "C++"

    ```cpp
    Eigen::VectorXd uk(input_dim);
    mpc.getNextInput(uk); // most common

    Eigen::VectorXd u_traj(input_dim * horizon_steps);
    mpc.getInputTrajectory(u_traj);

    Eigen::VectorXd u_traj_ctrl_pts(input_dim * num_control_points);
    mpc.getParameterizedInputTrajectory(u_traj_ctrl_pts);

    Eigen::VectorXd x_pred(state_dim * horizon_steps);
    mpc.getPredictedStateTrajectory(x_pred);
    ```

## Interface Notes

=== "Python"

    Many getters support two styles.

    Return a new array:

    ```python
    uk = mpc.getNextInput()  # most common
    u_traj = mpc.getInputTrajectory()
    u_traj_ctrl_pts = mpc.getParameterizedInputTrajectory()
    x_pred = mpc.getPredictedStateTrajectory()
    ```

    Or write into an existing array:

    ```python
    uk = np.empty(m)
    mpc.getNextInput(uk)  # uk is also returned
    ```

    The in-place form can be useful if you want to reuse arrays in tight loops for extra performance.

=== "C++"

    The C++ API commonly writes results into preallocated Eigen buffers.

    ```cpp
    Eigen::VectorXd uk(input_dim);
    mpc.getNextInput(uk);
    ```

    This is the natural pattern for tight loops where you want to avoid repeated allocations.

## Constructor Variants

Both `CondensedMPC` and `SparseMPC` support:

- an explicit `Parameterization` constructor
- a horizon-only constructor that defaults to one control point per step

=== "Python"

    ```python
    mpc = ampc.CondensedMPC(state_dim, input_dim, horizon_steps, opts)
    ```

=== "C++"

    ```cpp
    affine_mpc::CondensedMPC mpc(state_dim, input_dim, horizon_steps, opts);
    ```

## Solve Status

`solve()` returns `SolveStatus` for normal solver outcomes rather than throwing.

Common cases include:

- `Success`
- `NotInitialized`
- OSQP-derived failure conditions

Configuration misuse tends to throw exceptions, while runtime solver outcomes use return values.

## Model Propagation Helper

You can propagate the internal model one step with:

=== "Python"

    ```python
    x_next = mpc.propagateModel(xk, uk, x_next)
    ```

=== "C++"

    ```cpp
    mpc.propagateModel(xk, uk, x_next);
    ```

## Logging

Create a logger bound to one MPC object:

=== "Python"

    ```python
    logger = ampc.MPCLogger(mpc, save_dir, ts, prediction_stride=1)
    ```

=== "C++"

    ```cpp
    affine_mpc::MPCLogger logger{&mpc, "/tmp/ampc_example", dt};
    ```

Inside the control loop:

=== "Python"

    ```python
    logger.logStep(t, xk, solve_time)
    ```

=== "C++"

    ```cpp
    logger.logStep(t, xk, user_solve_time);
    ```

See [Logging](logging.md) for output format and workflow details.

## Practical Guidance

- Do not call `solve()` before `initializeSolver()`.
- If an option enables a constraint, call the corresponding setter before initialization.
- Fully configure the model, limits, weights, and references before calling `initializeSolver()`.
- QP matrix sparsity is fixed after initialization, so later updates must not introduce new nonzero structure.
- Runtime updates to model terms and weights must preserve the initialized QP sparsity pattern.
- If a model coefficient or cost weight may become nonzero later, initialize with that structure already present.
- If you enable `slew_initial_input`, provide the previous input before solving; after initial solve it is automatically set from the previous solve.

## Choosing Between Condensed and Sparse

- `CondensedMPC`: usually the best starting point, especially for shorter horizons and moderate dimensions
- `SparseMPC`: useful when you want to preserve more explicit sparsity structure or work with larger problems

The only way to know which is faster for your problem is to try them both.
