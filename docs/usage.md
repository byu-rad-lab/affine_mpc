---
title: Usage
---

# Usage

This page shows the intended workflow for both the C++ and Python interfaces.
The solver concepts and setup order are the same in both languages; the tabs below only change the interface syntax.

For the shared mathematical background, see [Concepts](concepts/index.md).

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

    The most important types in the `affine_mpc` namespace are:

    - `Parameterization`
    - `Options`
    - `MPCBase`
    - `CondensedMPC`
    - `SparseMPC`
    - `MPCLogger`
    - `SolveStatus`

    For users wanting to modify default solver settings, `OSQPSettings` is defined in the global namespace from OSQP.

## Typical Workflow

### 1. Choose a parameterization

Factory methods are provided for

- Move-blocking (`moveBlocking()`)
- Linear interpolation (`linearInterp()`)
- Clamped B-splines (`bspline()`)

When creating a `Parameterization` object, you can either specify how many control points to use (uniform knots are used) or you can supply a custom knot vector.
For unclamped splines, you must use the regular constructor directly.

If you are unsure what to use, linear interpolation is a good default:

=== "Python"

    ```python
    param = affine_mpc.Parameterization.linearInterp(horizon_steps, num_control_points)
    ```

=== "C++"

    ```cpp
    auto param = affine_mpc::Parameterization::linearInterp(horizon_steps, num_control_points);
    ```

See [Input Parameterization](concepts/input-parameterization.md#provided-factory-methods) for more details.

### 2. Configure options if needed

=== "Python"

    ```python
    opts = affine_mpc.Options(
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

See [Concepts](concepts/index.md) for more details on options.

### 3. Construct the MPC object

Choose between `CondensedMPC` and `SparseMPC` (they have the same interfaces).

- `CondensedMPC`: usually the best starting point, especially for shorter horizons and moderate dimensions
- `SparseMPC`: useful when you want to preserve more explicit sparsity structure or work with larger problems

The best way to know which is faster for your problem is to try them both.

**With Parameterization:**

=== "Python"

    ```python
    mpc = affine_mpc.CondensedMPC(state_dim=n, input_dim=m, param=param, opts=opts)
    # mpc = affine_mpc.SparseMPC(state_dim=n, input_dim=m, param=param, opts=opts)
    ```

=== "C++"

    ```cpp
    affine_mpc::CondensedMPC mpc(state_dim, input_dim, param, opts);
    // affine_mpc::SparseMPC mpc(state_dim, input_dim, param, opts);
    ```

**No Parameterization:**

=== "Python"

    ```python
    mpc = affine_mpc.CondensedMPC(state_dim, input_dim, horizon_steps, opts)
    # mpc = affine_mpc.SparseMPC(state_dim, input_dim, horizon_steps, opts)
    ```

=== "C++"

    ```cpp
    affine_mpc::CondensedMPC mpc(state_dim, input_dim, horizon_steps, opts);
    // affine_mpc::SparseMPC mpc(state_dim, input_dim, horizon_steps, opts);
    ```

### 4. Configure the MPC object

The order of these steps does not matter, but they all must happen before initializing the solver.

#### Set the model

**Discrete:**

=== "Python"

    ```python
    mpc.setModelDiscrete(Ad, Bd, wd)
    ```

=== "C++"

    ```cpp
    mpc.setModelDiscrete(Ad, Bd, wd);
    ```

**Continuous:**

=== "Python"

    ```python
    mpc.setModelContinuous2Discrete(Ac, Bc, wc, dt)
    ```

=== "C++"

    ```cpp
    mpc.setModelContinuous2Discrete(Ac, Bc, wc, dt);
    ```

#### Set input limits

=== "Python"

    ```python
    mpc.setInputLimits(u_min, u_max)
    ```

=== "C++"

    ```cpp
    mpc.setInputLimits(u_min, u_max);
    ```

#### Set enabled optional parameters

=== "Python"

    ```python
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

#### Set weights

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

#### Set references

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
    # OR
    mpc.setReferenceInputControlPoints(control_points)
    ```

=== "C++"

    ```cpp
    mpc.setReferenceInput(u_step);
    // OR
    mpc.setReferenceInputControlPoints(control_points);
    ```

### 5. Initialize the solver

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

You can pass an `OSQPSettings` object into `initializeSolver()` if you want to customize the default solver settings.

### 6. Solve in the control loop

Configuration misuse tends to throw exceptions, but `solve()` returns a `SolveStatus` type rather than throwing since it
is in the main runtime loop. Common cases include:

- `Success`
- `NotInitialized` (`solve()` was called prior to `initializeSolver()`)
- OSQP-derived failure conditions (see [OSQP documentation](https://osqp.org/docs/interfaces/status_values.html#status-values))

=== "Python"

    ```python
    status = mpc.solve(xk)
    if status != affine_mpc.SolveStatus.Success:
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

### 7. Retrieve results in the control loop

To get the first optimized input (the next to apply):

=== "Python"

    ```python
    uk = mpc.getNextInput()
    ```

=== "C++"

    ```cpp
    Eigen::VectorXd uk(input_dim);
    mpc.getNextInput(uk);
    ```

You can also access the full input and predicted state trajectories:

=== "Python"

    ```python
    u_traj = mpc.getInputTrajectory()
    u_control_points = mpc.getInputControlPoints()
    x_pred = mpc.getPredictedStateTrajectory()
    ```

=== "C++"

    ```cpp
    Eigen::VectorXd u_traj(input_dim * horizon_steps);
    mpc.getInputTrajectory(u_traj);

    Eigen::VectorXd u_control_points(input_dim * num_control_points);
    mpc.getInputControlPoints(u_control_points);

    Eigen::VectorXd x_pred(state_dim * horizon_steps);
    mpc.getPredictedStateTrajectory(x_pred);
    ```

### 8. (Optional) Logging

Create a logger bound to one MPC object:

=== "Python"

    ```python
    logger = affine_mpc.MPCLogger(
        mpc,
        save_dir,
        ts,
        prediction_stride=1,
        mode=affine_mpc.MPCLogger.Mode.NpzCompressed,
    )
    ```

=== "C++"

    ```cpp
    affine_mpc::MPCLogger logger{&mpc, "/tmp/affine_mpc_example", dt, 1, false,
                                 "log",
                                 affine_mpc::MPCLogger::Mode::NpzCompressed};
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

## Practical Guidance

- Do not call `solve()` before `initializeSolver()`.
- If an option enables a constraint, call the corresponding setter before initialization.
- Fully configure the model, limits, weights, and references before calling `initializeSolver()`.
- QP matrix sparsity is fixed after initialization, so later updates must not introduce new nonzero structure.
- Runtime updates to model terms and weights must preserve the initialized QP sparsity pattern.
- If a model coefficient or cost weight may become nonzero later, initialize with that structure already present.
- If you enable `slew_initial_input`, provide the previous input before solving; after initial solve, it is automatically set from the previous solve.
