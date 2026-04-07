# C++ Usage

This page shows the intended workflow for the public C++ API and highlights the most commonly used classes and methods.

## Main Types

Public headers are in `include/affine_mpc/`.

The most important types are:

- `affine_mpc::Parameterization`
- `affine_mpc::Options`
- `affine_mpc::MPCBase`
- `affine_mpc::CondensedMPC`
- `affine_mpc::SparseMPC`
- `affine_mpc::MPCLogger`
- `affine_mpc::SolveStatus`

## Typical Usage Flow

### 1. Choose a parameterization

```cpp
auto param = affine_mpc::Parameterization::linearInterp(horizon_steps,
                                                        num_control_points);
```

### 2. Configure options if needed

```cpp
affine_mpc::Options opts;
opts.use_input_cost = false;
opts.slew_initial_input = false;
opts.slew_control_points = false;
opts.saturate_states = false;
opts.saturate_input_trajectory = false;
```

### 3. Construct the MPC object

```cpp
affine_mpc::CondensedMPC mpc(state_dim, input_dim, param, opts);
```

Or:

```cpp
affine_mpc::SparseMPC mpc(state_dim, input_dim, param, opts);
```

### 4. Set the model

Either provide a discrete model directly:

```cpp
mpc.setModelDiscrete(Ad, Bd, wd);
```

or discretize from continuous time:

```cpp
mpc.setModelContinuous2Discrete(Ac, Bc, wc, dt);
```

### 5. Set limits and enabled optional constraints

```cpp
mpc.setInputLimits(u_min, u_max);
if (opts.slew_initial_input) mpc.setSlewRateInitial(u0_slew);
if (opts.slew_control_points) mpc.setSlewRate(u_slew);
if (opts.saturate_states) mpc.setStateLimits(x_min, x_max);
```

### 6. Set weights

Without input cost:

```cpp
mpc.setStateWeights(Q_diag, Qf_diag);
mpc.setStateWeights(Q_diag);
```

With input cost:

```cpp
mpc.setWeights(Q_diag, Qf_diag, R_diag);
mpc.setWeights(Q_diag, R_diag);

mpc.setStateWeights(Q_diag, Qf_diag);
mpc.setInputWeights(R_diag);
```

These overloads let you either provide a separate terminal state weight or reuse the stage state weights at the terminal step.

### 7. Set references

State step reference:

```cpp
mpc.setReferenceState(x_ref);
```

State trajectory reference:

```cpp
mpc.setReferenceStateTrajectory(x_traj);
```

If input cost is enabled, input references can also be configured.

### 8. Initialize the solver

```cpp
if (!mpc.initializeSolver()) {
  throw std::runtime_error("Failed to initialize MPC solver");
}
```

### 9. Solve in the control loop

```cpp
affine_mpc::SolveStatus status = mpc.solve(xk);
```

### 10. Retrieve results

```cpp
Eigen::VectorXd uk(input_dim);
mpc.getNextInput(uk);

Eigen::VectorXd u_traj(input_dim * horizon_steps);
mpc.getInputTrajectory(u_traj);

Eigen::VectorXd x_pred(state_dim * horizon_steps);
mpc.getPredictedStateTrajectory(x_pred);
```

## Constructor Variants

Both `CondensedMPC` and `SparseMPC` support:

- an explicit `Parameterization` constructor
- a horizon-only constructor that defaults to one control point per step

Example:

```cpp
affine_mpc::CondensedMPC mpc(state_dim, input_dim, horizon_steps, opts);
```

## Solve Status

`solve()` returns `affine_mpc::SolveStatus` for normal solver outcomes rather than throwing.

Common cases include:

- `Success`
- `NotInitialized`
- OSQP-derived failure conditions

Configuration misuse tends to throw exceptions, while runtime solver outcomes use return values.

## Model Propagation Helper

You can propagate the internal model one step with:

```cpp
mpc.propagateModel(xk, uk, x_next);
```

## Logging

The logger is separate from the MPC classes:

```cpp
affine_mpc::MPCLogger logger{&mpc, "/tmp/ampc_example", dt};
logger.logStep(t, xk, user_solve_time);
```

See [Logging](../logging.md) for output format and workflow details.

## Practical Guidance

- Fully configure the model, limits, weights, and references before calling `initializeSolver()`
- QP matrix sparsity is fixed after initialization
- Runtime updates to model terms and weights must preserve the initialized QP sparsity pattern
- If a model coefficient or cost weight may become nonzero later, initialize with that structure already present
- Prefer `CondensedMPC` unless you have a concrete reason to use the sparse formulation
- Use preallocated Eigen buffers in tight loops when practical
- If you enable `slew_initial_input`, provide the previous input before solving
