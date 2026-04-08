# Python Usage

This page shows the intended workflow for the Python package and highlights the most commonly used classes and methods.

## Main Types

The most important names exported by `affine_mpc` are:

- `Parameterization`
- `Options`
- `OSQPSettings`
- `CondensedMPC`
- `SparseMPC`
- `MPCLogger`
- `SolveStatus`

## Typical Workflow

### 1. Choose a parameterization

```python
param = ampc.Parameterization.linearInterp(horizon_steps=T, num_control_points=nc)
```

Other common choices are `moveBlocking(...)` and `bspline(...)`.

### 2. Configure options if you need optional costs or constraints

```python
opts = ampc.Options(
    use_input_cost=True,
    slew_initial_input=False,
    slew_control_points=True,
    saturate_states=False,
    saturate_input_trajectory=False,
)
```

### 3. Construct the MPC object

```python
mpc = ampc.CondensedMPC(state_dim=n, input_dim=m, param=param, opts=opts)
```

Or:

```python
mpc = ampc.SparseMPC(state_dim=n, input_dim=m, param=param, opts=opts)
```

May want to try `CondensedMPC` and `SparseMPC` to see which is faster for your problem.

### 4. Set the model

Provide a discrete model directly:

```python
mpc.setModelDiscrete(Ad, Bd, wd)
```

or discretize from continuous time:

```python
mpc.setModelContinuous2Discrete(Ac, Bc, wc, dt)
```

### 5. Set limits required by the enabled options

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

### 6. Set weights

Without input cost:

```python
mpc.setStateWeights(Q_diag, Qf_diag)
mpc.setStateWeights(Q_diag)  # Qf = Q
```

With input cost:

```python
# set together
mpc.setWeights(Q_diag, Qf_diag, R_diag)
mpc.setWeights(Q_diag, R_diag)  # Qf = Q

# OR set individually
mpc.setStateWeights(Q_diag, Qf_diag)
mpc.setInputWeights(R_diag)
```

### 7. Set references

State step reference:

```python
mpc.setReferenceState(x_step)
```

State trajectory reference:

```python
mpc.setReferenceStateTrajectory(x_traj)
```

If input cost is enabled, input references can also be configured:

```python
mpc.setReferenceInput(u_step)
mpc.setReferenceParameterizedInputTrajectory(u_traj_ctrl_pts)
```

### 8. Initialize the solver

```python
if not mpc.initializeSolver():
    raise RuntimeError("Failed to initialize solver")
```

### 9. Solve in the control loop

```python
status = mpc.solve(xk)
if status != ampc.SolveStatus.Success:
    # handle how you want
```

### 10. Retrieve results in the control loop

```python
uk = mpc.getNextInput()
u_traj = mpc.getInputTrajectory()
x_traj = mpc.getPredictedStateTrajectory()
```

## Getter Patterns

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

## Constructor Variants

Both `CondensedMPC` and `SparseMPC` support:

- an explicit `Parameterization` constructor
- a horizon-only constructor that defaults to one control point per step (same as no parameterization)

Example:

```python
mpc = affine_mpc.CondensedMPC(state_dim, input_dim, horizon_steps, opts)
```

## Solve Status

`solve()` returns `affine_mpc.SolveStatus` for normal solver outcomes rather than throwing.

Common cases include:

- `Success`
- `NotInitialized`
- OSQP-derived failure conditions

Configuration misuse tends to throw exceptions, while runtime solver outcomes use return values.

## Model Propagation Helper

You can propagate the internal model one step with:

```python
x_next = mpc.propagateModel(xk, uk, x_next)  # can pass in output arg to avoid memory copies
```

## Logging

Create a logger bound to one MPC object:

```python
logger = ampc.MPCLogger(mpc, save_dir, ts, prediction_stride=1)
```

Inside the control loop:

```python
logger.logStep(t, xk, solve_time)
```

See [Logging](../logging.md) for output format and workflow details.

## Common Pitfalls

- Do not call `solve()` before `initializeSolver()`.
- If an option enables a constraint, call the corresponding setter before initialization.
- Fully configure the model, limits, weights, and references before calling `initializeSolver()`.
- QP matrix sparsity is fixed after initialization, so later updates must not introduce new nonzero structure.
- Runtime updates to model terms and weights must preserve the initialized QP sparsity pattern.
- If a model coefficient or cost weight may become nonzero later, initialize with that structure already present.
- If you enable `slew_initial_input`, provide the previous input before solving; after initial solve it is automatically set from previous solve.

## Choosing Between Condensed and Sparse

- `CondensedMPC`: usually the best starting point, especially for shorter horizons and moderate dimensions
- `SparseMPC`: useful when you want to preserve more explicit sparsity structure or work with larger problems

The only way to know which is faster for your problem is to try them both!

For the shared mathematical background, see [Concepts](../concepts.md).
