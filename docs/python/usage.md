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

Start with `CondensedMPC` unless you have a concrete reason to use the sparse formulation.

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
if opts.slew_control_points:
    mpc.setSlewRate(u_slew)
if opts.saturate_states:
    mpc.setStateLimits(x_min, x_max)
```

### 6. Set weights and references

Without input cost:

```python
mpc.setStateWeights(Q_diag)
mpc.setReferenceState(x_ref)
```

With input cost:

```python
mpc.setWeights(Q_diag, R_diag)
mpc.setReferenceState(x_ref)
mpc.setReferenceInput(u_ref)
```

### 7. Initialize the solver

```python
if not mpc.initializeSolver():
    raise RuntimeError("Failed to initialize solver")
```

### 8. Solve in a loop

```python
status = mpc.solve(xk)
if status == ampc.SolveStatus.Success:
    uk = mpc.getNextInput()
```

## Getter Patterns

Many getters support two styles.

Return a new array:

```python
uk = mpc.getNextInput()
u_traj = mpc.getInputTrajectory()
x_pred = mpc.getPredictedStateTrajectory()
```

Or write into an existing array:

```python
uk = np.empty(m)
mpc.getNextInput(uk)
```

The in-place form can be useful if you want to reuse arrays in tight loops.

## Logging

Create a logger bound to one MPC object:

```python
logger = ampc.MPCLogger(mpc, save_dir, ts, prediction_stride=1)
logger.addMetadata("example_name", "mass_spring_damper")
```

Inside the control loop:

```python
logger.logStep(t, xk, solve_time)
```

See [Logging](../logging.md) for output details.

## Common Pitfalls

- Do not call `solve()` before `initializeSolver()`
- If an option enables a constraint, call the corresponding setter before initialization
- OSQP matrix sparsity is fixed after initialization, so later updates must not introduce new nonzero structure
- If you expect model entries to become nonzero later, initialize with that structure already present

## Choosing Between Condensed and Sparse

- `CondensedMPC`: usually the best starting point, especially for shorter horizons and moderate dimensions
- `SparseMPC`: useful when you want to preserve more explicit sparsity structure or work with larger problems

For the shared mathematical background, see [Concepts](../concepts.md).
