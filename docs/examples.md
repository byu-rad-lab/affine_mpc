# Examples

`affine_mpc` ships with a small but useful example workflow centered on a mass-spring-damper tracking problem.

## C++ Example

The main executable example is:

- `examples/sim.cpp`

It demonstrates:

- building a linear interpolation parameterization
- configuring a `CondensedMPC`
- setting model, limits, weights, and references
- solving in a loop
- logging outputs with `MPCLogger`

Build and run it with:

```sh
cmake -S . -B build -DAFFINE_MPC_BUILD_EXAMPLES=ON
cmake --build build --config Release --parallel
./build/example_sim
```

## What the Example Produces

The example writes logging output into the system temp directory under `ampc_example`.

Typical outputs:

- `log.npz`
- `params.yaml`

These files can be plotted from Python.

## Plotting Scripts

The repository includes:

- `examples/plot_sim_tracking.py`
- `examples/plot_sim_predictions.py`
- `examples/plot_npz.py`

### Tracking view

```sh
python examples/plot_sim_tracking.py
```

Shows:

- actual state history
- reference state history
- applied input
- optional input reference if input cost is enabled
- solve times

### Prediction view

```sh
python examples/plot_sim_predictions.py
```

Shows:

- actual trajectories
- reference trajectories
- selected prediction rollouts over time
- input limits and solve times

### Generic NPZ viewer

```sh
python examples/plot_npz.py /tmp/ampc_example
```

This helper loads both the `.npz` data file and the YAML parameter file.

## Suggested First Modifications

If you are new to the library, try changing:

- prediction horizon `T`
- number of control points
- state weights `Q_diag`
- input cost weight `R_diag`
- input or state limits

This is the fastest way to build intuition for how the parameterization and constraint options affect behavior.

## Using the Example as a Template

For a new application, the example structure is a good starting point:

1. define model dimensions
2. choose parameterization
3. construct MPC object
4. set model and problem data
5. initialize solver
6. solve in a loop
7. retrieve inputs and propagate the plant
8. optionally log and plot

## Relationship to Tests

Many tests use the same small mass-spring-damper system because it is easy to understand and fast to run.
The example therefore reflects the same style of problem setup exercised in the unit tests.
