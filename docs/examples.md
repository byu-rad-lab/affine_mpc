# Examples

`affine_mpc` ships with a shared example workflow centered on a simple mass-spring-damper tracking problem (a linear 2nd-order system).
The same problem is implemented in both C++ and Python so users can learn one conceptual workflow and choose the interface that best matches their application.

## Example Files

- `examples/sim.cpp`
- `examples/sim.py`
- `examples/plot_sim.py`

`sim.cpp` and `sim.py` were designed to be near-exact translations of each other; they both run the simple mass-spring-damper sim and print the location of the saved log file.
The log file is saved to the same location, so running `plot_sim.py` works after running either the C++ or Python versions of the sim.

The C++ version of `affine_mpc` does not come with plotting capabilities.
It is designed to log `.npz` files that can be loaded in Python with `numpy.load()`.
Since Python can natively plot the logged results, the only difference between the two sim versions is that `sim.py` will also plot the results when run directly.

## What the Example Demonstrates

- Building a linear interpolation parameterization
- Configuring a `CondensedMPC`
- Setting model, limits, weights, and references
- Solving in a closed loop
- Logging outputs with `MPCLogger`
- Loading and plotting the resulting `.npz` log from Python

## Run the Example

The simulation can be run from either interface, but visualization is currently supported from Python only.

- To run the Python example, see [Python Getting Started](python/getting-started.md).
- To build and run the C++ example, see [C++ Getting Started](cpp/getting-started.md).
- To inspect logs produced by either interface, use the Python plotting workflow described in [Python Getting Started](python/getting-started.md#plot-the-results).

## Core Example Structure

The two implementations follow the same steps:

1. Define the model dimensions
1. Choose a `Parameterization` and `Options`
1. Construct `CondensedMPC`
1. Set model, limits, weights, and references
1. Call `initializeSolver()`
1. Solve in a loop
1. Retrieve the next input and propagate the system
1. Log the results

## What the Example Produces

The example writes output into the system temp directory under `ampc_example`.

The two outputs are:

- `log.npz`
- `params.yaml`

The controller tracks a constant position target for the mass-spring-damper system.
The logger records both the realized trajectory and selected predictions.
Logs produced by either interface can be visualized with the Python plotting workflow described in [Python Getting Started](python/getting-started.md).

The plot shows:

- Actual state history
- Reference state history
- Applied input
- Optional input reference if input cost is enabled
- Selected prediction rollouts over time
- Solve times

## Suggested First Modifications

If you are new to the library, try changing:

- Prediction horizon `T`
- Number of control points
- State weights `Q_diag`
- Input cost weight `R_diag`
- Input or state limits

This is the fastest way to build intuition for how the parameterization and optional constraints affect behavior.

## Relationship to Tests

Many tests use the same small mass-spring-damper system because it is easy to understand and fast to run.
The example therefore reflects the same style of problem setup exercised in the unit tests.
