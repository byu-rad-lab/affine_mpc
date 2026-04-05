# Python Getting Started

This guide gets you to a first successful solve with the Python package.

## Install

If a published package is available for your platform, install it with `pip`.

```sh
python -m pip install affine_mpc
```

To install from this repository instead:

```sh
python -m pip install .
```

To run the example plotting workflow, install the optional extras:

```sh
python -m pip install ".[examples]"
```

## Minimal Example

```python
import numpy as np
import affine_mpc as ampc

n = 2
m = 1
T = 10
nc = 3
dt = 0.1

param = ampc.Parameterization.linearInterp(horizon_steps=T, num_control_points=nc)
mpc = ampc.CondensedMPC(state_dim=n, input_dim=m, param=param)

A = np.array([[0.0, 1.0], [-0.6, -0.1]])
B = np.array([0.0, 0.2])
w = np.zeros(n)

mpc.setModelContinuous2Discrete(A, B, w, dt)
mpc.setInputLimits(np.zeros(m), np.array([3.0]))
mpc.setStateWeights(np.array([1.0, 0.1]))
mpc.setReferenceState(np.array([1.0, 0.0]))

if not mpc.initializeSolver():
    raise RuntimeError("Failed to initialize solver")

xk = np.zeros(n)
status = mpc.solve(xk)
if status != ampc.SolveStatus.Success:
    raise RuntimeError(f"Solve failed with status {status}")

uk = mpc.getNextInput()
print("next input:", uk)
```

## Run the Example Script

The repository ships with `examples/sim.py`, which runs a mass-spring-damper example and logs the results.

If you installed from this repository, run:

```sh
python examples/sim.py
```

If you installed the package entry points, you can also run:

```sh
affine-mpc-example-sim
```

The example writes output to the system temporary directory under `ampc_example`, typically `/tmp/ampc_example` on Linux.

## Plot the Results

After running the example, plot the log with:

```sh
python examples/plot_sim.py
```

Or, if the package entry point is available:

```sh
affine-mpc-example-plot
```

## Array Conventions

- Use NumPy arrays for states, inputs, limits, weights, and references
- Vector arguments are typically one-dimensional arrays such as `np.zeros(n)`
- Many getters support both styles:
  - return a new NumPy array
  - write into an array you provide and return that same array

For the full workflow and class-level guidance, see [Usage](usage.md).
