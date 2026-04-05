# Examples

`affine_mpc` ships with a shared example workflow centered on a mass-spring-damper tracking problem. The same problem is implemented in both C++ and Python so users can learn one conceptual workflow and choose the interface that best matches their application.

## Example Files

- `examples/sim.cpp`
- `examples/sim.py`
- `examples/plot_sim.py`

## What the Example Demonstrates

- building a linear interpolation parameterization
- configuring a `CondensedMPC`
- setting model, limits, weights, and references
- solving in a closed loop
- logging outputs with `MPCLogger`
- plotting the resulting `.npz` log from Python

## Run the Example

=== "Python"

    Install from the repository if needed:

    ```sh
    python -m pip install .
    python -m pip install ".[examples]"
    ```

    Run the example:

    ```sh
    python examples/sim.py
    ```

=== "C++"

    Build with examples enabled:

    ```sh
    cmake -S . -B build -DAFFINE_MPC_BUILD_EXAMPLES=ON
    cmake --build build --config Release --parallel
    ```

    Run the example:

    ```sh
    ./build/example_sim
    ```

## Core Example Structure

The two implementations follow the same steps:

1. define the model dimensions
2. choose a parameterization
3. construct `CondensedMPC`
4. set model, limits, weights, and references
5. call `initializeSolver()`
6. solve in a loop
7. retrieve the next input and propagate the plant
8. log the results

## What the Example Produces

The example writes output into the system temp directory under `ampc_example`.

Typical outputs are:

- `log.npz`
- `params.yaml`

These files can be plotted from Python with:

```sh
python examples/plot_sim.py
```

The plot shows:

- actual state history
- reference state history
- applied input
- optional input reference if input cost is enabled
- selected prediction rollouts over time
- solve times

## Suggested First Modifications

If you are new to the library, try changing:

- prediction horizon `T`
- number of control points
- state weights `Q_diag`
- input cost weight `R_diag`
- input or state limits

This is the fastest way to build intuition for how the parameterization and optional constraints affect behavior.

## Relationship to Tests

Many tests use the same small mass-spring-damper system because it is easy to understand and fast to run. The example therefore reflects the same style of problem setup exercised in the unit tests.
