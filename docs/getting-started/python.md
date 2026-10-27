# Python Getting Started

This guide gets you to a first successful run with the Python package.

## Install

### From PyPI

```sh
pip install affine_mpc
```

Or if you want to also install the plotting dependencies for the examples:

```sh
pip install "affine_mpc[examples]"
```

### From Source

For extra control over the build process, you can build from source:

```sh
python -m pip install .
```

Or if you want to also install the plotting dependencies for the examples:

```sh
python -m pip install ".[examples]"
```

The main reasons to build from source are if you make any modifications to the package, or if you want to link to system libs rather than compile them into the Python bindings.
This will link to system shared libs if they are available, otherwise it will copy the shared libs into the Python environment:

```sh
python -m pip install . --config-settings=cmake.define.BUILD_SHARED_LIBS=ON
```

!!! warning

    Using shared libs may not work on Windows as it handles runtime paths differently.

## Run the Example Application

### Use Installed Scripts (fastest start)

Installing with `pip`, as shown above, installs the Python examples into your Python environment along with one command per example:

- `affine-mpc-example-msd`: the [mass-spring-damper](../examples/mass-spring-damper.md), which runs the sim and writes the log
- `affine-mpc-example-quadrotor`: the [quadrotor](../examples/quadrotor-model-updates.md)

Both take `--plot` to plot the results, and `--load --plot` to plot existing logs without simulating again.
So, to run the sim and visualize the results:

```sh
affine-mpc-example-msd --plot
```

OR the same result in 2 steps:

```sh
affine-mpc-example-msd
affine-mpc-example-msd --load --plot
```

!!! note

    Earlier versions installed `affine-mpc-example`, `affine-mpc-example-sim`, and `affine-mpc-example-plot`.
    They still work, with a deprecation notice, and are the same as `affine-mpc-example-msd` with `--plot`, no options, and `--load --plot`.

### Run Sim File Directly

This is useful if you wish to modify the example sim or just want to run it directly.

```sh
python examples/sim.py --plot
```

Expected outcome:

- The example runs successfully, printing the log path to the console.
- `log.npz` and `params.yaml` are written under the `ampc_example` temp directory, replacing the log of any earlier run.
- With `--plot`, the plotting helper opens figures if the visualization dependencies are installed.

For more detail on what the example demonstrates and how to interpret the outputs, see [Mass-Spring-Damper](../examples/mass-spring-damper.md).

### Plot the Results

After running the example (Python or C++), plot the log with:

```sh
python examples/plot_sim.py
```

or, equivalently, `python examples/sim.py --load --plot` or `affine-mpc-example-msd --load --plot`.

#### Visualization Dependencies

If you did not install the Python package with the `[examples]` optional dependencies, as shown above, you can install the visualization dependencies with:

```sh
python -m pip install -r requirements/examples.txt
```

OR install the following manually:

- `abracatabra`
- `matplotlib`
- `PySide6` OR `PyQt6`

## Array Conventions

- Use NumPy arrays for states, inputs, limits, weights, and references
- Vector arguments are typically one-dimensional arrays such as `np.zeros(n)`
- Many getters support both styles: return a new NumPy array, or write into an array you provide and return that same array

For the full workflow and class-level guidance, see [Usage](../usage.md).
