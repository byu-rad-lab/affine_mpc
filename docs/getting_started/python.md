# Python Getting Started

This guide gets you to a first successful solve with the Python package.

## Install

### From TestPyPI

```sh
pip install \
    --index-url https://test.pypi.org/simple/ \
    --extra-index-url https://pypi.org/simple/ \
    affine_mpc
```

Or if you want to also install the plotting dependencies for the examples:

```sh
pip install \
    --index-url https://test.pypi.org/simple/ \
    --extra-index-url https://pypi.org/simple/ \
    "affine_mpc[examples]"
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

Installing with `pip`, as shown above, installs the python examples into your Python environment along with these 3 scripts:

- `affine-mpc-example-sim` (runs sim and logs only)
- `affine-mpc-example-plot` (plots only)
- `affine-mpc-example` (runs sim, logs, and plots)

So, to run the sim and visualize the results:

```sh
affine-mpc-example
```

OR the same result with 2 commands:

```sh
affine-mpc-example-sim
affine-mpc-example-plot
```

### Run Sim File Directly

This is useful if you wish to modify the example sim or just want to run it directly.

```sh
python examples/sim.py
```

Expected outcome:

- The example runs successfully, printing the log path to the console.
- `log.npz` and `params.yaml` are written under the `ampc_example` temp directory.
- The plotting helper opens figures if the visualization dependencies are installed.

For more detail on what the example demonstrates and how to interpret the outputs, see [Examples](../examples.md).

### Plot the Results

After running the example, plot the log with:

```sh
python examples/plot_sim.py
```

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

For the full workflow and class-level guidance, see [Usage](usage.md).
