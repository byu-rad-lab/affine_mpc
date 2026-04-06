# affine_mpc

`affine_mpc` is a library for model predictive control (MPC) using discrete-time affine models.
There are interfaces in C++ and Python.
The library seeks to enable real-time MPC and lower the barrier to entry to develop an MPC controller.
It provides a base set of common MPC costs and constraint formulations that can be individually toggled, and efficiently allows for live parameter updates.

<!-- It provides condensed and sparse MPC formulations, input trajectory parameterization, OSQP-backed solves, and binary logging for simulation and analysis workflows. -->

## Statement of Need

Researchers and engineers often need an MPC tool that sits between low-level QP assembly and large general-purpose control frameworks.
`affine_mpc` focuses on a common and practical problem class:
discrete-time affine models with optional costs and constraints, efficient repeated solves, and a workflow that supports both experimentation in Python and integration in C++.

## Highlights

- Condensed and sparse MPC formulations behind a common interface
- Move-blocking, linear interpolation, and B-spline input parameterizations
- Optional input cost, state bounds, input bounds, and slew-rate constraints
- Efficient OSQP-backed updates when the QP sparsity pattern is unchanged
- Binary `.npz` logging via `MPCLogger` for plotting and post-run analysis

## Choose Your Interface

### Python

Use Python if you want fast iteration for research, analysis, scripting, or teaching.

The easiest way to get started today is to install from PyPI:

```sh
python -m pip install --index-url https://test.pypi.org/simple/ --extra-index-url https://pypi.org/simple/ affine_mpc
```

<!-- TODO: uncomment after release -->
<!-- ```python -->
<!-- python -m pip install affine_mpc -->
<!-- ``` -->

Then see:

- `docs/python/getting-started.md`
- `docs/python/usage.md`

### C++

Use C++ if you want direct integration into a native project or tighter control over runtime and memory behavior.

Build with CMake:

```sh
cmake -S . -B build -DAFFINE_MPC_BUILD_TESTS=ON -DAFFINE_MPC_BUILD_EXAMPLES=ON
cmake --build build --config Release --parallel
```

Then see:

- `docs/cpp/getting-started.md`
- `docs/cpp/usage.md`

## Documentation

The full documentation lives in `docs/` and can also be rendered with [Zensical](https://zensical.org/).

- `docs/index.md` - documentation home page
- `docs/concepts.md` - optimization problem, parameterization, formulations, and OSQP constraints
- `docs/python/getting-started.md` - Python installation and first run
- `docs/python/usage.md` - Python workflow and key APIs
- `docs/cpp/getting-started.md` - C++ build, tests, and first run
- `docs/cpp/usage.md` - C++ workflow and key APIs
- `docs/examples.md` - shared mass-spring-damper example in both interfaces
- `docs/logging.md` - `MPCLogger` outputs, metadata, and analysis notes
- `docs/development.md` - repository layout, testing, formatting, and contribution workflow

To build the docs locally:

```sh
python -m pip install zensical
zensical serve
```

<!-- ```sh -->
<!-- python -m pip install mkdocs mkdocs-material pymdown-extensions -->
<!-- mkdocs serve -->
<!-- ``` -->

## Example Workflow

The repository includes both C++ and Python mass-spring-damper examples:

- `examples/sim.cpp`
- `examples/sim.py`
- `examples/plot_sim.py`

See `docs/examples.md` for the recommended walkthrough.

## Important Usage Notes

- Fully configure the model, limits, weights, and references before calling `initializeSolver()`
- OSQP sparsity is fixed at initialization; later updates must not introduce new nonzero structure
- If a matrix entry may become nonzero later, initialize with that structure already present
- Prefer `CondensedMPC` unless you specifically need the sparse formulation

## Community Standards

- Contributing guide: `CONTRIBUTING.md`
- Code of conduct: `CODE_OF_CONDUCT.md`
- Citation metadata: `CITATION.cff`

## Citation

If you use `affine_mpc` in research, please cite the project using the metadata in `CITATION.cff`.

## License

This project is licensed under the BSD 3-Clause License.
See `LICENSE` for details.
