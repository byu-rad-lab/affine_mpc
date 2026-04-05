# affine_mpc

`affine_mpc` is a library for real-time model predictive control using discrete-time affine time-invariant models.
There are both C++ and Python versions of the library.

<!-- It provides condensed and sparse MPC formulations, -->
<!-- input trajectory parameterization through move-blocking and B-splines, -->
<!-- OSQP-backed solves, and binary logging for simulation and analysis workflows. -->

## Statement of Need

Researchers and engineers often need an MPC tool that sits between low-level QP assembly and large general-purpose control frameworks.
`affine_mpc` focuses on a common and practical problem class:
discrete-time affine models with optional costs and constraints, efficient repeated solves, and a workflow that supports both experimentation in Python and integration in C++.

<!-- `affine_mpc` formulates and manages a QP optimization from a simple and easy-to-use MPC problem that supports commonly used costs and constraints. -->
<!-- This allows users to think and interact with the MPC problem definition without having to set up an optimization problem at all. -->

## Highlights

- Condensed and sparse MPC formulations behind a common interface
- Move-blocking, linear interpolation, and B-spline input parameterizations
- Optional input cost, state bounds, input bounds, and slew-rate constraints
- Efficient OSQP-backed updates when the QP sparsity pattern is unchanged
- Binary `.npz` logging via `MPCLogger` for plotting and post-run analysis

## Choose Your Interface

### Python

The easiest way to get started is by installing from PyPI:

```sh
python -m pip install affine_mpc
```

Or install from this repository:

```sh
python -m pip install .
```

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

The full documentation lives in `docs/` and can also be rendered with MkDocs Material.

- `docs/index.md` - documentation home page
- `docs/concepts.md` - optimization problem, parameterization, formulations, and OSQP constraints
- `docs/python/getting-started.md` - Python installation and first run
- `docs/python/usage.md` - Python workflow and key APIs
- `docs/cpp/getting-started.md` - C++ build, tests, and first run
- `docs/cpp/usage.md` - C++ workflow and key APIs
- `docs/examples.md` - shared mass-spring-damper example in both interfaces
- `docs/logging.md` - `MPCLogger` outputs, metadata, and analysis notes
- `docs/development.md` - repository layout, testing, formatting, and contribution workflow

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

This project is licensed under the BSD 3-Clause License. See `LICENSE` for details.
