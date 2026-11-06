# <img src="https://raw.githubusercontent.com/byu-rad-lab/affine_mpc/main/docs/assets/favicon.svg" alt="affine_mpc logo" width="32" height="32"> affine_mpc

[![PyPI](https://img.shields.io/pypi/v/affine_mpc.svg)](https://pypi.org/project/affine-mpc/)
[![Build and Test](https://github.com/byu-rad-lab/affine_mpc/actions/workflows/build_test.yaml/badge.svg)](https://github.com/byu-rad-lab/affine_mpc/actions/workflows/build_test.yaml)
[![Check Stubs](https://github.com/byu-rad-lab/affine_mpc/actions/workflows/check_stubs.yaml/badge.svg)](https://github.com/byu-rad-lab/affine_mpc/actions/workflows/check_stubs.yaml)
[![Deploy Docs](https://github.com/byu-rad-lab/affine_mpc/actions/workflows/docs.yaml/badge.svg)](https://github.com/byu-rad-lab/affine_mpc/actions/workflows/docs.yaml)
[![Publish to PyPI](https://github.com/byu-rad-lab/affine_mpc/actions/workflows/publish_pypi.yaml/badge.svg)](https://github.com/byu-rad-lab/affine_mpc/actions/workflows/publish_pypi.yaml)

`affine_mpc` is a library for model predictive control using discrete-time affine models, with C++ and Python interfaces.
It is designed to support real-time control and rapid research prototyping with a focused set of common costs and constraints, along with live parameter updates.

## Statement of Need

Researchers and engineers often need an MPC tool that sits between low-level QP assembly and large general-purpose control frameworks.
`affine_mpc` aims to lower the barrier to entry for developing MPC controllers by reducing the amount of low-level problem assembly needed for common affine MPC workflows.
`affine_mpc` focuses on discrete-time affine MPC problems with optional costs and constraints, efficient repeated solves, and workflows that support both experimentation in Python and integration in C++.

## Highlights

- Condensed and sparse MPC formulations behind a common interface
- Move-blocking, linear-interpolation, B-spline, and custom-basis (e.g. Laguerre) input parameterizations
- Input limits enforced on the control points or the sampled inputs, and optional input and input-rate costs, per-sample slew-rate bounds, hard or soft state bounds, and general and terminal linear constraints
- Efficient OSQP updates when the QP sparsity pattern is unchanged, with DAQP as an optional dense solver
- Binary logging via `MPCLogger`, read in Python with `load_log()` for plotting and analysis

## Documentation

The primary documentation lives at <https://byu-rad-lab.github.io/affine_mpc/>.

- [Getting Started](https://byu-rad-lab.github.io/affine_mpc/getting-started/)
- [Python guide](https://byu-rad-lab.github.io/affine_mpc/getting-started/python/)
- [C++ guide](https://byu-rad-lab.github.io/affine_mpc/getting-started/cpp/)
- [Concepts and formulation](https://byu-rad-lab.github.io/affine_mpc/concepts/)
- [Example walkthrough](https://byu-rad-lab.github.io/affine_mpc/examples/)
- [Development guide](https://byu-rad-lab.github.io/affine_mpc/development/)

The Markdown source for the documentation lives in `docs/`.

Core C++ sources live under `src/` and `include/`, Python bindings and package sources live under `python/`, and example workflows live under `examples/`.

## Important Usage Notes

- Fully configure the model, limits, weights, and references before calling `initializeSolver()`
- OSQP sparsity is fixed at initialization; later updates must not introduce new nonzero structure (see [Runtime Updates](https://byu-rad-lab.github.io/affine_mpc/concepts/runtime-updates/) for which entries are tracked and how to include ones that become nonzero later)

## Community Standards

- [Contributing guide](https://github.com/byu-rad-lab/affine_mpc/blob/main/CONTRIBUTING.md)
- [Code of conduct](https://github.com/byu-rad-lab/affine_mpc/blob/main/CODE_OF_CONDUCT.md)

## Citation

If you use `affine_mpc` in research, please cite the project using the metadata in [`CITATION.cff`](https://github.com/byu-rad-lab/affine_mpc/blob/main/CITATION.cff).

## License

This project is licensed under the BSD 3-Clause License.
See [`LICENSE`](https://github.com/byu-rad-lab/affine_mpc/blob/main/LICENSE) for details.
