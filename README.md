# affine_mpc

`affine_mpc` is a library for model predictive control using discrete-time affine models, with C++ and Python interfaces.
It is designed to support real-time control and rapid research prototyping with a focused set of common costs and constraints, along with live parameter updates.

## Statement of Need

Researchers and engineers often need an MPC tool that sits between low-level QP assembly and large general-purpose control frameworks.
`affine_mpc` aims to lower the barrier to entry for developing MPC controllers by reducing the amount of low-level problem assembly needed for common affine MPC workflows.
`affine_mpc` focuses on discrete-time affine MPC problems with optional costs and constraints, efficient repeated solves, and workflows that support both experimentation in Python and integration in C++.

## Highlights

- Condensed and sparse MPC formulations behind a common interface
- Move-blocking, linear interpolation, and B-spline input parameterizations
- Optional input cost, state bounds, input bounds, and slew-rate constraints
- Efficient OSQP-backed updates when the QP sparsity pattern is unchanged
- Binary `.npz` logging via `MPCLogger` for plotting and post-run analysis

## Documentation

The primary documentation lives at <https://byu-rad-lab.github.io/affine_mpc/>.

- [Concepts and formulation](https://byu-rad-lab.github.io/affine_mpc/concepts/)
- [Python guide](https://byu-rad-lab.github.io/affine_mpc/python/)
- [C++ guide](https://byu-rad-lab.github.io/affine_mpc/cpp/)
- [Example walkthrough](https://byu-rad-lab.github.io/affine_mpc/examples/)
- [Development guide](https://byu-rad-lab.github.io/affine_mpc/development/)

The Markdown source for the documentation lives in `docs/`.

Core C++ sources live under `src/` and `include/`, Python bindings and package sources live under `python/`, and example workflows live under `examples/`.

## Important Usage Notes

- Fully configure the model, limits, weights, and references before calling `initializeSolver()`
- OSQP sparsity is fixed at initialization; later updates must not introduce new nonzero structure
- If a matrix entry may become nonzero later, initialize with that structure already present

## Community Standards

- Contributing guide: `CONTRIBUTING.md`
- Code of conduct: `CODE_OF_CONDUCT.md`
- Citation metadata: `CITATION.cff`

## Citation

If you use `affine_mpc` in research, please cite the project using the metadata in `CITATION.cff`.

## License

This project is licensed under the BSD 3-Clause License.
See `LICENSE` for details.
