# affine_mpc

`affine_mpc` is a library for model predictive control (MPC) of discrete-time affine systems,
with C++ and Python interfaces.

## What It Supports

- Discrete-time affine time-invariant models
- Finite-horizon tracking MPC
- Condensed and sparse formulations
- Input parameterization with move-blocking, linear interpolation, and B-splines
- Optional input, state, and slew-rate constraints
- Repeated-solve workflows with runtime updates
- Binary logging for simulation and analysis

## Choose Your Path

- [Getting Started](getting-started/index.md): install the library and run a first example
- [Concepts](concepts/index.md): understand the supported problem class and mathematical structure
- [Usage](usage.md): learn the API workflow for configuring and solving MPC problems

## Who It Is For

`affine_mpc` is aimed at researchers, students, and engineers who want a focused MPC library rather than a general optimization toolkit.
The Python interface is convenient for experimentation, analysis, and teaching.
The C++ interface is better suited for integration into performance-sensitive applications.
It is intended to lower the barrier to developing MPC controllers for discrete-time affine systems by packaging common formulations, constraints, and workflows into a focused library.

## Contributing

For repository structure, testing, and contribution workflow, see [Development](development.md).
