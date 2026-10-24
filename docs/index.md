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

## What It Is Not

`affine_mpc` solves one convex QP per time step for a model you provide, so some related tools are out of scope:

- **Nonlinear MPC.**
  The model inside the QP is affine.
  For a nonlinear plant, compute an affine approximation yourself (by hand, Eigen AutoDiff, CasADi, or JAX) about an operating point or the current trajectory and update the model between solves.
  The library does not differentiate models or iterate on the approximation within a time step.
- **Explicit MPC.**
  The QP is solved online at every step; no offline partition of the state space or lookup table is computed.
- **State estimation.**
  `solve()` takes the current state as given.
  Observers, Kalman filters, and disturbance estimators are up to you (see [Modeling Patterns](concepts/modeling-patterns.md) for how to use a disturbance estimate).
- **A general optimization toolkit.**
  The cost and constraint structure is fixed (see [Concepts](concepts/index.md)), and OSQP is the only QP solver.

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
