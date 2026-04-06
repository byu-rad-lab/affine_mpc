# affine_mpc

`affine_mpc` is a library for model predictive control of discrete-time affine systems with equal first-class interfaces in C++ and Python.
It provides condensed and sparse MPC formulations, input trajectory parameterization through move-blocking and B-splines, OSQP-backed solves, and binary logging for simulation and analysis workflows.

## Who It Is For

`affine_mpc` is aimed at researchers, students, and engineers who want a focused MPC library rather than a general optimization toolkit.
The Python interface is convenient for experimentation, analysis, and teaching.
The C++ interface is better suited for integration into performance-sensitive applications.

## Start Here

If you are new to the project, the shortest path is:

1. read [Concepts](concepts.md) for the supported problem class
2. choose [Python Getting Started](python/getting-started.md) or [C++ Getting Started](cpp/getting-started.md)
3. run the mass-spring-damper walkthrough in [Examples](examples.md)
4. inspect the generated outputs in [Logging](logging.md)

Choose Python if you want fast iteration, NumPy-based analysis, notebooks, or teaching workflows.
Choose C++ if you want native integration, tighter runtime control, or lower-overhead deployment.

For specific tasks:

- Want to use Python: go to [Python Getting Started](python/getting-started.md)
- Want to use C++: go to [C++ Getting Started](cpp/getting-started.md)
- Want to contribute: see [Development](development.md)

## Core Workflow

No matter which interface you use, the main workflow is the same:

1. Choose a `Parameterization`
2. Construct `CondensedMPC` or `SparseMPC`
3. Set model, limits, weights, and references
4. Call `initializeSolver()`
5. Call `solve(...)` in a loop
6. Retrieve inputs and predicted trajectories
7. Optionally log results with `MPCLogger`

## More Documentation

- [Python](python/index.md): Python package overview and workflow
- [C++](cpp/index.md): C++ library overview and workflow
- [Development](development.md): contributor workflow and repository conventions
