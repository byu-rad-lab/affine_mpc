# affine_mpc

`affine_mpc` is a library for model predictive control of discrete-time affine systems with equal first-class interfaces in C++ and Python. It provides condensed and sparse MPC formulations, input trajectory parameterization through move-blocking and B-splines, OSQP-backed solves, and binary logging for simulation and analysis workflows.

## Who It Is For

`affine_mpc` is aimed at researchers, students, and engineers who want a focused MPC library rather than a general optimization toolkit. The Python interface is convenient for experimentation, analysis, and teaching. The C++ interface is better suited for integration into performance-sensitive applications.

## What To Do First

If you are evaluating the project for research use or review, the shortest path is:

1. read [Concepts](concepts.md) for the supported problem class
2. choose [Python](python/getting-started.md) or [C++](cpp/getting-started.md)
3. run the mass-spring-damper example from [Examples](examples.md)
4. inspect the generated outputs with [Logging](logging.md)

## Start Here

- New to the project: read [Concepts](concepts.md)
- Want to use Python: go to [Python Getting Started](python/getting-started.md)
- Want to use C++: go to [C++ Getting Started](cpp/getting-started.md)
- Want to run the example workflow: see [Examples](examples.md)
- Want to inspect logged output: see [Logging](logging.md)
- Want to contribute: see [Development](development.md)

## Choose Your Interface

### Python

Use the Python package if you want:

- fast iteration from scripts or notebooks
- easy integration with NumPy-based workflows
- plotting and post-processing in the same environment

See [Python Getting Started](python/getting-started.md).

### C++

Use the C++ library if you want:

- direct integration into an existing CMake project
- tighter control over memory and runtime behavior
- lower-overhead deployment outside the Python runtime

See [C++ Getting Started](cpp/getting-started.md).

## Core Workflow

No matter which interface you use, the main workflow is the same:

1. Choose a `Parameterization`
2. Construct `CondensedMPC` or `SparseMPC`
3. Set model, limits, weights, and references
4. Call `initializeSolver()`
5. Call `solve(...)` in a loop
6. Retrieve inputs and predicted trajectories
7. Optionally log results with `MPCLogger`

## Documentation Map

- [Concepts](concepts.md): optimization problem, parameterization, formulations, and OSQP constraints
- [Python](python/index.md): install and use the Python package
- [C++](cpp/index.md): build and use the C++ library
- [Examples](examples.md): mass-spring-damper walkthrough in both interfaces
- [Logging](logging.md): binary logging format and analysis workflow
- [Development](development.md): contributor workflow and repository conventions
