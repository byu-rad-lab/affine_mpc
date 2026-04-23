# affine_mpc

`affine_mpc` is a library for model predictive control of discrete-time affine systems, with C++ and Python interfaces.
It provides condensed and sparse MPC formulations, input trajectory parameterization through B-splines, OSQP-backed solves, and binary logging for simulation and analysis workflows.

## Who It Is For

`affine_mpc` is aimed at researchers, students, and engineers who want a focused MPC library rather than a general optimization toolkit.
The Python interface is convenient for experimentation, analysis, and teaching.
The C++ interface is better suited for integration into performance-sensitive applications.
It is intended to lower the barrier to developing MPC controllers for discrete-time affine systems by packaging common formulations, constraints, and workflows into a focused library.

## Start Here

If you are new to the project, the shortest path is:

1. Read [Concepts](concepts/index.md) for the supported problem class.
1. Choose [Python](python/index.md) or [C++](cpp/index.md).
1. Run the mass-spring-damper walkthrough in [Examples](examples.md).
1. Inspect the generated outputs in [Logging](logging.md).

To contribute, see [Development](development.md).

Choose Python if you want fast iteration, NumPy-based analysis, notebooks, or teaching workflows.
Choose C++ if you want native integration, tighter runtime control, or lower-overhead deployment.

## Core Workflow

No matter which interface you use, the main workflow is the same:

1. Choose a `Parameterization`
1. Select desired `Options`
1. Construct `CondensedMPC` or `SparseMPC`
1. Set model, limits, weights, and references
1. Call `initializeSolver()`
1. Call `solve(...)` in a loop
1. Retrieve inputs and predicted trajectories
1. Optionally log results with `MPCLogger`
