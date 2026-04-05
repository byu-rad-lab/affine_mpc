# Python Interface

The Python package exposes the same core solver concepts as the C++ library through pybind11 bindings.
It is intended for rapid experimentation, analysis, teaching, and research workflows built around NumPy.

## Start Here

- Installation and first example: [Getting Started](getting-started.md)
- Recommended workflow and core classes: [Usage](usage.md)
- Shared concepts and terminology: [Concepts](../concepts.md)
- End-to-end example: [Examples](../examples.md)

## What the Python Interface Includes

- `CondensedMPC`
- `SparseMPC`
- `Parameterization`
- `Options`
- `OSQPSettings`
- `SolveStatus`
- `MPCLogger`

The Python API mirrors the C++ API closely.
The main differences are the use of NumPy arrays for data exchange and the availability of overloads that either return arrays directly or write into arrays you provide.
