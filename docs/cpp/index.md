# C++ Interface

The C++ library is the core implementation of `affine_mpc`.
It provides the public MPC API, the OSQP-backed solver integration, and the logging utilities used by both the native and Python interfaces.

## Start Here

- Build and run your first example: [Getting Started](getting-started.md)
- Recommended workflow and main classes: [Usage](usage.md)
- Shared concepts and terminology: [Concepts](../concepts.md)
- End-to-end example: [Examples](../examples.md)

## Main Public Types

- `affine_mpc::Parameterization`
- `affine_mpc::Options`
- `affine_mpc::MPCBase`
- `affine_mpc::CondensedMPC`
- `affine_mpc::SparseMPC`
- `affine_mpc::MPCLogger`
- `affine_mpc::SolveStatus`

Public headers live in `include/affine_mpc/`.
