# Concepts

This page summarizes the mathematical structure supported by `affine_mpc`, how the input trajectory parameterization works, and the practical constraints imposed by the OSQP backend. The same concepts apply to both the C++ and Python interfaces.

## Problem Class

`affine_mpc` solves model predictive control problems using a discrete-time affine model:

$$
x_{k+1} = A x_k + B u_k + w
$$

with:

- state tracking costs
- configurable input bounds
- optional input tracking costs
- optional slew-rate constraints
- optional state bounds

The library converts the MPC problem into a quadratic program solved by OSQP.

## Supported Optimization Problem

The library supports the following cost and constraint structure, with several optional terms controlled by `Options`:

$$
\begin{align}
\min
    &\quad J = \left\lVert \bar{x}_T - x_T \right\rVert^2_{Q_f}
    + \sum_{k=1}^{T-1} \left\lVert \bar{x}_k - x_k \right\rVert^2_Q
    + \underbrace{
        \sum_{i=0}^{n_c-1} \left\lVert \bar{\nu}_i - \nu_i \right\rVert^2_R
      }_{\textcolor{red}{\text{input cost}}} \\
w.r.t.
    &\quad \nu_0, \dots ,\nu_{n_c-1} \quad \underbrace{x_1, \dots, x_T}_{\textcolor{magenta}{\text{sparse only}}}\\
s.t.
    &\quad x_{k+1} = A x_k + B u_k + w \\
    &\quad u_k = g(\nu_0,...,\nu_{n_c-1}) \\
    &\quad u_{min} \leq \nu_k \leq u_{max} \quad \text{OR} \quad
        \underbrace{u_{min} \leq u_k \leq u_{max}}_{\textcolor{red}{\text{saturate input trajectory}}} \\
    &\quad \underbrace{x_{min} \leq x_k \leq x_{max}}_{\textcolor{red}{\text{saturate states}}} \\
    &\quad \underbrace{|u_0 - u_{{-}1}| \leq u_{0,slew}}_{\textcolor{red}{\text{initial slew rate}}} \\
    &\quad \underbrace{|\nu_{i+1} - \nu_i| \leq \nu_{slew}}_{\textcolor{red}{\text{slew control points}}}
\end{align}
$$

Here, `CondensedMPC` optimizes only the control points, while `SparseMPC` also includes the predicted states directly in the optimization variables.

## Symbols and Terminology

- $x \in \mathbb{R}^n$ is the system state
- $\bar{x} \in \mathbb{R}^n$ is the reference state trajectory
- $u \in \mathbb{R}^m$ is the dense per-step control input
- $\nu \in \mathbb{R}^m$ is a control point used to parameterize the dense input trajectory
- $\bar{\nu} \in \mathbb{R}^m$ is a reference control point when input cost is enabled
- $g(\cdot)$ is the trajectory evaluation map induced by the chosen B-spline parameterization
- $T$ is the number of horizon steps
- $p$ is the number of control points
- $A$, $B$, and $w$ define the affine discrete-time model
- $Q$, $Q_f$, and $R$ are positive semi-definite diagonal weight matrices

The weighted norm used above is:

$$
\left\lVert x \right\rVert^2_M = x^\top M x
$$

The control-point notation matters because the library may optimize a reduced representation of the input trajectory rather than every dense input value directly.

## State, Input, and Reference Conventions

- `x` is the system state
- `u` is the dense per-step control input
- control points are the reduced optimization variables used to parameterize the dense input trajectory
- `x_ref` is the reference state trajectory
- input reference data is only meaningful when `use_input_cost` is enabled

## Input Trajectory Parameterization

The dense input trajectory is not always optimized directly.
Instead, it may be parameterized by a smaller set of control points.

Supported parameterizations include:

- move-blocking
- linear interpolation
- clamped B-splines of arbitrary degree

This lets the user reduce the number of decision variables, smooth the control signal, and trade off fidelity versus solve time.

The `Parameterization` class stores:

- `horizon_steps`
- `degree`
- `num_control_points`
- `knots`

Factory helpers are available in both interfaces:

```cpp
auto p0 = affine_mpc::Parameterization::moveBlocking(T, nc);
auto p1 = affine_mpc::Parameterization::linearInterp(T, nc);
auto p2 = affine_mpc::Parameterization::bspline(T, degree, nc);
```

## Condensed vs Sparse MPC

`affine_mpc` provides two MPC formulations.

### CondensedMPC

- eliminates state variables analytically
- produces a smaller dense QP
- is usually the recommended default
- tends to work well for shorter horizons and moderate dimensions

### SparseMPC

- keeps both state and input variables in the optimization
- preserves more sparsity structure
- can be better for larger or more structured problems
- state saturation is simpler

If you are unsure, start with `CondensedMPC`.

## Optional Features via `Options`

The `Options` struct enables optional costs and constraints at construction time.

```cpp
affine_mpc::Options opts;
opts.use_input_cost = true;
opts.slew_initial_input = true;
opts.slew_control_points = true;
opts.saturate_states = true;
opts.saturate_input_trajectory = false;
```

These flags affect how the QP is assembled.
In practice, they behave like part of the problem structure, so they should be chosen before the solver is initialized.

## Configure-Then-Initialize Pattern

One of the most important rules in this library is:

1. construct the MPC object
2. set model, limits, weights, and references
3. initialize the solver
4. solve repeatedly while updating values as needed

This matters because OSQP fixes its matrix sparsity structure during initialization. If a model entry may become nonzero later, initialize the solver with that structure already present.

## OSQP Sparsity Constraint

OSQP allows value updates after initialization, but not structural sparsity changes.

That means:

- matrix values may be updated between solves
- new nonzero entries may not appear in `P` or `A` after `initializeSolver()`

Practical consequence:

- if a model entry may become nonzero later, it must already be nonzero when the solver is initialized

This is especially important for time-varying or re-linearized models.

## Runtime Updates Between Solves

The following can typically be updated between solves without full re-initialization:

- model values
- weights
- state references
- input references
- input limits
- state limits
- slew-rate limits

These updates are efficient only when the sparsity pattern remains unchanged.

## Logging Concepts

`MPCLogger` records simulation-time data into:

- a compressed `.npz` data file
- a YAML parameter file

It stores actual values, predictions, references, and metadata in a format that is easy to inspect from Python. See [Logging](logging.md) for details.
