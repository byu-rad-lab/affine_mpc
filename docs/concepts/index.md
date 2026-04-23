# Concepts

This section documents the mathematical structure behind `affine_mpc`.
It is shared between the C++ and Python interfaces.

## Supported Optimization Problem

`affine_mpc` assumes a discrete-time affine, time-invariant model

$$
\mathbf{x}_{k+1} = A \mathbf{x}_k + B \mathbf{u}_k + \mathbf{w},
$$

and solves a finite-horizon tracking MPC problem in which the dense input trajectory is parameterized by control points.
The same mathematical problem supports both library formulations:
`CondensedMPC` optimizes only the control points,
while `SparseMPC` keeps the predicted states in the QP decision vector as well.

The library supports the following cost and constraint structure,
with several optional terms controlled by `Options`:

$$
\begin{align}
\min
    &\quad J = \left\lVert \bar{\mathbf{x}}_T - \mathbf{x}_T \right\rVert^2_{Q_f}
    + \sum_{k=1}^{T-1} \left\lVert \bar{\mathbf{x}}_k - \mathbf{x}_k \right\rVert^2_Q
    + \underbrace{
        \sum_{i=0}^{\eta-1} \left\lVert \bar{\boldsymbol{\nu}}_i - \boldsymbol{\nu}_i \right\rVert^2_R
      }_{\textcolor{red}{\text{input cost}}} \\
\text{w.r.t.}
    &\quad \boldsymbol{\nu}_0, \dots ,\boldsymbol{\nu}_{\eta-1} \quad \underbrace{\mathbf{x}_1, \dots, \mathbf{x}_T}_{\textcolor{magenta}{\text{sparse only}}}\\
\text{s.t.}
    &\quad \mathbf{x}_{k+1} = A \mathbf{x}_k + B \mathbf{u}_k + \mathbf{w} \\
    &\quad \mathbf{u}_k = \sum_{i=0}^{\eta-1} b_i^p(k; \boldsymbol{\tau}) \, \boldsymbol{\nu}_i \\
    &\quad \mathbf{u}_{\min} \leq \boldsymbol{\nu}_i \leq \mathbf{u}_{\max} \quad \text{OR} \quad
        \underbrace{\mathbf{u}_{\min} \leq \mathbf{u}_k \leq \mathbf{u}_{\max}}_{\textcolor{red}{\text{saturate input trajectory}}} \\
    &\quad \underbrace{\mathbf{x}_{\min} \leq \mathbf{x}_k \leq \mathbf{x}_{\max}}_{\textcolor{red}{\text{saturate states}}} \\
    &\quad \underbrace{|\mathbf{u}_0 - \mathbf{u}_{{-}1}| \leq \mathbf{u}_{0,\mathrm{slew}}}_{\textcolor{red}{\text{initial slew rate}}} \\
    &\quad \underbrace{|\boldsymbol{\nu}_{i+1} - \boldsymbol{\nu}_i| \leq \boldsymbol{\nu}_{\mathrm{slew}}}_{\textcolor{red}{\text{slew control points}}}
\end{align}
$$

Index conventions:
$k$ indexes dense horizon inputs $\mathbf{u}_0, \dots, \mathbf{u}_{T-1}$,
predicted states are $\mathbf{x}_1, \dots, \mathbf{x}_T$,
and $i$ indexes control points $\boldsymbol{\nu}_0, \dots, \boldsymbol{\nu}_{\eta-1}$.
The control-point slew constraint applies for $i = 0, \dots, \eta-2$.

The highlighted terms indicate optional costs or constraints enabled by `Options`.
In stacked form,
the parameterization constraint is written as

$$
\mathbf{U} = M \boldsymbol{\nu},
$$

where $M$ is the sampled spline evaluation matrix.
This matrix form is the main bridge from the problem statement on this page to the QP derivation developed later in the section.

## Symbols and Terminology

### MPC Symbols

- $\mathbf{x}_k \in \mathbb{R}^n$: state at step $k$
- $\mathbf{u}_k \in \mathbb{R}^m$: dense input applied at step $k$
- $\bar{\mathbf{x}}_k$: state reference at step $k$
- $\bar{\boldsymbol{\nu}}_i$: control-point reference at index $i$ when input cost is enabled
- $n$: state dimension
- $m$: input dimension
- $T$: horizon length
- $A$, $B$, and $\mathbf{w}$: affine discrete-time model data
- $Q$, $Q_f$, and $R$: positive semi-definite diagonal weight matrices
- $\bar{(\cdot)}$: reference quantity

The weighted norm used above is:

$$
\left\lVert \mathbf{x} \right\rVert^2_M = \mathbf{x}^\top M \mathbf{x}
$$

### Parameterization Symbols

- $\boldsymbol{\nu}_i \in \mathbb{R}^m$: control point at index $i$
- $\eta$: number of control points
- $p$: spline degree
- $\mu$: number of knots
- $\boldsymbol{\tau} = (\tau_0, \dots, \tau_{\mu-1})$: knot vector
- $b_i^p(k; \boldsymbol{\tau})$: $i$th B-spline basis function of degree $p$, evaluated at horizon index $k$
- $\mathbf{U} = [\mathbf{u}_0^\top \; \mathbf{u}_1^\top \; \cdots \; \mathbf{u}_{T-1}^\top]^\top \in \mathbb{R}^{mT}$: stacked dense input trajectory
- $\boldsymbol{\nu} = [\boldsymbol{\nu}_0^\top \; \boldsymbol{\nu}_1^\top \; \cdots \; \boldsymbol{\nu}_{\eta-1}^\top]^\top \in \mathbb{R}^{m\eta}$: stacked control-point vector
- $M \in \mathbb{R}^{mT \times m\eta}$: sampled spline evaluation matrix

The dense input trajectory is parameterized rather than optimized directly.
The common stacked relation used throughout the deeper derivation pages is

$$
\mathbf{U} = M \boldsymbol{\nu},
$$

where $M$ is induced by the chosen parameterization.

### Code Terminology

- predicted state trajectory: denoted by stacked state vectors such as $\mathbf{X}$ in the derivation pages, and exposed in the API as `x_traj`
- dense input trajectory: denoted by $\mathbf{U}$ in the derivation pages, and exposed in the API as `u_traj`
- control-point trajectory: denoted by $\boldsymbol{\nu}$ in the derivation pages, and exposed in the API as `u_traj_ctrl_pts`
- next applied input: denoted by $\mathbf{u}_0$, and exposed by `getNextInput()`
- previous applied input: denoted by $\mathbf{u}_{-1}$, and set through `setPreviousInput()` when the initial slew-rate constraint is enabled

## Section Guide

- [Problem Formulation](problem-formulation.md): precise MPC problem statement and horizon-level notation
- [Input Parameterization](input-parameterization.md): B-spline basis, knot vectors, and the map from control points to dense inputs
- [QP Formulation](qp-formulation.md): stacked prediction model, quadratic cost, and the resulting QP structure
- [Runtime Updates](runtime-updates.md): OSQP sparsity rules and what may change between solves

## Practical Takeaway

For day-to-day use,
the most important rules are:

1. Choose a parameterization and `Options` before constructing the MPC object.
1. Set model, limits, weights, and references before calling `initializeSolver()`.
1. After initialization, updates may change values but must not introduce new nonzero entries into the QP matrices tracked by OSQP.

The deeper derivations on the following pages explain why these rules exist and how they map to the implementation.
