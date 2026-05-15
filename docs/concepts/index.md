# Concepts

This section documents the mathematical structure behind `affine_mpc`.
It is shared between the C++ and Python interfaces.

## Supported Optimization Problem

`affine_mpc` assumes a discrete-time affine, time-invariant model

$$
\mathbf{x}_{k+1} = A \mathbf{x}_k + B \mathbf{u}_k + \mathbf{w},
$$

and solves a finite-horizon tracking MPC problem in which the dense input trajectory is parameterized by control points through some mapping
$\mathbf{u}_k = g_k(\mathbf{c}_0, \dots, \mathbf{c}_{\eta-1})$.
The same mathematical problem supports both library formulations:
`CondensedMPC` optimizes only the control points,
while `SparseMPC` keeps the predicted states in the QP decision vector as well.

The library supports the following cost and constraint structure,
with several optional terms controlled by `Options` (highlighted in red):

$$
\begin{align}
\min
    &\quad J = \left\lVert \bar{\mathbf{x}}_T - \mathbf{x}_T \right\rVert^2_{Q_f}
    + \sum_{k=1}^{T-1} \left\lVert \bar{\mathbf{x}}_k - \mathbf{x}_k \right\rVert^2_Q
    + \underbrace{
        \sum_{i=0}^{\eta-1} \left\lVert \bar{\mathbf{c}}_i - \mathbf{c}_i \right\rVert^2_R
      }_{\textcolor{red}{\text{input cost}}} \\
\text{w.r.t.}
    &\quad \mathbf{c}_0, \dots ,\mathbf{c}_{\eta-1} \quad \underbrace{\mathbf{x}_1, \dots, \mathbf{x}_T}_{\textcolor{magenta}{\text{sparse only}}}\\
\text{s.t.}
    &\quad \mathbf{x}_{k+1} = A \mathbf{x}_k + B \mathbf{u}_k + \mathbf{w} \\
    &\quad \mathbf{u}_k = g_k(\mathbf{c}_0, \dots, \mathbf{c}_{\eta-1}) \\
    &\quad \mathbf{u}_{\min} \leq \mathbf{c}_i \leq \mathbf{u}_{\max} \quad \text{OR} \quad
        \underbrace{\mathbf{u}_{\min} \leq \mathbf{u}_k \leq \mathbf{u}_{\max}}_{\textcolor{red}{\text{saturate input trajectory}}} \\
    &\quad \underbrace{\mathbf{x}_{\min} \leq \mathbf{x}_k \leq \mathbf{x}_{\max}}_{\textcolor{red}{\text{saturate states}}} \\
    &\quad \underbrace{|\mathbf{u}_0 - \mathbf{u}_{{-}1}| \leq \mathbf{u}_{0,\mathrm{slew}}}_{\textcolor{red}{\text{initial slew rate}}} \\
    &\quad \underbrace{|\mathbf{c}_i - \mathbf{c}_{i-1}| \leq \mathbf{c}_{\mathrm{slew}}}_{\textcolor{red}{\text{slew control points}}}
\end{align}
$$

The weighted norm used above is:

$$
\left\lVert \mathbf{y} \right\rVert^2_M = \mathbf{y}^\top M \mathbf{y}
$$

--8<-- "concepts/nomenclature.md:mpc-terms"

## Section Guide

- [Problem Formulation](problem-formulation.md): explanation of cost and constraint functions and when to use them.
- [Input Parameterization](input-parameterization.md): supported parameterizations and how they are implemented through B-splines.
- [QP Formulation](qp-formulation.md): conversion from MPC to QP.
- [Runtime Updates](runtime-updates.md): OSQP sparsity rules and what may change between solves.

<!-- ## Practical Takeaway -->
<!---->
<!-- For day-to-day use, -->
<!-- the most important rules are: -->
<!---->
<!-- 1. Choose a parameterization and `Options` before constructing the MPC object. -->
<!-- 1. Set model, limits, weights, and references before calling `initializeSolver()`. -->
<!-- 1. After initialization, updates may change values but must not introduce new nonzero entries into the QP matrices tracked by OSQP. -->
<!---->
<!-- The deeper derivations on the following pages explain why these rules exist and how they map to the implementation. -->
