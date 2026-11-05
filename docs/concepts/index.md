# Concepts

This section documents the mathematical structure behind `affine_mpc`.
It is shared between the C++ and Python interfaces.

## Supported Optimization Problem

`affine_mpc` assumes a discrete-time affine, time-invariant model

$$
\bm{x}_{k+1} = A \bm{x}_k + B \bm{u}_k + \bm{w},
$$

and solves a finite-horizon tracking MPC problem in which the dense input trajectory is parameterized by control points through some mapping
$\bm{u}_k = g_k(\bm{c}_0, \dots, \bm{c}_{\eta-1})$.
The same mathematical problem supports both library formulations, chosen with the `Formulation` argument of `MPC`:
the condensed formulation (the default) optimizes only the control points,
while the sparse formulation keeps the predicted states in the QP decision vector as well.

The library supports the following cost and constraint structure,
with several optional terms controlled by `Options` (labeled in $\textcolor{red}{\text{red}}$ below):

$$
\begin{align}
\min
    &\quad J =
        \left\lVert \bm{x}_{\mathrm{ref},T} - \bm{x}_T \right\rVert^2_{Q_{\mathrm{f}}} % terminal cost
        + \sum_{k=1}^{T-1} \left\lVert \bm{x}_{\mathrm{ref},k} - \bm{x}_k \right\rVert^2_Q \\ % state stracking cost
    &\qquad\quad + \:
        \underbrace{
            \sum_{i=0}^{\eta-1} \lVert \bm{c}_{\mathrm{ref},i} - \bm{c}_i \rVert^2_R
        }_{ \textcolor{red}{\text{control point cost}} }
        \quad \text{OR} \quad
        \underbrace{
            \sum_{k=0}^{T-1} \lVert \bm{u}_{\mathrm{ref},k} - \bm{u}_k \rVert^2_R
        }_{ \textcolor{red}{\text{input trajectory cost}} } \\
    &\qquad\quad + \:
        \underbrace{
            \sum_{k=0}^{T-1} \lVert \bm{u}_k - \bm{u}_{k-1} \rVert^2_{R_\Delta}
        }_{ \textcolor{red}{\text{input rate cost}} } \\
    &\qquad\quad + \:
        \underbrace{
            \bm{\rho}_1^\T \bm{s} + \lVert \bm{s} \rVert^2_{\diag(\bm{\rho}_2)}
        }_{ \textcolor{red}{\text{soft state bounds}} } \\
\text{w.r.t.}
    &\quad \bm{c}_0, \dots ,\bm{c}_{\eta-1},
        \quad
        \underbrace{
            \bm{x}_1, \dots, \bm{x}_T
        }_{ \textcolor{magenta}{\text{sparse only}} },
        \quad
        \underbrace{
            \bm{s}
        }_{ \textcolor{red}{\text{soft state bounds}} } \\
\text{s.t.}
    &\quad \bm{x}_{k+1} = A \bm{x}_k + B \bm{u}_k + \bm{w} \\
    &\quad \bm{u}_k = g_k(\bm{c}_0, \dots, \bm{c}_{\eta-1}) \\
    &\quad \bm{u}_{\min} \leq \bm{c}_i \leq \bm{u}_{\max}
        \quad \text{OR} \quad
        \underbrace{
            \bm{u}_{\min} \leq \Omega \bm{c} \leq \bm{u}_{\max}
        }_{ \textcolor{red}{\text{refined input bounds}} }
        \quad \text{OR} \quad
        \underbrace{
            \bm{u}_{\min} \leq \bm{u}_k \leq \bm{u}_{\max}
        }_{ \textcolor{red}{\text{exact input bounds}} } \\
    &\quad \underbrace{
            \bm{x}_{\min} \leq \bm{x}_k \leq \bm{x}_{\max}
        }_{ \textcolor{red}{\text{hard state bounds}} }
        \quad \text{OR} \quad
        \underbrace{
            \bm{x}_{\min} - \bm{s} \leq \bm{x}_k \leq \bm{x}_{\max} + \bm{s}
        }_{ \textcolor{red}{\text{soft state bounds}} } \\
    &\quad \underbrace{
            |\bm{u}_0 - \bm{u}_{{-}1}| \leq \bm{u}_{0,\mathrm{slew}}
        }_{ \textcolor{red}{\text{initial slew rate}} } \\
    &\quad \underbrace{
            |\bm{c}_i - \bm{c}_{i-1}| \leq \lambda_i \bm{u}_{\mathrm{slew}}
        }_{ \textcolor{red}{\text{control point slew bounds}} }
        \quad \text{OR} \quad
        \underbrace{
            |\bm{u}_k - \bm{u}_{k-1}| \leq \bm{u}_{\mathrm{slew}}
        }_{ \textcolor{red}{\text{exact slew bounds}} } \\
    &\quad \underbrace{
            \bm{h}_{\min} \leq H_x \bm{x}_k + H_u \bm{u}_k \leq \bm{h}_{\max}
        }_{ \textcolor{red}{\text{general constraints}} } \\
    &\quad \underbrace{
            \bm{h}_{\mathrm{f},\min} \leq H_{\mathrm{f}} \bm{x}_T \leq \bm{h}_{\mathrm{f},\max}
        }_{ \textcolor{red}{\text{terminal constraints}} }
\end{align}
$$

The weighted norm used above is:

$$
\left\lVert \bm{y} \right\rVert^2_M = \bm{y}^\T M \bm{y}
$$

--8<-- "concepts/nomenclature.md:mpc-terms"

## Section Guide

- [Problem Formulation](problem-formulation.md): explanation of cost and constraint functions and when to use them.
- [Input Parameterization](input-parameterization.md): supported parameterizations and how they are implemented through B-splines.
- [QP Formulation](qp-formulation.md): conversion from MPC to QP.
- [Runtime Updates](runtime-updates.md): OSQP sparsity rules and what may change between solves.
- [Modeling Patterns](modeling-patterns.md): operating points, offset-free tracking, input delay, and deviation form.
- [Model Update Strategies](model-updates.md): when to update the model of a nonlinear plant relative to the state measurement, and where the library work goes in each case.

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
