# Problem Formulation

This page defines the finite-horizon MPC problem solved by `affine_mpc`.
It establishes the notation used in the derivation pages that follow.

## Discrete-Time Affine Model

The plant model is assumed to be discrete-time, affine, and time-invariant:

$$
\mathbf{x}_{k+1} = A \mathbf{x}_k + B \mathbf{u}_k + \mathbf{w},
$$

where:

- $\mathbf{x}_k \in \mathbb{R}^n$ is the state at step $k$
- $\mathbf{u}_k \in \mathbb{R}^m$ is the applied input at step $k$
- $A \in \mathbb{R}^{n \times n}$ is the state transition matrix
- $B \in \mathbb{R}^{n \times m}$ is the input matrix
- $\mathbf{w} \in \mathbb{R}^n$ is the affine offset term

The affine offset makes the formulation suitable for linearized systems with bias terms,
disturbance models,
or exact discrete-time lifts of continuous systems that include a constant input-independent offset.

## Horizon and Indexing

The prediction horizon contains $T$ discrete steps.
The current measured state is treated as $\mathbf{x}_0$.
Predicted future states are $\mathbf{x}_1, \dots, \mathbf{x}_T$.

The dense input trajectory is

$$
\mathbf{u}_0, \mathbf{u}_1, \dots, \mathbf{u}_{T-1},
$$

but `affine_mpc` usually optimizes a reduced control-point trajectory

$$
\boldsymbol{\nu}_0, \boldsymbol{\nu}_1, \dots, \boldsymbol{\nu}_{\eta-1},
$$

with $\eta \leq T$.

Throughout this page,
$k$ indexes dense horizon inputs $\mathbf{u}_0, \dots, \mathbf{u}_{T-1}$,
while $i$ indexes control points $\boldsymbol{\nu}_0, \dots, \boldsymbol{\nu}_{\eta-1}$.

## Reference Quantities

The state reference may be supplied as either:

- a step reference $\bar{\mathbf{x}}$ used across the full horizon, or
- a stacked reference trajectory $\bar{\mathbf{x}}_1, \dots, \bar{\mathbf{x}}_T$

If input tracking cost is enabled,
the control-point references may also be supplied as either:

- a step reference $\bar{\boldsymbol{\nu}}$, or
- a parameterized reference trajectory $\bar{\boldsymbol{\nu}}_0, \dots, \bar{\boldsymbol{\nu}}_{\eta-1}$

The overbar notation always denotes a reference value.

## Base Finite-Horizon Problem

Ignoring optional costs and constraints for the moment,
the base MPC problem is

$$
\begin{aligned}
\min_{\boldsymbol{\nu}_0,\dots,\boldsymbol{\nu}_{\eta-1}}
    &\quad J_x = \sum_{k=1}^{T-1} \lVert \mathbf{x}_k - \bar{\mathbf{x}}_k \rVert_Q^2
      + \lVert \mathbf{x}_T - \bar{\mathbf{x}}_T \rVert_{Q_f}^2 \\
\text{s.t.}
    &\quad \mathbf{x}_{k+1} = A \mathbf{x}_k + B \mathbf{u}_k + \mathbf{w},
      \qquad k = 0, \dots, T-1, \\
    &\quad \mathbf{u}_k = \sum_{i=0}^{\eta-1} b_i^p(k; \boldsymbol{\tau}) \, \boldsymbol{\nu}_i,
      \qquad k = 0, \dots, T-1.
\end{aligned}
$$

In stacked form,
the input parameterization constraint is

$$
\mathbf{U} = M \boldsymbol{\nu}.
$$

The remaining subsections describe the optional terms controlled by `Options`.

## Cost Function

The base objective is state tracking with a terminal cost:

$$
J_x = \sum_{k=1}^{T-1} \lVert \mathbf{x}_k - \bar{\mathbf{x}}_k \rVert_Q^2
      + \lVert \mathbf{x}_T - \bar{\mathbf{x}}_T \rVert_{Q_f}^2.
$$

The weighted norm used throughout is

$$
\lVert \mathbf{y} \rVert_M^2 = \mathbf{y}^\top M \mathbf{y}.
$$

In the current API,
$Q$,
$Q_f$,
and $R$ are provided as diagonal matrices through their diagonal vectors.

### Optional Input Cost

If `Options::use_input_cost` is enabled,
the objective includes the additional term

$$
J_u = \sum_{i=0}^{\eta-1} \lVert \boldsymbol{\nu}_i - \bar{\boldsymbol{\nu}}_i \rVert_R^2.
$$

This penalty is defined over control points,
not over the dense input samples $\mathbf{u}_k$.

## Input Parameterization Constraint

The dense input trajectory is generated from control points by a linear evaluation map

$$
\mathbf{U} = M \boldsymbol{\nu},
$$

where:

- $\mathbf{U} \in \mathbb{R}^{mT}$ is the stacked dense input trajectory
- $\boldsymbol{\nu} \in \mathbb{R}^{m\eta}$ is the stacked control-point vector
- $M \in \mathbb{R}^{mT \times m\eta}$ is the sampled spline evaluation matrix built from the chosen parameterization

The full derivation of $M$ is given in [Input Parameterization](input-parameterization.md).

## Optional Constraints

Depending on `Options`,
the optimization may include the following additional constraints.

### Optional Input Bounds

Input limits may be imposed either on the control points or on the sampled dense input trajectory.

If `Options::saturate_input_trajectory` is disabled,
the bounds apply to the control points:

$$
\mathbf{u}_{\min} \leq \boldsymbol{\nu}_i \leq \mathbf{u}_{\max},
\qquad i = 0, \dots, \eta-1.
$$

If `Options::saturate_input_trajectory` is enabled,
the bounds apply to the dense trajectory samples:

$$
\mathbf{u}_{\min} \leq \mathbf{u}_k \leq \mathbf{u}_{\max},
\qquad k = 0, \dots, T-1.
$$

This distinction matters because the first form constrains the parameterization variables directly,
while the second constrains the evaluated input trajectory.

### Optional State Bounds

If `Options::saturate_states` is enabled,
the predicted states satisfy

$$
\mathbf{x}_{\min} \leq \mathbf{x}_k \leq \mathbf{x}_{\max},
\qquad k = 1, \dots, T.
$$

These bounds apply only to predicted states,
not to the measured initial state $\mathbf{x}_0$.

### Optional Initial Slew-Rate Constraint

If `Options::slew_initial_input` is enabled,
the first horizon input is constrained relative to the previously applied input:

$$
|\mathbf{u}_0 - \mathbf{u}_{-1}| \leq \mathbf{u}_{0,\mathrm{slew}}.
$$

Here $\mathbf{u}_{-1}$ denotes the previously applied input,
which is stored internally and updated after each solve.

### Optional Control-Point Slew-Rate Constraint

If `Options::slew_control_points` is enabled,
consecutive control points satisfy

$$
|\boldsymbol{\nu}_{i+1} - \boldsymbol{\nu}_i| \leq \boldsymbol{\nu}_{\mathrm{slew}},
\qquad i = 0, \dots, \eta-2.
$$

This constraint is imposed in control-point space rather than on successive dense input samples.

## Decision Variables

The sparse and condensed formulations differ in what is optimized directly.

### Sparse formulation

The sparse formulation retains predicted states as optimization variables,
so a typical decision vector has the form

$$
\mathbf{z}_{\mathrm{sparse}} =
\begin{bmatrix}
\boldsymbol{\nu} \\
\mathbf{x}_1 \\
\vdots \\
\mathbf{x}_T
\end{bmatrix}.
$$

### Condensed formulation

The condensed formulation eliminates the predicted states analytically using the model equations,
so the optimization variables are only the control points:

$$
\mathbf{z}_{\mathrm{condensed}} = \boldsymbol{\nu}.
$$

The dense predicted states remain available after a solve,
but they are recovered from the solved control-point vector rather than being optimized directly.

## From MPC to QP

The next two pages provide the missing steps:

1. [Input Parameterization](input-parameterization.md) derives the matrix $M$.
1. [QP Formulation](qp-formulation.md) stacks the prediction model and converts the full MPC problem into quadratic-program form.
