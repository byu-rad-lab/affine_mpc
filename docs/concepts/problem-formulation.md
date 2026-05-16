# Problem Formulation

This page defines the finite-horizon MPC problem solved by `affine_mpc`.

## Discrete-Time Affine Model

The system model is assumed to be discrete-time, affine, and time-invariant:

$$
\mathbf{x}_{k+1} = A \mathbf{x}_k + B \mathbf{u}_k + \mathbf{w},
$$

## Base Problem

$$
\begin{align}
\min_{\mathbf{c}}
    &\quad J_x = \left\lVert \bar{\mathbf{x}}_T - \mathbf{x}_T \right\rVert^2_{Q_f}
    + \sum_{k=1}^{T-1} \left\lVert \bar{\mathbf{x}}_k - \mathbf{x}_k \right\rVert^2_Q
    \\
\text{s.t.}
    &\quad \mathbf{x}_{k+1} = A \mathbf{x}_k + B \mathbf{u}_k + \mathbf{w} \\
    &\quad \mathbf{u}_k = g_k(\mathbf{c}_0, \dots, \mathbf{c}_{\eta-1}) \\
    &\quad \mathbf{u}_{\min} \leq \mathbf{c}_i \leq \mathbf{u}_{\max} \quad \\
\end{align}
$$

This generates a dynamically feasible input trajectory by minimizing state tracking error while respecting input limits.
In `affine_mpc`, the parameterization mapping function $g_k$ is implemented with B-splines,
so saturating the control points saturates the entire input trajectory as well
(due to the convex hull property of B-splines).

#### Nomenclature

--8<-- "concepts/nomenclature.md:mpc-terms"

## Optional Terms

### Input Cost

$$
J_u = \sum_{i=0}^{\eta-1} \left\lVert \bar{\mathbf{c}}_i - \mathbf{c}_i \right\rVert^2_R
$$

In unconstrained LQR-style formulations,
an input penalty is typically used to regularize control effort,
whereas MPC can also enforce hard input limits directly without an input regularizing term.

**When you may want to use it:**

- You specifically want to minimize control effort or energy (set $\bar{\mathbf{c}}=0$)
- You want to stay near an equilibrium value (set $\bar{\mathbf{c}}=\mathbf{u}_e$)
- You have a reference input trajectory, perhaps from using differential flatness

!!! note

    `affine_mpc` currently only supports setting reference control points rather than a full reference input trajectory.
    For all of our use cases thus far, regularizing only the control points has been sufficient.

    If you need that functionality then please submit an issue or, better yet, a pull request!

### Input Trajectory Saturation Constraint

$$
\mathbf{u}_{\min} \leq \mathbf{u}_k \leq \mathbf{u}_{\max} \quad k=0,\dots,T-1
$$

The default is to saturate the control points only, which saturates the entire input trajectory due to the convex hull property of B-splines.
However, the higher the degree of the spline the harder it becomes for the parameterization to utilize the full input range (it essentially becomes flattened).
Saturating each input in the horizon adds more optimization constraints, but it lets the control points leave the input saturation range while the evaluated inputs remain saturated.

**When you may want to use it:**

- When using a spline with degree higher than 1 to prevent input trajectory flattening

See [Input Parameterization](input-parameterization.md) for more details on B-splines.

### State Saturation Constraint

$$
\mathbf{x}_{\min} \leq \mathbf{x}_k \leq \mathbf{x}_{\max} \quad k=1,\dots,T
$$

Enable this to set state saturation limits.

**When you may want to use it:**

- Your system has physical state limits
- You want to provide extra stability (e.g., setting roll limits for an aircraft)

!!! danger

    If your current state starts outside of the state limits,
    perhaps due to a disturbance,
    then this constraint can cause the solver to immediately fail.

### Initial Input Slew Rate Constraint

$$
|\mathbf{u}_0 - \mathbf{u}_{{-}1}| \leq \mathbf{u}_{0,\mathrm{slew}}
$$

This constrains the difference between the previous input and the first input in the horizon $\mathbf{u}_0$
(which is the next to apply).

**When you may want to use it:**

- Your system is physically limited in how fast it can change inputs

### Control Point Slew Rate Constraint

$$
|\mathbf{c}_i - \mathbf{c}_{i-1}| \leq \mathbf{c}_{\mathrm{slew}} \quad i=1,\dots,\eta-1
$$

This constrains the difference between consecutive control points.
Slew rates are used to smooth out trajectories,
preventing potentially harmful input spikes and often yielding smoother or less aggressive actuation.
This is more natural for degree 0 and 1 splines to essentially set a slew rate constraint between evaluated inputs.
It can still be used for higher degree splines, but the result may not be as intuitive.

**When you may want to use it:**

- You want to try to smooth the input trajectory
- Your system is physically limited in how fast it can change inputs

## From MPC to QP

The next two pages provide the missing steps:

1. [Input Parameterization](input-parameterization.md) derives the parameterization basis matrix $\Psi^p(\boldsymbol{\tau})$.
1. [QP Formulation](qp-formulation.md) stacks the prediction model and converts the full MPC problem into quadratic-program form.
