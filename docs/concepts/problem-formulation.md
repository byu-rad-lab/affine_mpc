# Problem Formulation

This page defines the finite-horizon MPC problem solved by `affine_mpc`.

## Discrete-Time Affine Model

The system model is assumed to be discrete-time, affine, and time-invariant:

$$
\mathbf{x}_{k+1} = A \mathbf{x}_k + B \mathbf{u}_k + \mathbf{w},
$$

where $\mathbf{w}$ is the model offset.
It holds everything that does not depend on the state or input, such as gravity, the operating point of an affine approximation, or a disturbance estimate (see [Modeling Patterns](modeling-patterns.md)).

## Base Problem

$$
\begin{align}
\min_{\mathbf{c}}
    &\quad J_x = \left\lVert \mathbf{x}_{\mathrm{ref},T} - \mathbf{x}_T \right\rVert^2_{Q_{\mathrm{f}}}
    + \sum_{k=1}^{T-1} \left\lVert \mathbf{x}_{\mathrm{ref},k} - \mathbf{x}_k \right\rVert^2_Q
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
A [custom basis](input-parameterization.md#custom-bases) has no such property, so its sampled inputs are always saturated instead.

#### Nomenclature

--8<-- "concepts/nomenclature.md:mpc-terms"

## Optional Terms

### Input Cost

$$
J_u = \sum_{i=0}^{\eta-1} \left\lVert \mathbf{c}_{\mathrm{ref},i} - \mathbf{c}_i \right\rVert^2_R
$$

In unconstrained LQR-style formulations,
an input penalty is typically used to regularize control effort,
whereas MPC can also enforce hard input limits directly without an input regularizing term.

**When you may want to use it:**

- You specifically want to minimize control effort or energy (set $\mathbf{c}_{\mathrm{ref}}=0$)
- You want to stay near an equilibrium value (set $\mathbf{c}_{\mathrm{ref}}=\mathbf{u}_e$)
- You have a reference input trajectory, perhaps from using differential flatness

!!! note

    `affine_mpc` currently only supports setting reference control points rather than a full reference input trajectory.
    For all of our use cases thus far, regularizing only the control points has been sufficient.

    If you need that functionality then please submit an issue or, better yet, a pull request!

### Input Rate Cost

$$
J_{\Delta u} = \sum_{k=0}^{T-1} \left\lVert \mathbf{u}_k - \mathbf{u}_{k-1} \right\rVert^2_{R_\Delta}
$$

This penalizes changes of the sampled inputs, starting from the previous input $\mathbf{u}_{-1}$.
Enable it with `use_input_rate_cost` and set $R_\Delta$ with `setInputRateWeights()` (it defaults to ones).
The previous input is stored after every solve that returns `Success` or `SolvedInaccurate`; call `setPreviousInput()` before the first solve, or after a failed solve if you applied a different input.

**When you may want to use it:**

- You want smooth inputs, e.g. to limit actuator wear or excite fewer high-frequency modes, without the hard limit of a slew rate constraint
- You compare parameterizations: unlike $R$, which weights the control points, $R_\Delta$ acts on the sampled inputs, so it penalizes the same input change equally for every basis, including a [custom basis](input-parameterization.md#custom-bases)

### Input Trajectory Saturation Constraint

$$
\mathbf{u}_{\min} \leq \mathbf{u}_k \leq \mathbf{u}_{\max} \quad k=0,\dots,T-1
$$

The default is to saturate the control points only, which saturates the entire input trajectory due to the convex hull property of B-splines.
However, when the spline does not pass through its control points at the sample times, it becomes harder for the parameterization to utilize the full input range (it essentially becomes flattened).
This happens for any degree higher than 1, and for degree 1 when an active knot is not an integer.
Saturating the evaluated inputs adds more optimization constraints, but it lets the control points leave the input saturation range while the evaluated inputs remain saturated.

![Cubic spline with the same shape scaled to fit input limits [0, 1] two ways. When the control points are saturated the sampled inputs stay well inside the limits, and when every sample is saturated the control points leave the limits while the samples reach them.](../assets/input-parameterization/degree3-saturation.svg)
Saturating the control points (left) versus every sample (right) of a cubic spline.
Only the samples are constrained, so the continuous spline may cross a limit between samples.

How many constraints are added depends on the degree $p$:

- $p = 0$: the option is ignored, since the control points are the evaluated inputs.
- $p = 1$: the spline is linear between knots, so only the samples $\lfloor t \rfloor$ and $\lceil t \rceil$ adjacent to each active knot $t$ are constrained (duplicates removed).
  This is exactly equivalent to constraining every sample and adds at most $\min(T, 2\eta - 2)$ constraints per input.
  With integer knots it is the same as saturating the control points.
- $p > 1$: every sample $k = 0, \dots, T-1$ is constrained.
- A [custom basis](input-parameterization.md#custom-bases) always constrains every sample, whether or not the option is set.

![Degree 1 spline with active knots at 0, 2.5, 6.5, and 12 and input limits [0, 1]. Saturating the control points keeps the samples next to the non-integer knots off the limits, while saturating only the knot-adjacent samples lets the control points leave the limits so the samples reach them.](../assets/input-parameterization/degree1-saturation.svg)
Saturating the control points (left) versus the knot-adjacent samples (right) of a degree 1 spline with non-integer knots.

**When you may want to use it:**

- When using a spline with degree higher than 1 to prevent input trajectory flattening
- When using a linear spline (degree 1) with non-integer knots

See [Input Parameterization](input-parameterization.md) for more details on B-splines.

### State Saturation Constraint

$$
\mathbf{x}_{\min} \leq \mathbf{x}_k \leq \mathbf{x}_{\max} \quad k=1,\dots,T
$$

Enable this to set state saturation limits.

**When you may want to use it:**

- Your system has physical state limits
- You want to provide extra stability (e.g., setting roll limits for an aircraft)

Often only a few states need limits (e.g. roll and pitch of a quadrotor with 12 states).
List them in `bounded_states`; the limits then have one entry per listed state.
Every bounded state adds one constraint row per horizon step, and with the condensed formulation each row is dense, so bounding only the states that need it keeps the QP much smaller (in one 12-state example, bounding 2 states through infinite limits on the other 10 made the condensed solves 12 to 14 times slower than no state limits at all).

!!! danger

    If your current state starts outside of the state limits,
    perhaps due to a disturbance,
    then this constraint can cause the solver to immediately fail.
    Soft state limits avoid this.

### Soft State Limits

$$
\mathbf{x}_{\min} - \mathbf{s} \leq \mathbf{x}_k \leq \mathbf{x}_{\max} + \mathbf{s}, \quad \mathbf{s} \geq \mathbf{0},
\qquad
J_s = \boldsymbol{\rho}_1^\top \mathbf{s} + \left\lVert \mathbf{s} \right\rVert^2_{\operatorname{diag}(\boldsymbol{\rho}_2)}
$$

With `soft_state_bounds`, one slack per bounded state, shared over the horizon, may widen both of its limits at the cost $J_s$, set with `setStateBoundPenalties(l1, l2)` (there is no default).
The slack is the largest violation of its state over the horizon.

- The L1 penalty $\boldsymbol{\rho}_1$ is exact: when it exceeds the limit's Lagrange multiplier, the solution equals the hard-limit solution whenever the limits can be met, and the limits are only violated when they can not.
- The L2 penalty $\boldsymbol{\rho}_2$ alone always allows a small violation of an active limit (smaller for a larger penalty).
  With the L1 penalty it does not change the solution while the limits can be met.
- OSQP converges much faster with an L2 penalty on the scale of the state weights or larger: in one example, an L1 penalty of 10 with an L2 penalty of 0.01 took about 4000 iterations, and with an L2 penalty of 100 it took 250, about the same as hard limits.
  A very large L1 penalty also slows OSQP.
- Polishing (in the solver settings) makes the solution match the hard-limit solution to round-off when the limits can be met.

**When you may want to use it:**

- Disturbances or model error can push the state past a limit, where hard limits would make the problem infeasible
- Some limits are preferences rather than physical boundaries

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
It requires a B-spline parameterization: with a [custom basis](input-parameterization.md#custom-bases), differences of the coefficients do not bound input changes, so the MPC rejects `slew_control_points` at construction.

**When you may want to use it:**

- You want to try to smooth the input trajectory
- Your system is physically limited in how fast it can change inputs

### General Linear Constraints

$$
\mathbf{h}_{\min} \leq H_x \mathbf{x}_k + H_u \mathbf{u}_k \leq \mathbf{h}_{\max} \quad k=0,\dots,T-1,
\qquad
\mathbf{h}_{\mathrm{f},\min} \leq H_{\mathrm{f}} \mathbf{x}_T \leq \mathbf{h}_{\mathrm{f},\max}
$$

Set `num_general_constraints` ($n_h$) to constrain linear combinations of the state and input at every step, with `setGeneralConstraints(Hx, Hu, h_min, h_max)`, and `num_terminal_constraints` ($n_{h,\mathrm{f}}$) to constrain the final predicted state, with `setTerminalConstraints(Hf, hf_min, hf_max)`.
This is the usual stage and terminal split: $\mathbf{x}_0$ is the measured state passed to `solve()`, so the $k = 0$ constraints limit $\mathbf{u}_0$ given $\mathbf{x}_0$, each predicted state $\mathbf{x}_k$ with $k \geq 1$ is paired with $\mathbf{u}_k$, and $\mathbf{x}_T$ (which has no input) is covered by the terminal constraints.
Pass `-inf` or `inf` for a side without a bound.
Rows of the identity in $H_x$ with $H_u = 0$ (plus the same terminal rows) give state limits, and $H_x = 0$, $H_u = I$ limits every sampled input.

Changing $H_x$, $H_u$, or $H_{\mathrm{f}}$ rebuilds QP matrices and makes OSQP refactor, like a model change; changing only the bounds is cheap.
With the condensed formulation, as with the model, an entry of these matrices that may become nonzero later must be nonzero when the solver is initialized; the sparse formulation keeps every entry in its sparsity pattern (see [Runtime Updates](runtime-updates.md)).

**When you may want to use it:**

- Limits on outputs or combinations of states, e.g. $y = C\mathbf{x}$ within bounds
- Constraints that couple states and inputs, e.g. a power or friction-cone limit linearized at the operating point
- Terminal constraints, e.g. reaching a target set at the end of the horizon

!!! danger

    A constraint row without an input part (a zero row of $H_u$) only checks the measured state at $k = 0$,
    so the problem is infeasible whenever $\mathbf{x}_0$ violates it, as with hard state limits.
    OSQP can also need many iterations when more constraints are active than there are control points.

## From MPC to QP

The next two pages provide the missing steps:

1. [Input Parameterization](input-parameterization.md) derives the parameterization basis matrix $\Psi^p(\boldsymbol{\tau})$.
1. [QP Formulation](qp-formulation.md) stacks the prediction model and converts the full MPC problem into quadratic-program form.
