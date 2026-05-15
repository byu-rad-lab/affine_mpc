# Runtime Updates

This page explains which MPC data may change between solves,
why `initializeSolver()` is a structural boundary,
and how OSQP's fixed sparsity pattern limits runtime updates.

The main assumption is that $\mathbf{x}_0$ changes between solves,
so it is the only parameter passed directly to `solve(x0)`.
Other MPC data can be updated through setter methods,
but those updates must preserve the QP structure fixed by `initializeSolver()`.

## Configure Then Initialize

The most important lifecycle rule in `affine_mpc` is:

1. construct the MPC object
1. set model, limits, weights, and references
1. call `initializeSolver()`
1. solve repeatedly while applying only updates that preserve sparsity structure

The reason is not merely stylistic.
Calling `initializeSolver()` fixes the QP structure that OSQP will track internally.
After that point,
runtime updates may change values but not the initialized sparsity structure.

## What OSQP Fixes at Initialization

OSQP solves quadratic programs of the form

$$
\begin{aligned}
\min_z &\quad \frac{1}{2} \mathbf{z}^\top P \mathbf{z} + \mathbf{q}^\top \mathbf{z} \\
\text{s.t.} &\quad \boldsymbol{\ell} \le A_\mathrm{qp} \mathbf{z} \le \boldsymbol{\upsilon}.
\end{aligned}
$$

After initialization,
OSQP allows the values in these matrices and vectors to be updated,
but it does not allow the sparsity pattern of $P$ or $A_\mathrm{qp}$ to change.

In practice this means:

- existing nonzero entries may change value
- new nonzero positions cannot be introduced through runtime updates

## Which User Inputs Affect QP Structure?

The QP matrices $P$ and $A_\mathrm{qp}$ are affected by:

- model terms: $A$, $B$, $w$
- weights: $Q$, $Q_f$, $R$
- enabled options and constraint structure
- parameterization structure through the chosen `Parameterization`

`Options` and `Parameterization` do not change at runtime,
but they determine the QP structure within which runtime updates occur.
The runtime-updated inputs that can affect QP sparsity are the model terms and weights.
To preserve sparsity structure,
ensure that any model or weight updates you expect to make later will not introduce new nonzero entries into $P$ or $A_\mathrm{qp}$.

For example, it is common to compute Jacobians of a nonlinear model for $A$, $B$, and $w$.
Often when evaluating those Jacobians at equilibrium points, many entries are exactly zero,
but they may become nonzero when evaluated at other points.
In that case,
initialize the solver using a model whose sparsity pattern covers the operating conditions you expect,
then update the model to the current operating point before the first call to `solve()`.

!!! note

    OSQP exploits sparsity for efficiency,
    so coefficients that are truly zero throughout the expected operating range should remain exactly zero at initialization.
    Include entries that may become active later,
    but avoid making matrices denser than necessary just to be conservative.

## What Can Change Between Solves?

Provided the initialized sparsity pattern of QP matrices is preserved,
the following can be updated between solves.
Entries related to optional constraints assume the corresponding constraint family was enabled before `initializeSolver()`.

| Parameter                                            | Update Method                                             |
| ---------------------------------------------------- | --------------------------------------------------------- |
| $\mathbf{x}_0$                                       | `solve()`                                                 |
| $A$, $B$, $\mathbf{w}$                               | `setModelDiscrete()`, `setModelContinuous2Discrete()`     |
| $\bar{\mathbf{x}}$                                   | `setReferenceState()`, `setReferenceStateTrajectory()`    |
| $\bar{\mathbf{c}}$                                   | `setReferenceInput()`, `setReferenceInputControlPoints()` |
| $Q$, $Q_f$                                           | `setWeights()`, `setStateWeights()`                       |
| $R$                                                  | `setWeights()`, `setInputWeights()`                       |
| $\mathbf{x}_\mathrm{min}$, $\mathbf{x}_\mathrm{max}$ | `setStateLimits()`                                        |
| $\mathbf{u}_\mathrm{min}$, $\mathbf{u}_\mathrm{max}$ | `setInputLimits()`                                        |
| $\mathbf{u}_{0,\mathrm{slew}}$                       | `setSlewRateInitial()`                                    |
| $\mathbf{c}_\mathrm{slew}$                           | `setSlewRate()`                                           |

## MPC to QP Relationship

The following tables summarize how MPC parameter updates affect QP parameters.
These tables show which OSQP data may change for each MPC update.
Updates that touch QP matrices are typically heavier than vector-only updates,
but the actual cost depends on the formulation and problem structure.

**Sparse:**

| MPC Term       | $P$ | $A_\mathrm{qp}$ | $q$ | $\boldsymbol{\ell}$ | $\boldsymbol{\upsilon}$ |
| -------------- | --- | --------------- | --- | ------------------- | ----------------------- |
| $\mathbf{x}_0$ |     |                 |     | ✓                   | ✓                       |
| $A$            |     | ✓               |     | ✓                   | ✓                       |
| $B$            |     | ✓               |     |                     |                         |
| $\mathbf{w}$   |     |                 |     | ✓                   | ✓                       |
| $Q$, $Q_f$     | ✓   |                 | ✓   |                     |                         |
| $R$            | ✓   |                 | ✓   |                     |                         |

**Condensed:**

| MPC Term       | $P$ | $A_\mathrm{qp}$ | $q$ | $\boldsymbol{\ell}$ | $\boldsymbol{\upsilon}$ |
| -------------- | --- | --------------- | --- | ------------------- | ----------------------- |
| $\mathbf{x}_0$ |     |                 | ✓   | xsat                | xsat                    |
| $A$            | ✓   |                 | ✓   | xsat                | xsat                    |
| $B$            | ✓   |                 | ✓   | xsat                | xsat                    |
| $\mathbf{w}$   | ✓   |                 | ✓   | xsat                | xsat                    |
| $Q$, $Q_f$     | ✓   |                 | ✓   |                     |                         |
| $R$            | ✓   |                 | ✓   |                     |                         |

!!! note

    The model and initial state only affect
    $\boldsymbol{\ell}$ and $\boldsymbol{\upsilon}$
    if state saturation constraints are enabled.

**Shared:**

| MPC Term                       | $P$ | $A_\mathrm{qp}$ | $q$ | $\boldsymbol{\ell}$ | $\boldsymbol{\upsilon}$ |
| ------------------------------ | --- | --------------- | --- | ------------------- | ----------------------- |
| $\bar{\mathbf{x}}$             |     |                 | ✓   |                     |                         |
| $\bar{\mathbf{c}}$             |     |                 | ✓   |                     |                         |
| $\mathbf{x}_\mathrm{min}$      |     |                 |     | ✓                   | ✓                       |
| $\mathbf{x}_\mathrm{max}$      |     |                 |     | ✓                   | ✓                       |
| $\mathbf{u}_\mathrm{min}$      |     |                 |     | ✓                   | ✓                       |
| $\mathbf{u}_\mathrm{max}$      |     |                 |     | ✓                   | ✓                       |
| $\mathbf{u}_{0,\mathrm{slew}}$ |     |                 |     | ✓                   | ✓                       |
| $\mathbf{c}_\mathrm{slew}$     |     |                 |     | ✓                   | ✓                       |

## Related Topics

- [QP Formulation](qp-formulation.md): how MPC data maps into the OSQP problem
- [Usage](../usage.md): configuring an MPC object and running repeated solves
- [Examples](../examples.md): complete repeated-solve workflows in context
