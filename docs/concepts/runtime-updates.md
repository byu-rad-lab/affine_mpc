# Runtime Updates

This page explains which MPC data may change between solves,
when and at what cost those changes reach the QP,
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

- model matrices: $A$, $B$
- weights: $Q$, $Q_{\mathrm{f}}$, $R$
- enabled options and constraint structure
- parameterization structure through the chosen `Parameterization`

`Options` and `Parameterization` do not change at runtime,
but they determine the QP structure within which runtime updates occur.
The runtime-updated inputs that can affect QP sparsity are the model matrices and weights.
The model offset $\mathbf{w}$ only enters the QP vectors, so it never affects sparsity.
To preserve sparsity structure,
ensure that any model or weight updates you expect to make later will not introduce new nonzero entries into $P$ or $A_\mathrm{qp}$.

For example, it is common to compute Jacobians of a nonlinear model for $A$, $B$, and $\mathbf{w}$.
Often when evaluating those Jacobians at equilibrium points, many entries are exactly zero,
but they may become nonzero when evaluated at other points.
In that case,
initialize the solver using a model whose sparsity pattern covers the operating conditions you expect,
then update the model to the current operating point before the first call to `solve()`.
Evaluating the Jacobians at a random state and input usually gives such a pattern
(a point that is merely nonzero can still give exact zeros, such as $\sin(\pi) = 0$).
If you switch between several models, such as piecewise or gain-scheduled models,
initialize with a model whose nonzeros cover all of them.

Weights in `SparseMPC` are the one exception:
`SparseMPC` places $Q$, $Q_{\mathrm{f}}$, and $R$ only on the diagonal of $P$,
which OSQP always includes in its linear system,
so `initializeSolver()` stores every weight position at no cost to solve time.
Weights in `SparseMPC` may therefore change between zero and nonzero freely.
In `CondensedMPC`, weights enter $P$ through matrix products,
so they follow the same rule as the model.

Updates that break these rules are not detected during normal operation,
because checking would add work to every update.
Values outside the initialized pattern are silently dropped,
so the solver solves a different problem than the one you set.
If you suspect this, call `isWithinSparsityPattern()` after `solve()` or `prepare()`;
it returns `false` when the current QP matrices have nonzeros outside the initialized pattern.
It scans the full QP matrices, so use it for debugging rather than in a control loop.

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
| $\mathbf{w}$ only                                    | `setModelOffsetDiscrete()`, `setModelOffsetContinuous2Discrete()` |
| $\mathbf{x}_{\mathrm{ref}}$                          | `setReferenceState()`, `setReferenceStateTrajectory()`    |
| $\mathbf{c}_{\mathrm{ref}}$                          | `setReferenceInput()`, `setReferenceInputControlPoints()` |
| $Q$, $Q_{\mathrm{f}}$                                | `setWeights()`, `setStateWeights()`                       |
| $R$                                                  | `setWeights()`, `setInputWeights()`                       |
| $\mathbf{x}_\mathrm{min}$, $\mathbf{x}_\mathrm{max}$ | `setStateLimits()`                                        |
| $\mathbf{u}_\mathrm{min}$, $\mathbf{u}_\mathrm{max}$ | `setInputLimits()`                                        |
| $\mathbf{u}_{0,\mathrm{slew}}$                       | `setSlewRateInitial()`                                    |
| $\mathbf{c}_\mathrm{slew}$                           | `setSlewRate()`                                           |
| $\mathbf{u}_\mathrm{prev}$                           | `setPreviousInput()` (also set by each successful solve)  |

## When Updates Reach the QP

Setters only validate and store their values and mark a change category.
All QP work happens in `solve()`, once per changed category, no matter how many setters were called since the last solve.
Updates that OSQP rejects (often from non-finite values) are reported by `solve()` as `UpdateFailed` and stay pending, so the next solve retries them.

The categories differ in cost:

| Category | Setters | Condensed QP work | Sparse QP work | OSQP refactors |
| --- | --- | --- | --- | --- |
| Initial state $\mathbf{x}_0$ | `solve()` | $\mathbf{v}$, $q$, state-limit bounds | first dynamics-row bounds | No |
| Model offset $\mathbf{w}$ | `setModelOffset*()` | $\mathbf{v}$, $q$, state-limit bounds | dynamics-row bounds | No |
| Model $A$, $B$ | `setModelDiscrete()`, `setModelContinuous2Discrete()` | $S$, $P$, state rows of $A_\mathrm{qp}$ | dynamics blocks of $A_\mathrm{qp}$ | Yes |
| Weights | `setWeights()`, `setStateWeights()`, `setInputWeights()` | $P$, $q$ | diagonal of $P$, $q$ | Yes |
| References | `setReference*()` | $q$ | $q$ | No |
| Limits and slew rates | `setInputLimits()`, `setStateLimits()`, `setSlewRate()`, `setSlewRateInitial()` | $\boldsymbol{\ell}$, $\boldsymbol{\upsilon}$ | $\boldsymbol{\ell}$, $\boldsymbol{\upsilon}$ | No |
| Previous input | `setPreviousInput()` | initial-slew bounds | initial-slew bounds | No |

The model setters also set $\mathbf{w}$, so they mark both model categories.
If only the model offset changes, e.g. a disturbance estimate for offset-free tracking, use `setModelOffsetDiscrete()` or `setModelOffsetContinuous2Discrete()` instead: they only update QP vectors, like references and limits.

`setModelContinuous2Discrete()` is the one setter that does real work itself: it computes the exact discretization (a matrix exponential) inside the setter, so that cost is not part of `solve()` or `prepare()` or the reported `mpc_solve_time`.
Time it as part of your control loop if the model changes every step.
`setModelOffsetContinuous2Discrete()` reuses that discretization and costs only a matrix-vector product, so it is the cheap path when only $\mathbf{w}$ changes.

## Preparing Before the Measurement

`prepare()` optionally applies every pending change that does not depend on $\mathbf{x}_0$ (all categories in the table above except the initial state).
The next `solve(x0)` then only does the $\mathbf{x}_0$-dependent vector work and the solver iterations.
This is the split used by real-time iteration schemes:
when the model or weights change every step (an affine approximation along the trajectory, or scheduled weights), set them and call `prepare()` right after applying the previous input, while waiting for the next measurement,
so the matrix rebuild and refactorization no longer add to the delay between measurement and input.

=== "Python"

    ```python
    while running:
        mpc.setModelDiscrete(Ad, Bd, wd)  # e.g. about the predicted next state
        mpc.prepare()  # matrix work and refactorization, before the measurement
        xk = measure()
        status = mpc.solve(xk)  # only x0-dependent work and solver iterations
        apply(mpc.getNextInput())
    ```

=== "C++"

    ```cpp
    while (running) {
      mpc.setModelDiscrete(Ad, Bd, wd); // e.g. about the predicted next state
      (void)mpc.prepare(); // matrix work and refactorization, before measuring
      measure(xk);
      const affine_mpc::SolveStatus status{mpc.solve(xk)};
      mpc.getNextInput(uk);
      apply(uk);
    }
    ```

Calling `prepare()` is never required, since `solve()` applies anything still pending, and both orders give the same solution.
It returns `UpdateFailed` like `solve()` if OSQP rejects an update, and its duration is reported as `SolveInfo.prepare_time`.
For an LTI problem with fixed weights it has nothing to do.

## MPC to QP Relationship

The following tables summarize how MPC parameter updates affect QP parameters.
These tables show which OSQP data may change for each MPC update.
Updates that touch QP matrices are typically heavier than vector-only updates,
but the actual cost depends on the formulation and problem structure.

**Sparse:**

| MPC Term              | $P$ | $A_\mathrm{qp}$ | $q$ | $\boldsymbol{\ell}$ | $\boldsymbol{\upsilon}$ |
| --------------------- | --- | --------------- | --- | ------------------- | ----------------------- |
| $\mathbf{x}_0$        |     |                 |     | ✓                   | ✓                       |
| $A$                   |     | ✓               |     | ✓                   | ✓                       |
| $B$                   |     | ✓               |     |                     |                         |
| $\mathbf{w}$          |     |                 |     | ✓                   | ✓                       |
| $Q$, $Q_{\mathrm{f}}$ | ✓   |                 | ✓   |                     |                         |
| $R$                   | ✓   |                 | ✓   |                     |                         |

**Condensed:**

| MPC Term              | $P$ | $A_\mathrm{qp}$ | $q$ | $\boldsymbol{\ell}$ | $\boldsymbol{\upsilon}$ |
| --------------------- | --- | --------------- | --- | ------------------- | ----------------------- |
| $\mathbf{x}_0$        |     |                 | ✓   | xsat                | xsat                    |
| $A$                   | ✓   | xsat            | ✓   | xsat                | xsat                    |
| $B$                   | ✓   | xsat            | ✓   | xsat                | xsat                    |
| $\mathbf{w}$          |     |                 | ✓   | xsat                | xsat                    |
| $Q$, $Q_{\mathrm{f}}$ | ✓   |                 | ✓   |                     |                         |
| $R$                   | ✓   |                 | ✓   |                     |                         |

!!! note

    The model and initial state only affect
    $A_\mathrm{qp}$, $\boldsymbol{\ell}$, and $\boldsymbol{\upsilon}$
    if state saturation constraints are enabled.

**Shared:**

| MPC Term                       | $P$ | $A_\mathrm{qp}$ | $q$ | $\boldsymbol{\ell}$ | $\boldsymbol{\upsilon}$ |
| ------------------------------ | --- | --------------- | --- | ------------------- | ----------------------- |
| $\mathbf{x}_{\mathrm{ref}}$    |     |                 | ✓   |                     |                         |
| $\mathbf{c}_{\mathrm{ref}}$    |     |                 | ✓   |                     |                         |
| $\mathbf{x}_\mathrm{min}$      |     |                 |     | ✓                   | ✓                       |
| $\mathbf{x}_\mathrm{max}$      |     |                 |     | ✓                   | ✓                       |
| $\mathbf{u}_\mathrm{min}$      |     |                 |     | ✓                   | ✓                       |
| $\mathbf{u}_\mathrm{max}$      |     |                 |     | ✓                   | ✓                       |
| $\mathbf{u}_{0,\mathrm{slew}}$ |     |                 |     | ✓                   | ✓                       |
| $\mathbf{c}_\mathrm{slew}$     |     |                 |     | ✓                   | ✓                       |
| $\mathbf{u}_\mathrm{prev}$     |     |                 |     | ✓                   | ✓                       |

## Related Topics

- [QP Formulation](qp-formulation.md): how MPC data maps into the OSQP problem
- [Usage](../usage.md): configuring an MPC object and running repeated solves
- [Examples](../examples.md): complete repeated-solve workflows in context
