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
The runtime-updated inputs that can affect QP sparsity are the model matrices, the weights, and the general and terminal constraint matrices.
The model offset $\mathbf{w}$ only enters the QP vectors, so it never affects sparsity.

### Model Entries: Initial Nonzeros or Full

`initializeSolver()` takes a `ModelSparsity` that chooses which entries of $A$ and $B$ the solver tracks:

- `ModelSparsity::InitialNonzeros` (the default): the entries that are nonzero in the model set when `initializeSolver()` is called.
  An entry that may become nonzero later must already be nonzero then.
- `ModelSparsity::Full`: every entry of $A$ and $B$, so any entry may change between zero and nonzero later.
  Supported by the sparse formulation only for now (the condensed formulation throws `std::invalid_argument`, `ValueError` in Python).

=== "Python"

    ```python
    mpc.initializeSolver()  # ModelSparsity.InitialNonzeros
    mpc.initializeSolver(model_sparsity=affine_mpc.ModelSparsity.Full)
    ```

=== "C++"

    ```cpp
    (void)mpc.initializeSolver(); // ModelSparsity::InitialNonzeros
    (void)mpc.initializeSolver(affine_mpc::ModelSparsity::Full);
    ```

This is a trade-off between speed and modeling fidelity.
OSQP's factorization treats every tracked entry like a nonzero, even when its value is zero,
so tracking fewer entries makes each solve cheaper when the model has structural zeros
(in the sparse formulation, the untracked entries are left out of its constraint matrix entirely).
In exchange, every later model must be zero outside the tracked entries:
either zero those entries yourself, or do not compute them.

It is common to compute Jacobians of a nonlinear model for $A$, $B$, and $\mathbf{w}$.
At an equilibrium many of their entries are exactly zero,
but they become nonzero at other operating points (e.g. a $g \sin\theta$ term that is zero at hover).
To track them, set a representative model before `initializeSolver()`,
then set the actual model before the first call to `solve()`:

- a model at a generic operating point: evaluating the Jacobians at a random state and input usually gives every entry that can become nonzero
  (a point that is merely nonzero can still give exact zeros, such as $\sin(\pi) = 0$), or
- a model with ones at the union of the nonzeros over the operating points the model visits, which also covers switching between several models, such as piecewise or gain-scheduled models.

Take the union over the discrete $A$ and $B$, not over $A_c$ and $B_c$:
the discretization $e^{A_c \Delta t}$ fills in the pattern of $A_c$
(a chain of integrators has a sparse $A_c$ but an upper triangular $A$).

=== "Python"

    ```python
    Ad_union, Bd_union = np.zeros((n, n)), np.zeros((n, m))
    for x, u in operating_points:
        Ad, Bd, _ = discretizer.discretize(*affine_model(x, u), dt)
        Ad_union[Ad != 0] = 1.0
        Bd_union[Bd != 0] = 1.0
    mpc.setModelDiscrete(Ad_union, Bd_union, np.zeros(n))  # representative
    mpc.initializeSolver()
    mpc.setModelContinuous2Discrete(*affine_model(x0, u0), dt)  # actual
    ```

=== "C++"

    ```cpp
    Eigen::MatrixXd Ad_union{Eigen::MatrixXd::Zero(n, n)};
    Eigen::MatrixXd Bd_union{Eigen::MatrixXd::Zero(n, m)};
    for (const auto& [x, u] : operating_points) {
      affineModel(x, u, Ac, Bc, wc);
      discretizer.discretize(Ac, Bc, wc, dt, Ad, Bd, wd);
      Ad_union = (Ad.array() != 0.0).select(1.0, Ad_union);
      Bd_union = (Bd.array() != 0.0).select(1.0, Bd_union);
    }
    mpc.setModelDiscrete(Ad_union, Bd_union, Eigen::VectorXd::Zero(n));
    (void)mpc.initializeSolver();
    affineModel(x0, u0, Ac, Bc, wc);
    mpc.setModelContinuous2Discrete(Ac, Bc, wc, dt); // actual
    ```

`ModelSparsity::Full` avoids the representative model at the cost of slower solves when the model has structural zeros.

The rest of the pattern does not depend on this choice.
The sparse formulation tracks every weight position on the diagonal of $P$
and every entry of $H_x$, $H_u$, and $H_{\mathrm{f}}$, whatever their values when `initializeSolver()` is called,
so its weights and constraint matrices may change between zero and nonzero freely.
In the condensed formulation, the model and the weights enter $P$ and $A_\mathrm{qp}$ through matrix products,
and its whole pattern comes from the values at initialization,
so ensure that any model, weight, or constraint matrix updates you expect to make later will not introduce new nonzero entries into $P$ or $A_\mathrm{qp}$.

Updates that break these rules are not detected during normal operation,
because checking would add work to every update.
Values outside the initialized pattern are silently dropped,
so the solver solves a different problem than the one you set.
If you suspect this, call `isWithinSparsityPattern()` after `solve()` or `prepare()`;
it returns `false` when the current QP matrices have nonzeros outside the initialized pattern,
or, for the sparse formulation, when the model has a nonzero outside its tracked entries.
It scans the QP matrices, so use it for debugging rather than in a control loop.
Debug builds of the library also assert in the sparse formulation's model setters that the model stays within its tracked entries.

!!! note

    OSQP exploits sparsity for efficiency,
    so coefficients that are truly zero throughout the expected operating range should remain exactly zero at initialization.
    Include entries that may become active later,
    but avoid making matrices denser than necessary just to be conservative.
    The sparse formulation stores every entry of its constraint matrix blocks ($H_x$, $H_u$, $H_{\mathrm{f}}$, and, with `ModelSparsity::Full`, $A$ and $B$),
    including exact zeros, which OSQP's factorization treats like nonzeros.

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
| $H_x$, $H_u$, $\mathbf{h}_\mathrm{min}$, $\mathbf{h}_\mathrm{max}$ | `setGeneralConstraints()`                       |
| $H_\mathrm{f}$, $\mathbf{h}_{\mathrm{f},\mathrm{min}}$, $\mathbf{h}_{\mathrm{f},\mathrm{max}}$ | `setTerminalConstraints()`  |
| $\mathbf{u}_\mathrm{prev}$                           | `setPreviousInput()` (also set by each successful solve)  |

## When Updates Reach the QP

Setters only validate and store their values and mark a change category.
All QP work happens in `solve()`, once per changed category, no matter how many setters were called since the last solve.
Updates that OSQP rejects (often from non-finite values) are reported by `solve()` as `UpdateFailed` and stay pending, so the next solve retries them.

The categories differ in cost:

| Category | Setters | Condensed QP work | Sparse QP work | OSQP refactors |
| --- | --- | --- | --- | --- |
| Initial state $\mathbf{x}_0$ | `solve()` | $\mathbf{v}$, $q$, state-limit and constraint bounds | first dynamics-row bounds, general constraint bounds | No |
| Model offset $\mathbf{w}$ | `setModelOffset*()` | $\mathbf{v}$, $q$, state-limit bounds | dynamics-row bounds | No |
| Model $A$, $B$ | `setModelDiscrete()`, `setModelContinuous2Discrete()` | $S$, $P$, state rows of $A_\mathrm{qp}$ | dynamics blocks of $A_\mathrm{qp}$ | Yes |
| Weights | `setWeights()`, `setStateWeights()`, `setInputWeights()` | $P$, $q$ | diagonal of $P$, $q$ | Yes |
| References | `setReference*()` | $q$ | $q$ | No |
| Limits and slew rates | `setInputLimits()`, `setStateLimits()`, `setSlewRate()`, `setSlewRateInitial()`, constraint bounds | $\boldsymbol{\ell}$, $\boldsymbol{\upsilon}$ | $\boldsymbol{\ell}$, $\boldsymbol{\upsilon}$ | No |
| Constraint matrices $H_x$, $H_u$, $H_\mathrm{f}$ | `setGeneralConstraints()`, `setTerminalConstraints()` | constraint rows of $A_\mathrm{qp}$ (with $S$) and their bounds | constraint rows of $A_\mathrm{qp}$ | Yes |
| Previous input | `setPreviousInput()` | initial-slew bounds | initial-slew bounds | No |

The model setters also set $\mathbf{w}$, so they mark both model categories.
The constraint setters mark the constraint matrices only when a value of $H_x$, $H_u$, or $H_\mathrm{f}$ differs from the stored one, so passing the same matrices with new bounds only updates QP vectors.
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
It returns `UpdateFailed` like `solve()` if OSQP rejects an update.
The next `solve()` reports the total time of the `prepare()` calls since the previous solve as `SolveInfo.prepare_time`, so `mpc_solve_time` stays the latency after the measurement.
For an LTI problem with fixed weights it has nothing to do.
[Model Update Strategies](model-updates.md) applies this split to a nonlinear model and measures what it saves.

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
- [Model Update Strategies](model-updates.md): updating the affine approximation of a nonlinear model with `prepare()` and the offset-only setters
- [Usage](../usage.md): configuring an MPC object and running repeated solves
- [Examples](../examples/index.md): complete repeated-solve workflows in context
