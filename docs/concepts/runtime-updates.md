# Runtime Updates

This page explains what may change between solves,
why `initializeSolver()` is a structural boundary,
and how OSQP's fixed sparsity pattern affects model and weight updates.

## Configure Then Initialize

The most important lifecycle rule in `affine_mpc` is:

1. construct the MPC object
1. set model, limits, weights, and references
1. call `initializeSolver()`
1. solve repeatedly while updating values as needed

The reason is not merely stylistic.
Calling `initializeSolver()` fixes the QP structure that OSQP will track internally.

## What OSQP Fixes at Initialization

OSQP solves quadratic programs of the form

$$
\begin{aligned}
\min_z &\quad \frac{1}{2} z^\top P z + q^\top z \\
\text{s.t.} &\quad l \le A z \le u.
\end{aligned}
$$

After initialization,
OSQP allows the values in these matrices and vectors to be updated,
but it does not allow the sparsity pattern of $P$ or $A$ to change.

In practice this means:

- existing nonzero entries may change value
- entries that were structurally zero at initialization cannot later become active solver entries

## Which User Inputs Affect QP Structure?

The QP matrices are affected by:

- model terms: $A$, $B$, $w$
- weights: $Q$, $Q_f$, $R$
- enabled options and constraint structure
- parameterization structure through the chosen `Parameterization`

In particular,
model updates can affect QP matrices $P$ and $A$,
and weight updates can affect $P$.

Therefore,
if model coefficients or weight entries may become nonzero later,
they should already be nonzero when the solver is initialized.

## Practical Examples

### Safe update

If an entry of $Q$ is already nonzero,
changing its numeric value between solves is safe because the QP sparsity structure is unchanged.

### Unsafe update

If an entry of $Q$ is exactly zero when the solver is initialized,
and later you want it to become nonzero,
that may require a new nonzero entry in $P$.
OSQP will not track that new index through cheap runtime updates.

The same principle applies to model coefficients that influence the assembled QP matrices.

## Structural vs Value Changes

The safest way to think about updates is:

- values may change after initialization
- structural choices should not

Structural choices include:

- which optional constraints and costs are enabled
- which matrix entries may ever be active
- how the input trajectory is parameterized

This is why `Options` are construction-time feature toggles rather than settings that are freely changed later.

## What Can Typically Be Updated Between Solves?

Provided the initialized sparsity pattern is preserved,
the following can typically be updated efficiently between solves:

- model values
- weights
- state references
- input references
- input limits
- state limits
- slew-rate limits
- current state

These updates are one of the main reasons the library is useful for repeated-solve MPC workflows.

## Relation to the Public API

The practical API consequence is simple:

- set the full model,
  all enabled limits,
  weights,
  and references before `initializeSolver()`
- use the setter methods after initialization only for updates that preserve the already-initialized structure

If you expect a coefficient or weight to become active later,
initialize with that structure already present.

## Where to Read Next

- [Problem Formulation](problem-formulation.md): full MPC problem statement
- [Input Parameterization](input-parameterization.md): derivation of the parameterization matrix
- [QP Formulation](qp-formulation.md): how the MPC problem becomes the OSQP problem
