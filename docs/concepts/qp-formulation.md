# QP Formulation

This page shows how the finite-horizon MPC problem in `affine_mpc` is converted into quadratic-program form.
The derivation is written at the level of the mathematical structure used by the library,
with the same high-level QP objects that OSQP expects.

## Stacked Variables

Define the stacked predicted state trajectory

$$
\mathbf{X} =
\begin{bmatrix}
\mathbf{x}_1 \\
\mathbf{x}_2 \\
\vdots \\
\mathbf{x}_T
\end{bmatrix}
\in \mathbb{R}^{nT},
$$

the stacked dense input trajectory

$$
\mathbf{U} =
\begin{bmatrix}
\mathbf{u}_0 \\
\mathbf{u}_1 \\
\vdots \\
\mathbf{u}_{T-1}
\end{bmatrix}
\in \mathbb{R}^{mT},
$$

and the stacked control-point vector

$$
\boldsymbol{\nu} =
\begin{bmatrix}
\boldsymbol{\nu}_0 \\
\boldsymbol{\nu}_1 \\
\vdots \\
\boldsymbol{\nu}_{\eta-1}
\end{bmatrix}
\in \mathbb{R}^{m\eta}.
$$

From [Input Parameterization](input-parameterization.md),
the dense input trajectory satisfies

$$
\mathbf{U} = M \boldsymbol{\nu}.
$$

## Stacked Prediction Model

Repeated substitution of the affine model

$$
\mathbf{x}_{k+1} = A \mathbf{x}_k + B \mathbf{u}_k + \mathbf{w}
$$

produces the standard stacked prediction relation

$$
\mathbf{X} = S_x \mathbf{x}_0 + S_u \mathbf{U} + \mathbf{s}_w,
$$

where:

- $S_x \in \mathbb{R}^{nT \times n}$ maps the initial state to the full trajectory
- $S_u \in \mathbb{R}^{nT \times mT}$ maps the dense input trajectory to the state trajectory
- $\mathbf{s}_w \in \mathbb{R}^{nT}$ collects the repeated affine offset terms

Substituting the parameterized trajectory gives

$$
\mathbf{X} = S_x \mathbf{x}_0 + S_u M \boldsymbol{\nu} + \mathbf{s}_w.
$$

This is the key relation used by the condensed formulation.

## Quadratic Cost in Stacked Form

Let the stacked state reference be

$$
\bar{\mathbf{X}} =
\begin{bmatrix}
\bar{\mathbf{x}}_1 \\
\bar{\mathbf{x}}_2 \\
\vdots \\
\bar{\mathbf{x}}_T
\end{bmatrix},
$$

and define the block-diagonal state-weight matrix

$$
\bar{Q} = \mathrm{blkdiag}(Q, \dots, Q, Q_f).
$$

If input tracking cost is enabled,
define the stacked control-point reference

$$
\bar{\boldsymbol{\nu}} =
\begin{bmatrix}
\bar{\boldsymbol{\nu}}_0 \\
\bar{\boldsymbol{\nu}}_1 \\
\vdots \\
\bar{\boldsymbol{\nu}}_{\eta-1}
\end{bmatrix},
$$

and the block-diagonal input-weight matrix

$$
\bar{R} = I_{\eta} \otimes R.
$$

The full stacked objective becomes

$$
J = (\mathbf{X} - \bar{\mathbf{X}})^\top \bar{Q} (\mathbf{X} - \bar{\mathbf{X}})
  + (\boldsymbol{\nu} - \bar{\boldsymbol{\nu}})^\top \bar{R} (\boldsymbol{\nu} - \bar{\boldsymbol{\nu}}),
$$

with the understanding that the second term is absent when input cost is disabled.

## Condensed Formulation

In the condensed formulation,
the decision vector is simply

$$
\mathbf{z} = \boldsymbol{\nu}.
$$

Substituting

$$
\mathbf{X} = S_x \mathbf{x}_0 + S_u M \boldsymbol{\nu} + \mathbf{s}_w
$$

into the cost gives a quadratic function of $\boldsymbol{\nu}$.
After collecting quadratic and linear terms,
the result has the standard QP form

$$
\min_{\boldsymbol{\nu}}
\quad \frac{1}{2} \boldsymbol{\nu}^\top P \boldsymbol{\nu} + q^\top \boldsymbol{\nu},
$$

where,
up to the conventional factor of $2$ in the Hessian,

$$
P \sim M^\top S_u^\top \bar{Q} S_u M + \bar{R},
$$

and the linear term depends on

- the initial state $\mathbf{x}_0$
- the affine offset stack $\mathbf{s}_w$
- the state reference $\bar{\mathbf{X}}$
- the input reference $\bar{\boldsymbol{\nu}}$ when input cost is enabled

State variables do not appear in the optimization vector,
but they are still recoverable after a solve through the condensed prediction equation.

## Sparse Formulation

In the sparse formulation,
the decision vector keeps both control points and predicted states:

$$
\mathbf{z} =
\begin{bmatrix}
\boldsymbol{\nu} \\
\mathbf{X}
\end{bmatrix}.
$$

The dynamics remain as explicit linear equality constraints.
The cost is then assembled directly in terms of $\mathbf{X}$ and $\boldsymbol{\nu}$,
giving a larger but more structured QP.

The sparse form is often preferable when:

- the horizon is long
- the state dimension is large
- explicit state constraints dominate the problem structure

## Constraint Stacking

All constraints are written in OSQP form

$$
l \le A \mathbf{z} \le u.
$$

Depending on the chosen options,
the constraint matrix may include rows for:

### Dynamics

In the sparse formulation,
the model equations appear explicitly as equality constraints.

### Input bounds

Depending on the option choice,
the bounds may apply either to:

- the control points $\boldsymbol{\nu}$, or
- the dense input trajectory $\mathbf{U} = M\boldsymbol{\nu}$

### State bounds

If state saturation is enabled,
stacked bounds on $\mathbf{X}$ become additional linear constraints.

### Slew-rate constraints

Initial slew-rate and control-point slew-rate limits both become linear inequalities on either:

- the first dense input relative to the previous applied input, or
- consecutive control points

## Final QP Form

Both formulations ultimately produce a QP in the form expected by OSQP:

$$
\begin{aligned}
\min_{\mathbf{z}} &\quad \frac{1}{2} \mathbf{z}^\top P \mathbf{z} + q^\top \mathbf{z} \\
\text{s.t.} &\quad l \le A \mathbf{z} \le u.
\end{aligned}
$$

The main structural difference is the meaning of $\mathbf{z}$:

- condensed: $\mathbf{z} = \boldsymbol{\nu}$
- sparse: $\mathbf{z} = [\boldsymbol{\nu}^\top\; \mathbf{X}^\top]^\top$

## Which MPC Ingredients Affect the QP Matrices?

The QP data are assembled from:

- model terms: $A$, $B$, $\mathbf{w}$
- weights: $Q$, $Q_f$, $R$
- references
- bounds and slew-rate limits
- parameterization choices through $M$

This matters because changes to model terms and weights can affect the initialized sparsity pattern of the QP matrices.
That runtime-update constraint is described in [Runtime Updates](runtime-updates.md).

## Implementation Mapping

At a high level,
the derivation maps to the implementation as follows:

- `Parameterization` defines the evaluation map $M$
- `CondensedMPC` uses the condensed relation between $\boldsymbol{\nu}$ and $\mathbf{X}$
- `SparseMPC` keeps $\mathbf{X}$ directly in the QP decision vector
- `MPCBase` manages the shared model,
  references,
  weights,
  constraints,
  and solver lifecycle

The final mathematical ingredient needed for robust usage is understanding what may be updated after initialization and why OSQP imposes fixed sparsity constraints.
That is covered in [Runtime Updates](runtime-updates.md).
