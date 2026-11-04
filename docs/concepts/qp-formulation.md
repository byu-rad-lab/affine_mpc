# QP Formulation

This page shows how the finite-horizon MPC problem in `affine_mpc` is converted into quadratic-program form.
The derivation is written at the level of the mathematical structure used by the library,
with the standard QP form that OSQP and DAQP accept.

--8<-- "concepts/nomenclature.md"

From [Input Parameterization](input-parameterization.md),
the control points can be mapped to the input trajectory by

$$
\bm{u} = \Psi^p(\bm{\tau}) \, \bm{c}.
$$

A [custom basis](input-parameterization.md#custom-bases) takes the place of $\Psi^p(\bm{\tau})$ in everything below, with the coefficients in place of the control points; it is dense, so the band of every sample holds every coefficient.

## Quadratic Program (QP)

The form of a QP is

$$
\begin{align}
    \min_{\bm{z}} &\quad \frac{1}{2} \bm{z}^\T P \bm{z} + \bm{q}^\T \bm{z} \\
    s.t. &\quad \bm{\ell} \leq A_{\mathrm{qp}} \, \bm{z} \leq \bm{\upsilon}
\end{align}
$$

where $\bm{z}$ is the design variable,
$P$ is the cost matrix,
$\bm{q}$ is the cost vector,
$A_\mathrm{qp}$ is the constraint matrix,
and $\bm{\ell}$ and $\bm{\upsilon}$ are the lower and upper bounds on the constraints,
respectively.
Some of the symbols collide with MPC symbols,
so we made slight modifications in notation from OSQP's documentation.

The design variable and the constraint rows are stacked from blocks, and only the blocks the options enable are present.
The design variable holds the control points $\bm{c}$, then the predicted states $\bm{x}$ (sparse formulation only), then the slacks $\bm{s}$ (soft state bounds only).
The constraint rows are stacked in this order:

$$
\underbrace{\begin{bmatrix}
    \bm{\ell}_{\mathrm{model}} \\
    \bm{\ell}_{\mathrm{usat}} \\
    \bm{\ell}_{\mathrm{slew0}} \\
    \bm{\ell}_{\mathrm{slew}} \\
    \bm{\ell}_{\mathrm{xsat}} \\
    \bm{\ell}_{\mathrm{gen}} \\
    \bm{\ell}_{\mathrm{term}}
\end{bmatrix}}_{\bm{\ell}}
    \leq
\underbrace{\begin{bmatrix}
    A_{\mathrm{model}} \\
    A_{\mathrm{usat}} \\
    A_{\mathrm{slew0}} \\
    A_{\mathrm{slew}} \\
    A_{\mathrm{xsat}} \\
    A_{\mathrm{gen}} \\
    A_{\mathrm{term}}
\end{bmatrix}}_{A_\mathrm{qp}} \bm{z}
    \leq
\underbrace{\begin{bmatrix}
    \bm{\upsilon}_{\mathrm{model}} \\
    \bm{\upsilon}_{\mathrm{usat}} \\
    \bm{\upsilon}_{\mathrm{slew0}} \\
    \bm{\upsilon}_{\mathrm{slew}} \\
    \bm{\upsilon}_{\mathrm{xsat}} \\
    \bm{\upsilon}_{\mathrm{gen}} \\
    \bm{\upsilon}_{\mathrm{term}}
\end{bmatrix}}_{\bm{\upsilon}}
$$

| Subscript | Constraint | Block in `getQPConstraintBlocks()` | Present when |
| --- | --- | --- | --- |
| model | model | `dynamics` | sparse formulation |
| usat | input saturation | `input_bounds` | always |
| slew0 | initial input slew rate | `initial_slew` | `slew_initial_input` |
| slew | control point slew rate | `control_point_slew` | `slew_control_points` |
| xsat | state saturation | `state_bounds` | `state_bounds` is `Hard` or `Soft` |
| gen | general constraints | `general_constraints` | `num_general_constraints` $> 0$ |
| term | terminal constraints | `terminal_constraints` | `num_terminal_constraints` $> 0$ |

The MPC problem has summation terms in the cost function and a list of various constraints,
which both need to be converted into a matrix multiply.

### Sparse vs Condensed Design Variable

The main difference between two formulations is the choice of design vector $\bm{z}$ used in the optimization.
Sparse uses the control points and state trajectory while condensed only uses the control points:

$$
\bm{z}_{\mathrm{c}}
    = \bm{c} \quad \quad \quad \bm{z}_{\mathrm{s}}
    = \begin{bmatrix} \bm{c} \\ \bm{x} \end{bmatrix}
$$

With soft state bounds, both append the slacks $\bm{s}$ (see [Selected and Soft State Bounds](#selected-and-soft-state-bounds)).

!!! note

    The current (initial) state $\bm{x}_0$ is often included in the design vector,
    constrained to equal itself.
    Since it is fixed and not optimized, we remove it from the design vector entirely.
    This causes some slight adjustments in the QP formulation.

The rest of this page describes how to convert the MPC optimization problem into a QP problem.

### Cost Function in Matrix Form

The MPC cost is

$$
J = \left\lVert \bm{x}_{\mathrm{ref},T} - \bm{x}_T \right\rVert^2_{Q_{\mathrm{f}}}
    + \sum_{k=1}^{T-1} \left\lVert \bm{x}_{\mathrm{ref},k} - \bm{x}_k \right\rVert^2_Q
    + \sum_{i=0}^{\eta-1} \left\lVert \bm{c}_{\mathrm{ref},i} - \bm{c}_i \right\rVert^2_R
$$

This is the cost with the input cost on the control points (`InputCostMode::ControlPoints`); with `Off`, the input cost terms drop out of everything below, and the other cost terms are added in [Optional Terms](#optional-terms).

Since we're optimizing a norm, it doesn't matter which term is being subtracted.
We will switch to use value minus reference in the QP conversion just to make the value positive,
but it can be done either way.

First, we will create block diagonal matrices using the weights:

$$
\bar{Q} = \mathrm{blkdiag}(Q, \dots, Q, Q_{\mathrm{f}})
    \quad \quad \quad \bar{R} = I_\eta \otimes R
$$

Now the cost function can be written as

$$
J = (\bm{x} - \bm{x}_{\mathrm{ref}})^\T \bar{Q} (\bm{x} - \bm{x}_{\mathrm{ref}})
    + (\bm{c} - \bm{c}_{\mathrm{ref}})^\T \bar{R} (\bm{c} - \bm{c}_{\mathrm{ref}})
$$

We will use this form of the cost function as the starting point for both the sparse and condensed formulations,
but the way we substitute in the model and convert to QP form will differ between the two.

## Sparse Formulation

We'll start with the sparse formulation since it is perhaps a bit more intuitive.
The dynamics remain as explicit linear equality constraints.
The cost is then assembled directly in terms of $\bm{x}$ and $\bm{c}$,
giving a larger but more structured QP.

The sparse may be preferable when:

- the horizon is long
- the state dimension is large
- state saturation is enabled

### Model Constraint

Starting with the model

$$
\bm{x}_{k+1} = A \bm{x}_k + B \bm{u}_k + \bm{w},
$$

we need to rearrange it slightly to fit into a QP form.
The design variable $\bm{z}_{\mathrm{s}}$ has states and inputs
(excluding $\bm{x}_0$ since it is fixed and not optimized),
so we group all of those on one side of the equation:

$$
-\bm{w} = -\bm{x}_{k+1} + A \bm{x}_k + B \bm{u}_k
$$

Now we'll convert to matrix form:

$$
-\begin{bmatrix} \bm{w} \\ \vdots \\ \bm{w} \end{bmatrix}
-\begin{bmatrix} A \bm{x}_0 \\ 0 \\ \vdots \end{bmatrix}
    =
    \begin{bmatrix}
        -I         & 0      & \dots  & \mathbf{0}  \\
        A          & -I     & \ddots & \vdots      \\
                   & \ddots & \ddots & \mathbf{0}  \\
        \mathbf{0} &        & A      & -I
    \end{bmatrix}
    \begin{bmatrix}
        \bm{x}_1 \\ \vdots \\ \bm{x}_T
    \end{bmatrix}
    +
    \begin{bmatrix}
        B          &        & \mathbf{0} \\
                   & \ddots &            \\
        \mathbf{0} &        & B
    \end{bmatrix}
    \begin{bmatrix}
        \bm{u}_0 \\ \vdots \\ \bm{u}_{T-1}
    \end{bmatrix},
$$

$$\Downarrow$$

$$
-\underbrace{
    (\mathbf{1}_T \otimes \bm{w} + \bm{e}_1 \otimes A \bm{x}_0)
}_{\bm{s}_w}
    =
    \underbrace{
        ( I_T^{(-1)} \otimes A - I_{nT} )
    }_{S_x}
    \bm{x}
    +
    \underbrace{
        ( I_T \otimes B )
    }_{S_u}
    \bm{u}.
$$

$$\Downarrow$$

$$
-\bm{s}_w = S_x \bm{x} + S_u \bm{u}
$$

Substituting in the input parameterization gives

$$
-\bm{s}_w = S_x \bm{x} + S_u \Psi^p(\bm{\tau}) \bm{c}.
$$

Finally, we can substitute in the design variable and get the model constraint:

$$
\underbrace{-\bm{s}_w}_{\bm{\ell}_{\mathrm{model}}}
    =
    \underbrace{
        \begin{bmatrix} S_u \Psi^p(\bm{\tau}) & S_x \end{bmatrix}
    }_{A_{\mathrm{model}}}
    \bm{z}_{\mathrm{s}}
    =
    \underbrace{-\bm{s}_w}_{\bm{\upsilon}_{\mathrm{model}}}
$$

### Cost Terms

Starting with the cost function we derived above:

$$
\begin{align}
J &= (\bm{x} - \bm{x}_{\mathrm{ref}})^\T \bar{Q} (\bm{x} - \bm{x}_{\mathrm{ref}})
    + (\bm{c} - \bm{c}_{\mathrm{ref}})^\T \bar{R} (\bm{c} - \bm{c}_{\mathrm{ref}}) \\
  &=(\bm{z}_{\mathrm{s}} - \bm{z}_{\mathrm{s},\mathrm{ref}})^\T
        \underbrace{
            \begin{bmatrix} \bar{R} & \mathbf{0} \\ \mathbf{0} & \bar{Q} \end{bmatrix}
        }_{P}
    (\bm{z}_{\mathrm{s}} - \bm{z}_{\mathrm{s},\mathrm{ref}}) \\
  &= \bm{z}_{\mathrm{s}}^\T P \bm{z}_{\mathrm{s}}
    -2 \bm{z}_{\mathrm{s},\mathrm{ref}}^\T P \bm{z}_{\mathrm{s}}
    + \underbrace{\bm{z}_{\mathrm{s},\mathrm{ref}}^\T P \bm{z}_{\mathrm{s},\mathrm{ref}}}_{\text{drop}}
\end{align}
$$

Here we see the last term has no design variables in it,
so it is constant in terms of the optimization and can be dropped since we only care about the argmin.
Scaling the cost by $\frac{1}{2}$ also leaves the argmin unchanged and matches the QP form:

$$
\frac{1}{2} J
    = \frac{1}{2} \bm{z}_{\mathrm{s}}^\T P \bm{z}_{\mathrm{s}}
    + (\, \underbrace{-P \bm{z}_{\mathrm{s},\mathrm{ref}}}_{\bm{q}} \,)^\T \bm{z}_{\mathrm{s}}
    + \text{const}
$$

Now we have the definition of the QP cost terms:

$$
P = \mathrm{blkdiag}(\bar{R}, \bar{Q})
    \quad \quad \quad \bm{q} = -P \bm{z}_{\mathrm{s},\mathrm{ref}}
$$

### Input Saturation

**Bound the control points** (`InputBoundMode::ControlPoints`, the default):

$$
\begin{bmatrix}
    \bm{u}_{\mathrm{min}} \\ \vdots \\ \bm{u}_{\mathrm{min}}
\end{bmatrix}
\leq
\bm{c}
\leq
\begin{bmatrix}
    \bm{u}_{\mathrm{max}} \\ \vdots \\ \bm{u}_{\mathrm{max}}
\end{bmatrix}
$$

$$
\Downarrow
$$

$$
\underbrace{\mathbf{1}_\eta \otimes \bm{u}_{\mathrm{min}}}_{\bm{\ell}_{\mathrm{usat}}}
\leq
\underbrace{
    \begin{bmatrix} I_{m\eta} & \mathbf{0}_{m\eta \times nT} \end{bmatrix}
}_{A_{\mathrm{usat}}} \bm{z}_{\mathrm{s}}
\leq
\underbrace{\mathbf{1}_\eta \otimes \bm{u}_{\mathrm{max}}}_{\bm{\upsilon}_{\mathrm{usat}}}
$$

**Bound the sampled inputs** (`InputBoundMode::Exact`):

$$
\begin{bmatrix}
    \bm{u}_{\mathrm{min}} \\ \vdots \\ \bm{u}_{\mathrm{min}}
\end{bmatrix}
\leq
\bm{u}
\leq
\begin{bmatrix}
    \bm{u}_{\mathrm{max}} \\ \vdots \\ \bm{u}_{\mathrm{max}}
\end{bmatrix}
$$

$$
\Downarrow
$$

$$
\underbrace{\mathbf{1}_T \otimes \bm{u}_{\mathrm{min}}}_{\bm{\ell}_{\mathrm{usat}}}
\leq
\underbrace{
    \begin{bmatrix} \Psi^p(\bm{\tau}) & \mathbf{0}_{mT \times nT} \end{bmatrix}
}_{A_{\mathrm{usat}}} \bm{z}_{\mathrm{s}}
\leq
\underbrace{\mathbf{1}_T \otimes \bm{u}_{\mathrm{max}}}_{\bm{\upsilon}_{\mathrm{usat}}}
$$

For degree $p = 0$ these are the control point rows above, since the control points are the sampled inputs.
For degree $p = 1$, only the rows of $\Psi^1(\bm{\tau})$ (and bounds) for the samples adjacent to each active knot are kept, which is exactly equivalent since the spline is linear between knots.
A custom basis always uses these constraints, in place of bounding the control points, whatever `input_bounds` is, omitting the samples whose basis row norm is at most $10^{-9}$ times the largest.

**Bound the refined control points** (`InputBoundMode::Refined`, degree $p \ge 2$):

$$
\underbrace{\mathbf{1}_{\eta_{\mathrm{r}}} \otimes \bm{u}_{\mathrm{min}}}_{\bm{\ell}_{\mathrm{usat}}}
\leq
\underbrace{
    \begin{bmatrix} \Omega \otimes I_m & \mathbf{0}_{m\eta_{\mathrm{r}} \times nT} \end{bmatrix}
}_{A_{\mathrm{usat}}} \bm{z}_{\mathrm{s}}
\leq
\underbrace{\mathbf{1}_{\eta_{\mathrm{r}}} \otimes \bm{u}_{\mathrm{max}}}_{\bm{\upsilon}_{\mathrm{usat}}}
$$

The $\eta_{\mathrm{r}} \times \eta$ matrix $\Omega$ maps the control points of each input channel to the control points of the same spline on a refined knot vector (knot insertion), keeping only the $\eta_{\mathrm{r}}$ refined control points that some sample depends on.
It is banded (each row has at most $p+1$ nonzeros) and depends only on the knots, so it is computed once at construction.
For $p \le 1$, `Refined` uses the `ControlPoints` ($p = 0$) or `Exact` ($p = 1$) rows, and for $p \ge 2$ it uses the `Exact` rows when $\eta_{\mathrm{r}} \ge T$.
See [Input Bounds](input-parameterization.md#input-bounds).
See [Input Trajectory Saturation Constraint](problem-formulation.md#input-trajectory-saturation-constraint).

### Slew Initial Input

$$
| \bm{u}_0 - \bm{u}_{-1} | \leq \bm{u}_{0,\mathrm{slew}}
$$

$$
\Downarrow
$$

$$
\underbrace{\bm{u}_{-1} - \bm{u}_{0,\mathrm{slew}}}_{\bm{\ell}_{\mathrm{slew0}}}
\leq
\underbrace{
    \begin{bmatrix}
        (\bm{e}_1^\T \otimes I_m) \Psi^p(\bm{\tau}) & \mathbf{0}_{m \times nT}
    \end{bmatrix}
}_{A_{\mathrm{slew0}}} \bm{z}_{\mathrm{s}}
\leq
\underbrace{\bm{u}_{-1} + \bm{u}_{0,\mathrm{slew}}}_{\bm{\upsilon}_{\mathrm{slew0}}}
$$

### Slew Control Points

$$
| \bm{c}_i - \bm{c}_{i-1} | \leq \bm{c}_{\mathrm{slew}} \quad i=1, \dots, \eta-1
$$

$$
\Downarrow
$$

$$
\underbrace{-\mathbf{1}_{\eta-1} \otimes \bm{c}_{\mathrm{slew}}}_{\bm{\ell}_{\mathrm{slew}}}
\leq
\underbrace{
    \begin{bmatrix}
        D_\eta \otimes I_m & \mathbf{0}_{m(\eta-1) \times nT} \\
    \end{bmatrix}
}_{A_{\mathrm{slew}}} \bm{z}_{\mathrm{s}}
\leq
\underbrace{\mathbf{1}_{\eta-1} \otimes \bm{c}_{\mathrm{slew}}}_{\bm{\upsilon}_{\mathrm{slew}}}
$$

where

$$
D_\eta
    = \begin{bmatrix} \mathbf{0}_{(\eta-1) \times 1} & I_{\eta-1} \end{bmatrix}
    - \begin{bmatrix} I_{\eta-1} & \mathbf{0}_{(\eta-1) \times 1} \end{bmatrix}
    = \begin{bmatrix}
        -1     & 1      &        &    \\
               & \ddots & \ddots &    \\
               &        & -1     & 1
    \end{bmatrix}
    \in \R^{(\eta-1) \times \eta}
$$

is the first-difference matrix, so the row blocks of $(D_\eta \otimes I_m) \bm{c}$ are $\bm{c}_i - \bm{c}_{i-1}$ for $i = 1, \dots, \eta-1$.

### State Saturation

$$
\begin{bmatrix}
    \bm{x}_{\mathrm{min}} \\ \vdots \\ \bm{x}_{\mathrm{min}}
\end{bmatrix}
\leq
\bm{x}
\leq
\begin{bmatrix}
    \bm{x}_{\mathrm{max}} \\ \vdots \\ \bm{x}_{\mathrm{max}}
\end{bmatrix}
$$

$$
\Downarrow
$$

$$
\underbrace{\mathbf{1}_T \otimes \bm{x}_{\mathrm{min}}}_{\bm{\ell}_{\mathrm{xsat}}}
\leq
\underbrace{
    \begin{bmatrix} \mathbf{0}_{nT \times m\eta} & I_{nT} \end{bmatrix}
}_{A_{\mathrm{xsat}}} \bm{z}_{\mathrm{s}}
\leq
\underbrace{\mathbf{1}_T \otimes \bm{x}_{\mathrm{max}}}_{\bm{\upsilon}_{\mathrm{xsat}}}
$$

## Condensed Formulation

This formulation cannot use predicted states in its cost or constraint functions since they are not part of the design variable.
Thus, the condensed formulation does not have the model as an explicit constraint function;
instead, the model is implicitly wrapped into the cost function.
This also means that state saturation cannot be directly performed on states in the condensed case.

The condensed formulation may be preferable when:

- the horizon is short
- the state dimension is small
- state bounds are not enabled

!!! note

    In our limited testing thus far, the condensed formulation seems to be generally faster.

### Model Constraint

To convert the model prediction into a single matrix multiply, we'll start writing out predicted states:

$$
\begin{align}
    \bm{x}_1 &= A \bm{x}_0 + B \bm{u}_0 + \bm{w} \\
    \bm{x}_2 &= A \bm{x}_1 + B \bm{u}_1 + \bm{w} \\
        &= A (A \bm{x}_0 + B \bm{u}_0 + \bm{w}) + B \bm{u}_1 + \bm{w} \\
        &= A^2 \bm{x}_0 + A (B \bm{u}_0 + \bm{w}) + (B \bm{u}_1 + \bm{w}) \\
        &\;\; \vdots \\
    \bm{x}_k &= A^k \bm{x}_0 + \sum_{i=1}^k A^{i-1} ( B \bm{u}_{k-i} + \bm{w} ) \\
\end{align}
$$

Writing this in matrix form yields

$$
\bm{x}
    =
    \begin{bmatrix}
        A \bm{x}_0 \\ A^2 \bm{x}_0 \\ \vdots \\ A^T \bm{x}_0
    \end{bmatrix}
    +
    \begin{bmatrix}
        I       & \mathbf{0} & \cdots & \mathbf{0} \\
        A       & I          & \ddots & \vdots     \\
        \vdots  & \ddots     & \ddots & \mathbf{0} \\
        A^{T-1} & A^{T-2}    & \cdots & I
    \end{bmatrix}
    \left(
    \begin{bmatrix}
        B          &        & \mathbf{0} \\
                   & \ddots &            \\
        \mathbf{0} &        & B
    \end{bmatrix}
    \bm{u}
    +
    \begin{bmatrix}
        \bm{w} \\ \vdots \\ \bm{w}
    \end{bmatrix}
    \right)
$$

$$\Downarrow$$

$$
\bm{x}
    =
    % \mathrm{blkdiag}(A, A^2, \dots, A^T) (\mathbf{1}_T \otimes \bm{x}_0)
    \sum_{i=1}^{T}(\bm{e}_i \otimes A^i \bm{x}_0)
    +
    \left( \sum_{i=0}^{T-1} I_T^{(-i)} \otimes A^i \right)
    \left( (I_T \otimes B) \bm{u} + \mathbf{1}_T \otimes \bm{w} \right)
$$

Now if we let

$$
S = \left( \sum_{i=0}^{T-1} I_T^{(-i)} \otimes A^i \right) (I_T \otimes B) \Psi^p(\bm{\tau})
$$

$$
\bm{v}
    =
    % \mathrm{blkdiag}(A, A^2, \dots, A^T) (\mathbf{1}_T \otimes \bm{x}_0)
    \sum_{i=1}^{T}(\bm{e}_i \otimes A^i \bm{x}_0)
    +
    \left( \sum_{i=0}^{T-1} I_T^{(-i)} \otimes A^i \right) (\mathbf{1}_T \otimes \bm{w})
$$

??? info "Alternate Expanded Visual Form"

    $$
    S =
        \begin{bmatrix}
            B        & \mathbf{0} & \cdots & \mathbf{0} \\
            AB       & B          & \ddots & \vdots     \\
            \vdots   & \ddots     & \ddots & \mathbf{0} \\
            A^{T-1}B & A^{T-2}B   & \cdots & B
        \end{bmatrix}
        \Psi^p(\bm{\tau})
    $$

    $$
    \bm{v} =
        \begin{bmatrix}
            A \bm{x}_0   + \bm{w} \\
            A^2 \bm{x}_0 + A \bm{w} + \bm{w} \\
            \vdots \\
            A^T \bm{x}_0 + \sum_{i=0}^{T-1} A^i \bm{w}
        \end{bmatrix}
    $$

then the final condensed form is

$$
\bm{x} = S \bm{z}_{\mathrm{c}} + \bm{v}.
$$

This gives us a model function we can use to substitute out predicted states from the cost function.
The condensed formulation does not have an explicit model constraint, so
$A_{\mathrm{model}}$,
$\bm{\ell}_{\mathrm{model}}$,
and $\bm{\upsilon}_{\mathrm{model}}$ are all empty.

### Cost Terms

Starting with the cost function we derived above:

$$
\begin{align}
J
    &=
    (\bm{x} - \bm{x}_{\mathrm{ref}})^\T \bar{Q} (\bm{x} - \bm{x}_{\mathrm{ref}})
    + (\bm{c} - \bm{c}_{\mathrm{ref}})^\T \bar{R} (\bm{c} - \bm{c}_{\mathrm{ref}}) \\
    &=
    \bm{x}^\T \bar{Q} \bm{x}
    - 2 \bm{x}^\T \bar{Q} \bm{x}_{\mathrm{ref}}
    + \underbrace{\bm{x}_{\mathrm{ref}}^\T \bar{Q} \bm{x}_{\mathrm{ref}}}_{\text{drop}}
    + \bm{c}^\T \bar{R} \bm{c}
    - 2 \bm{c}^\T \bar{R} \bm{c}_{\mathrm{ref}}
    + \underbrace{\bm{c}_{\mathrm{ref}}^\T \bar{R} \bm{c}_{\mathrm{ref}}}_{\text{drop}} \\
    &=
    \underbrace{\bm{x}^\T \bar{Q} \bm{x}
    - 2 \bm{x}^\T \bar{Q} \bm{x}_{\mathrm{ref}}}_{J_x}
    + \underbrace{\bm{c}^\T \bar{R} \bm{c}
    - 2 \bm{c}_{\mathrm{ref}}^\T \bar{R} \bm{c}}_{J_c}
\end{align}
$$

Substituting in the condensed model for the predicted state trajectory yields

$$
\begin{align}
J_x
    &=
    (S \bm{z}_{\mathrm{c}} + \bm{v})^\T \bar{Q} (S \bm{z}_{\mathrm{c}} + \bm{v})
    - 2 (S \bm{z}_{\mathrm{c}} + \bm{v})^\T \bar{Q} \bm{x}_{\mathrm{ref}} \\
    &=
    \bm{z}_{\mathrm{c}}^\T (S^\T \bar{Q} S) \bm{z}_{\mathrm{c}}
    + 2 \bm{v}^\T \bar{Q} S \bm{z}_{\mathrm{c}}
    + \underbrace{\bm{v}^\T \bar{Q} \bm{v}}_{\text{drop}}
    - 2 \bm{x}_{\mathrm{ref}}^\T \bar{Q} S \bm{z}_{\mathrm{c}}
    - \underbrace{2 \bm{v}^\T \bar{Q} \bm{x}_{\mathrm{ref}}}_{\text{drop}} \\
    &=
    \bm{z}_{\mathrm{c}}^\T (S^\T \bar{Q} S) \bm{z}_{\mathrm{c}}
    + 2 (S^\T \bar{Q} (\bm{v} - \bm{x}_{\mathrm{ref}}))^\T \bm{z}_{\mathrm{c}}
    + \text{const}
\end{align}
$$

and

$$
\begin{align}
J_c
    &=
    \bm{z}_{\mathrm{c}}^\T \bar{R} \bm{z}_{\mathrm{c}}
    - 2 \bm{c}_{\mathrm{ref}}^\T \bar{R} \bm{z}_{\mathrm{c}} \\
    &=
    \bm{z}_{\mathrm{c}}^\T \bar{R} \bm{z}_{\mathrm{c}}
    - 2 (\bar{R} \bm{c}_{\mathrm{ref}})^\T \bm{z}_{\mathrm{c}} \\
\end{align}
$$

Combining $J_x$ and $J_c$ back together and scaling by $\frac{1}{2}$
(which, like dropping constants, leaves the argmin unchanged) gives the final form

$$
\begin{align}
\frac{1}{2} J
    &=
    \frac{1}{2} \bm{z}_{\mathrm{c}}^\T
    \underbrace{(S^\T \bar{Q} S + \bar{R})}_{P}
    \bm{z}_{\mathrm{c}}
    +
    (\underbrace{
        S^\T \bar{Q} (\bm{v} - \bm{x}_{\mathrm{ref}}) - \bar{R} \bm{c}_{\mathrm{ref}}
    }_{\bm{q}})^\T \bm{z}_{\mathrm{c}}
    + \text{const}
\end{align}
$$

where we see that

$$
P = (S^\T \bar{Q} S + \bar{R})
$$

$$
\bm{q} =  S^\T \bar{Q} (\bm{v} - \bm{x}_{\mathrm{ref}}) - \bar{R} \bm{c}_{\mathrm{ref}}
$$

### Input Saturation

**Bound the control points** (`InputBoundMode::ControlPoints`, the default):

$$
\begin{bmatrix}
    \bm{u}_{\mathrm{min}} \\ \vdots \\ \bm{u}_{\mathrm{min}}
\end{bmatrix}
\leq
\bm{c}
\leq
\begin{bmatrix}
    \bm{u}_{\mathrm{max}} \\ \vdots \\ \bm{u}_{\mathrm{max}}
\end{bmatrix}
$$

$$
\Downarrow
$$

$$
\underbrace{\mathbf{1}_\eta \otimes \bm{u}_{\mathrm{min}}}_{\bm{\ell}_{\mathrm{usat}}}
\leq
\underbrace{I_{m\eta}}_{A_{\mathrm{usat}}} \bm{z}_{\mathrm{c}}
\leq
\underbrace{\mathbf{1}_\eta \otimes \bm{u}_{\mathrm{max}}}_{\bm{\upsilon}_{\mathrm{usat}}}
$$

**Bound the sampled inputs** (`InputBoundMode::Exact`):

$$
\begin{bmatrix}
    \bm{u}_{\mathrm{min}} \\ \vdots \\ \bm{u}_{\mathrm{min}}
\end{bmatrix}
\leq
\bm{u}
\leq
\begin{bmatrix}
    \bm{u}_{\mathrm{max}} \\ \vdots \\ \bm{u}_{\mathrm{max}}
\end{bmatrix}
$$

$$
\Downarrow
$$

$$
\underbrace{\mathbf{1}_T \otimes \bm{u}_{\mathrm{min}}}_{\bm{\ell}_{\mathrm{usat}}}
\leq
\underbrace{ \Psi^p(\bm{\tau}) }_{A_{\mathrm{usat}}} \bm{z}_{\mathrm{c}}
\leq
\underbrace{\mathbf{1}_T \otimes \bm{u}_{\mathrm{max}}}_{\bm{\upsilon}_{\mathrm{usat}}}
$$

For degree $p = 0$ these are the control point rows above, since the control points are the sampled inputs.
For degree $p = 1$, only the rows of $\Psi^1(\bm{\tau})$ (and bounds) for the samples adjacent to each active knot are kept, which is exactly equivalent since the spline is linear between knots.
A custom basis always uses these constraints, in place of bounding the control points, whatever `input_bounds` is, omitting the samples whose basis row norm is at most $10^{-9}$ times the largest.

**Bound the refined control points** (`InputBoundMode::Refined`, degree $p \ge 2$):

$$
\underbrace{\mathbf{1}_{\eta_{\mathrm{r}}} \otimes \bm{u}_{\mathrm{min}}}_{\bm{\ell}_{\mathrm{usat}}}
\leq
\underbrace{
    \Omega \otimes I_m
}_{A_{\mathrm{usat}}} \bm{z}_{\mathrm{c}}
\leq
\underbrace{\mathbf{1}_{\eta_{\mathrm{r}}} \otimes \bm{u}_{\mathrm{max}}}_{\bm{\upsilon}_{\mathrm{usat}}}
$$

The $\eta_{\mathrm{r}} \times \eta$ matrix $\Omega$ maps the control points of each input channel to the control points of the same spline on a refined knot vector (knot insertion), keeping only the $\eta_{\mathrm{r}}$ refined control points that some sample depends on.
It is banded (each row has at most $p+1$ nonzeros) and depends only on the knots, so it is computed once at construction.
For $p \le 1$, `Refined` uses the `ControlPoints` ($p = 0$) or `Exact` ($p = 1$) rows, and for $p \ge 2$ it uses the `Exact` rows when $\eta_{\mathrm{r}} \ge T$.
See [Input Bounds](input-parameterization.md#input-bounds).
See [Input Trajectory Saturation Constraint](problem-formulation.md#input-trajectory-saturation-constraint).

### Slew Initial Input

$$
| \bm{u}_0 - \bm{u}_{-1} | \leq \bm{u}_{0,\mathrm{slew}}
$$

$$
\Downarrow
$$

$$
\underbrace{\bm{u}_{-1} - \bm{u}_{0,\mathrm{slew}}}_{\bm{\ell}_{\mathrm{slew0}}}
\leq
\underbrace{
        (\bm{e}_1^\T \otimes I_m) \Psi^p(\bm{\tau})
}_{A_{\mathrm{slew0}}} \bm{z}_{\mathrm{c}}
\leq
\underbrace{\bm{u}_{-1} + \bm{u}_{0,\mathrm{slew}}}_{\bm{\upsilon}_{\mathrm{slew0}}}
$$

### Slew Control Points

$$
| \bm{c}_i - \bm{c}_{i-1} | \leq \bm{c}_{\mathrm{slew}} \quad i=1, \dots, \eta-1
$$

$$
\Downarrow
$$

$$
\underbrace{-\mathbf{1}_{\eta-1} \otimes \bm{c}_{\mathrm{slew}}}_{\bm{\ell}_{\mathrm{slew}}}
\leq
\underbrace{
    (D_\eta \otimes I_m)
}_{A_{\mathrm{slew}}} \bm{z}_{\mathrm{c}}
\leq
\underbrace{\mathbf{1}_{\eta-1} \otimes \bm{c}_{\mathrm{slew}}}_{\bm{\upsilon}_{\mathrm{slew}}}
$$

where $D_\eta$ is the first-difference matrix defined in the [sparse formulation](#slew-control-points).

### State Saturation

$$
\begin{bmatrix}
    \bm{x}_{\mathrm{min}} \\ \vdots \\ \bm{x}_{\mathrm{min}}
\end{bmatrix}
\leq
\bm{x}
\leq
\begin{bmatrix}
    \bm{x}_{\mathrm{max}} \\ \vdots \\ \bm{x}_{\mathrm{max}}
\end{bmatrix}
$$

$$
\Downarrow
$$

$$
\begin{bmatrix}
    \bm{x}_{\mathrm{min}} \\ \vdots \\ \bm{x}_{\mathrm{min}}
\end{bmatrix}
\leq
S \bm{z}_{\mathrm{c}} + \bm{v}
\leq
\begin{bmatrix}
    \bm{x}_{\mathrm{max}} \\ \vdots \\ \bm{x}_{\mathrm{max}}
\end{bmatrix}
$$

$$
\Downarrow
$$

$$
\underbrace{\mathbf{1}_T \otimes \bm{x}_{\mathrm{min}} - \bm{v}}_{\bm{\ell}_{\mathrm{xsat}}}
\leq
\underbrace{ S }_{A_{\mathrm{xsat}}} \bm{z}_{\mathrm{c}}
\leq
\underbrace{\mathbf{1}_T \otimes \bm{x}_{\mathrm{max}} - \bm{v}}_{\bm{\upsilon}_{\mathrm{xsat}}}
$$

!!! note

    As shown here, state saturation in the condensed formulation adds $S$ to $A_{\mathrm{qp}}$ rather than $I$.
    This makes the constraint matrix less sparse.
    Thus, it may be more efficient to use the sparse formulation when saturating states,
    but the best way to find out for your system is to try both!
    Bounding only the states that need limits (below) keeps only their rows of $S$.

## Optional Terms

The derivations above use the input cost on the control points and bound every state.
The following sections show how the other options change the QP of both formulations.

### Selected and Soft State Bounds

With `bounded_states` listing $n_b$ of the $n$ states, let $E_b \in \R^{n_b \times n}$ hold the matching rows of $I_n$.
The state saturation rows become $(I_T \otimes E_b)$ times the state rows above, with bounds $\mathbf{1}_T \otimes \bm{x}_{\mathrm{min}}$ and $\mathbf{1}_T \otimes \bm{x}_{\mathrm{max}}$ of size $n_b$ (and $-(I_T \otimes E_b)\bm{v}$ in the condensed formulation): $n_b T$ rows instead of $nT$.
In the condensed formulation these are the selected rows of $S$, so no product is needed.

Soft bounds add one slack per bounded state, $\bm{s} \in \R^{n_b}$, after the formulation's variables.
A row can not hold one slack on both of its sides, so each limit becomes its own one-sided row:

$$
\begin{aligned}
(I_T \otimes E_b)\, \bm{x} - \mathbf{1}_T \otimes \bm{s} &\leq \mathbf{1}_T \otimes \bm{x}_{\mathrm{max}}, \\
(I_T \otimes E_b)\, \bm{x} + \mathbf{1}_T \otimes \bm{s} &\geq \mathbf{1}_T \otimes \bm{x}_{\mathrm{min}}, \\
\bm{s} &\geq \mathbf{0},
\end{aligned}
$$

which is $2 n_b T + n_b$ rows.
The penalty $J_s = \bm{\rho}_1^\T \bm{s} + \bm{s}^\T \diag(\bm{\rho}_2) \bm{s}$ is scaled by $\frac{1}{2}$ like the rest of the cost, so the slack block of $P$ is $\diag(\bm{\rho}_2)$ and the slack part of $\bm{q}$ is $\frac{1}{2} \bm{\rho}_1$.

### Input Rate Cost

The input rate cost only involves the control points, which come first in both $\bm{z}_{\mathrm{s}}$ and $\bm{z}_{\mathrm{c}}$, so it adds the same terms to both formulations.
With the $T \times T$ difference matrix $D_T = I_T - I_T^{(-1)}$, which unlike $D_\eta$ keeps the first row so that $\bm{u}_0 - \bm{u}_{-1}$ is included, and $D = D_T \otimes I_m$, the input changes are

$$
\begin{bmatrix} \bm{u}_0 - \bm{u}_{-1} \\ \vdots \\ \bm{u}_{T-1} - \bm{u}_{T-2} \end{bmatrix}
= D \Psi^p(\bm{\tau}) \, \bm{c} - \bm{e}_1 \otimes \bm{u}_{-1},
$$

so with $\bar{R}_\Delta = I_T \otimes R_\Delta$,

$$
J_{\Delta u}
    =
    \bm{c}^\T (D \Psi)^\T \bar{R}_\Delta (D \Psi) \, \bm{c}
    - 2 \left( \Psi^\T (\bm{e}_1 \otimes R_\Delta \bm{u}_{-1}) \right)^\T \bm{c}
    + \text{const},
$$

using $D^\T (\bm{e}_1 \otimes R_\Delta \bm{u}_{-1}) = \bm{e}_1 \otimes R_\Delta \bm{u}_{-1}$.
Scaling by $\frac{1}{2}$ as before, the control point block of $P$ gains $(D \Psi)^\T \bar{R}_\Delta (D \Psi)$ and the control point part of $\bm{q}$ gains $-\Psi^\T (\bm{e}_1 \otimes R_\Delta \bm{u}_{-1})$, which only involves the control points that shape $\bm{u}_0$.

Since $\Psi = \Phi \otimes I_m$, $(D \Psi)^\T \bar{R}_\Delta (D \Psi) = G \otimes R_\Delta$ with $G = (D_T \Phi)^\T (D_T \Phi)$.
$G$ depends only on the basis (banded for B-splines, dense for a custom basis), so it is computed once; a weight change only rescales it.
The previous input $\bm{u}_{-1}$ changes after every solve, so $\bm{q}$ is updated at every solve when this cost is enabled.

### Input Cost on the Sampled Inputs

The derivations above use the input cost on the control points (`InputCostMode::ControlPoints`), whose block of $P$ is $\bar{R} = I_\eta \otimes R$.
With `InputCostMode::Trajectory`, the input cost is evaluated on the sampled inputs $\bm{u} = \Psi^p(\bm{\tau}) \, \bm{c}$ instead.
Like the rate cost, it only involves the control points, so it adds the same terms to both formulations.
With $\bar{R}_T = I_T \otimes R$ and the stacked sampled reference $\bm{u}_{\mathrm{ref}}$,

$$
J_u
    = (\Psi \bm{c} - \bm{u}_{\mathrm{ref}})^\T \bar{R}_T (\Psi \bm{c} - \bm{u}_{\mathrm{ref}})
    = \bm{c}^\T \Psi^\T \bar{R}_T \Psi \, \bm{c}
    - 2 \left( \Psi^\T \bar{R}_T \bm{u}_{\mathrm{ref}} \right)^\T \bm{c}
    + \text{const}.
$$

Scaling by $\frac{1}{2}$ as before, the control point block of $P$ gains $\Psi^\T \bar{R}_T \Psi$ in place of $\bar{R}$, and the control point part of $\bm{q}$ gains $-\Psi^\T \bar{R}_T \bm{u}_{\mathrm{ref}}$ in place of $-\bar{R} \bm{c}_{\mathrm{ref}}$.

Since $\Psi = \Phi \otimes I_m$, $\Psi^\T \bar{R}_T \Psi = \Phi^\T \Phi \otimes R$.
$\Phi^\T \Phi$ depends only on the basis, so it is computed once and declared in the sparsity pattern at construction; a weight change only rescales it.
For a B-spline of degree $p$ it is banded with bandwidth $2p + 1$ (diagonal for degree 0, where its entries are the block lengths of move-blocking), and it is dense for a custom basis.
A reference change only updates $\bm{q}$, accumulated sample by sample from the band of $\Phi$, so it needs no matrix update or refactorization.

### General and Terminal Constraints

The general constraints pair $\bm{x}_k$ with $\bm{u}_k$, while the stacked state $\bm{x}$ starts at $\bm{x}_1$, so the $k = 0$ rows only hold the input part, with the measured state moved into the bounds:

$$
\bm{h}_{\min} - H_x \bm{x}_0 \leq H_u \bm{u}_0 \leq \bm{h}_{\max} - H_x \bm{x}_0,
\qquad
\bm{h}_{\min} \leq H_x \bm{x}_k + H_u \bm{u}_k \leq \bm{h}_{\max} \quad k = 1, \dots, T-1.
$$

With $\bm{u} = \Psi^p(\bm{\tau})\, \bm{c}$, the input part of all $n_h T$ rows is $(I_T \otimes H_u)\, \Psi^p(\bm{\tau})\, \bm{c}$: row block $k$ holds $H_u$ times the basis weights of $\bm{u}_k$ on the control points of its band.
The state part of the rows for $k \geq 1$ is $H_x$ applied to $\bm{x}_1, \dots, \bm{x}_{T-1}$, which each formulation writes as it writes state rows:

- Sparse: $H_x$ on the columns of $\bm{x}_k$ in $\bm{z}_{\mathrm{s}}$, with bounds $\bm{h}_{\min}$ and $\bm{h}_{\max}$.
- Condensed: $H_x S_k$ on the control point columns, where $S_k$ holds the rows of $S$ for $\bm{x}_k$, with bounds $\bm{h}_{\min} - H_x \bm{v}_k$ and $\bm{h}_{\max} - H_x \bm{v}_k$.
  Row $i$ of $H_x S_k$ is a sum over the nonzero entries of row $i$ of $H_x$, so a sparse $H_x$ costs little, and only the columns of the control points up to the band of step $k-1$ are nonzero.

The terminal rows are the same state rows for $\bm{x}_T$ alone: $H_{\mathrm{f}}$ on the columns of $\bm{x}_T$ (sparse), or $H_{\mathrm{f}} S_T$ with bounds shifted by $-H_{\mathrm{f}} \bm{v}_T$ (condensed).
State bounds on selected states are the special case $H_u = 0$ with $H_x = E_b$.

The general rows add $n_h T$ rows and the terminal rows $n_{h,\mathrm{f}}$, after every other block of $A_{\mathrm{qp}}$.
The general bounds depend on $\bm{x}_0$ (through the $k = 0$ rows in both formulations, and through $\bm{v}$ in the condensed one), and so do the condensed terminal bounds (through $\bm{v}$), so they are updated at every solve.

## Accessing the QP

`getQP()` returns the QP as last written by `solve()`, `prepare()`, or `assembleQP()`, and `assembleQP(x0, qp)` assembles it for the current settings and an initial state without solving, before or after `initializeSolver()`.
Both give a `QuadraticProgram` with $P$ and $A_{\mathrm{qp}}$ (`P` and `A`) as compressed sparse column matrices in the same form for both formulations (`scipy.sparse.csc_matrix` in Python, which needs `pip install "affine-mpc[sparse]"`), $P$ full symmetric, and the vectors $\bm{q}$, $\bm{\ell}$, $\bm{\upsilon}$ (`q`, `l`, `u`).
The cost omits the constant terms of the MPC cost, equality rows have $\bm{\ell} = \bm{\upsilon}$, and unbounded sides are $\pm\infty$.
Neither function is meant for the control loop: both copy the QP, and `assembleQP()` leaves its changes to be sent again by the next `prepare()` or `solve()`.

`getQPVariableBlocks()` and `getQPConstraintBlocks()` name the blocks of $\bm{z}$ and of the rows of $A_{\mathrm{qp}}$, in order, each with its offset and size; only the blocks the options enable are listed.

| Block | Kind | Built from |
| --- | --- | --- |
| `control_points` | variables | the input coefficients $\bm{c}$ |
| `states` | variables | sparse formulation only: $\bm{x}_1, \dots, \bm{x}_T$ |
| `state_slacks` | variables | soft state bounds: one slack per bounded state |
| `dynamics` | rows | sparse formulation only: the model as equality rows |
| `input_bounds` | rows | input bounds on the control points, the sampled inputs, or the refined control points |
| `initial_slew` | rows | `slew_initial_input` |
| `control_point_slew` | rows | `slew_control_points` |
| `state_bounds` | rows | `state_bounds` `Hard` or `Soft` (soft: upper rows, lower rows, then $\bm{s} \ge 0$) |
| `general_constraints` | rows | `num_general_constraints` rows per step, steps in order |
| `terminal_constraints` | rows | `num_terminal_constraints` rows on $\bm{x}_T$ |

For example, to hand the QP to another solver or to count its rows and nonzeros:

=== "Python"

    ```python
    qp = affine_mpc.QuadraticProgram()
    mpc.assembleQP(x0, qp)  # or qp = mpc.getQP() after a solve
    print(qp.P.nnz, qp.A.nnz)
    for block in mpc.getQPConstraintBlocks():
        print(block.name, block.offset, block.size)
    ```

=== "C++"

    ```cpp
    affine_mpc::QuadraticProgram qp;
    mpc.assembleQP(x0, qp); // or auto qp = mpc.getQP(); after a solve
    std::cout << qp.P.nonZeros() << " " << qp.A.nonZeros() << "\n";
    for (const affine_mpc::QPBlock& block : mpc.getQPConstraintBlocks())
      std::cout << block.name << " " << block.offset << " " << block.size << "\n";
    ```

`getDualSolution()` gives the dual solution $\bm{y}$ of the last solve, one value per row of $A_{\mathrm{qp}}$ in the order of `getQPConstraintBlocks()`.
It satisfies $P\bm{z} + \bm{q} + A_{\mathrm{qp}}^\T\bm{y} = 0$ at a solution, so $y_i > 0$ when row $i$ is at its upper bound, $y_i < 0$ at its lower bound, and $y_i = 0$ when it is inactive; for example, the nonzero entries of the `input_bounds` block show where the input saturates.
These are the multipliers of the QP's cost, which is half the MPC cost.
With OSQP they are approximate, like its primal solution, and `SolveInfo` reports how closely the solution satisfies the optimality conditions:
`primal_residual` is the largest violation of $\bm{\ell} \le A_{\mathrm{qp}}\bm{z} \le \bm{\upsilon}$, and `dual_residual` the largest entry of $|P\bm{z} + \bm{q} + A_{\mathrm{qp}}^\T\bm{y}|$.
They are the solver's own values, so they cost nothing to read in the control loop, while computing them from `getQP()` copies the QP.
They are absolute, so compare them with a tolerance that suits your problem; OSQP accepts a solution once each is below `eps_abs` plus `eps_rel` times a problem-dependent scale.
DAQP does not report residuals (they are NaN), since its active-set solutions meet its tolerances.

## Implementation Mapping

At a high level,
the derivation maps to the implementation as follows:

- `Parameterization` defines the evaluation map $\Psi^p(\bm{\tau})$
- `Formulation::Condensed` (`Formulation.Condensed` in Python) uses the condensed relation between $\bm{c}$ and $\bm{x}$
- `Formulation::Sparse` keeps $\bm{x}$ directly in the QP decision vector
- `MPC` manages the model, references, weights, constraints, and solver lifecycle the same way for both

The final mathematical ingredient needed for robust usage is understanding what may be updated after initialization and why OSQP imposes fixed sparsity constraints.
That is covered in [Runtime Updates](runtime-updates.md).
