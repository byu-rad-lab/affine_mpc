# QP Formulation

This page shows how the finite-horizon MPC problem in `affine_mpc` is converted into quadratic-program form.
The derivation is written at the level of the mathematical structure used by the library,
with the same high-level QP objects that OSQP expects.

--8<-- "concepts/nomenclature.md"

From [Input Parameterization](input-parameterization.md),
the control points can be mapped to the input trajectory by

$$
\mathbf{u} = \Psi^p(\boldsymbol{\tau}) \, \mathbf{c}.
$$

## Quadratic Program (QP)

The form of a QP is

$$
\begin{align}
    \min_{\mathbf{z}} &\quad \frac{1}{2} \mathbf{z}^\top P \mathbf{z} + \mathbf{q}^\top \mathbf{z} \\
    s.t. &\quad \boldsymbol{\ell} \leq A_{\mathrm{qp}} \, \mathbf{z} \leq \boldsymbol{\upsilon}
\end{align}
$$

where $\mathbf{z}$ is the design variable,
$P$ is the cost matrix,
$\mathbf{q}$ is the cost vector,
$A_\mathrm{qp}$ is the constraint matrix,
and $\boldsymbol{\ell}$ and $\boldsymbol{\upsilon}$ are the lower and upper bounds on the constraints,
respectively.
Some of the symbols collide with MPC symbols,
so we made slight modifications in notation from OSQP's documentation.

`affine_mpc` supports the following constraints:

- model
- input saturation
- initial input slew rate
- control point slew rate
- state saturation

$$
\underbrace{\begin{bmatrix}
    \boldsymbol{\ell}_{\mathrm{model}} \\
    \boldsymbol{\ell}_{\mathrm{usat}} \\
    \boldsymbol{\ell}_{\mathrm{slew0}} \\
    \boldsymbol{\ell}_{\mathrm{slew}} \\
    \boldsymbol{\ell}_{\mathrm{xsat}}
\end{bmatrix}}_{\boldsymbol{\ell}}
    \leq
\underbrace{\begin{bmatrix}
    A_{\mathrm{model}} \\
    A_{\mathrm{usat}} \\
    A_{\mathrm{slew0}} \\
    A_{\mathrm{slew}} \\
    A_{\mathrm{xsat}}
\end{bmatrix}}_{A_\mathrm{qp}} \mathbf{z}
    \leq
\underbrace{\begin{bmatrix}
    \boldsymbol{\upsilon}_{\mathrm{model}} \\
    \boldsymbol{\upsilon}_{\mathrm{usat}} \\
    \boldsymbol{\upsilon}_{\mathrm{slew0}} \\
    \boldsymbol{\upsilon}_{\mathrm{slew}} \\
    \boldsymbol{\upsilon}_{\mathrm{xsat}}
\end{bmatrix}}_{\boldsymbol{\upsilon}}
$$

The MPC problem has summation terms in the cost function and a list of various constraints,
which both need to be converted into a matrix multiply.

### Sparse vs Condensed Design Variable

The main difference between two formulations is the choice of design vector $\mathbf{z}$ used in the optimization.
Sparse uses the control points and state trajectory while condensed only uses the control points:

$$
\mathbf{z}_{\mathrm{c}}
    = \mathbf{c} \quad \quad \quad \mathbf{z}_{\mathrm{s}}
    = \begin{bmatrix} \mathbf{c} \\ \mathbf{x} \end{bmatrix}
$$

!!! note

    The current (initial) state $\mathbf{x}_0$ is often included in the design vector,
    constrained to equal itself.
    Since it is fixed and not optimized, we remove it from the design vector entirely.
    This causes some slight adjustments in the QP formulation.

The rest of this page describes how to convert the MPC optimization problem into a QP problem.

### Cost Function in Matrix Form

The MPC cost is

$$
J = \left\lVert \mathbf{x}_{\mathrm{ref},T} - \mathbf{x}_T \right\rVert^2_{Q_{\mathrm{f}}}
    + \sum_{k=1}^{T-1} \left\lVert \mathbf{x}_{\mathrm{ref},k} - \mathbf{x}_k \right\rVert^2_Q
    + \sum_{i=0}^{\eta-1} \left\lVert \mathbf{c}_{\mathrm{ref},i} - \mathbf{c}_i \right\rVert^2_R
$$

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
J = (\mathbf{x} - \mathbf{x}_{\mathrm{ref}})^\top \bar{Q} (\mathbf{x} - \mathbf{x}_{\mathrm{ref}})
    + (\mathbf{c} - \mathbf{c}_{\mathrm{ref}})^\top \bar{R} (\mathbf{c} - \mathbf{c}_{\mathrm{ref}})
$$

We will use this form of the cost function as the starting point for both the sparse and condensed formulations,
but the way we substitute in the model and convert to QP form will differ between the two.

## Sparse Formulation

We'll start with the sparse formulation since it is perhaps a bit more intuitive.
The dynamics remain as explicit linear equality constraints.
The cost is then assembled directly in terms of $\mathbf{x}$ and $\mathbf{c}$,
giving a larger but more structured QP.

The sparse may be preferable when:

- the horizon is long
- the state dimension is large
- state saturation is enabled

### Model Constraint

Starting with the model

$$
\mathbf{x}_{k+1} = A \mathbf{x}_k + B \mathbf{u}_k + \mathbf{w},
$$

we need to rearrange it slightly to fit into a QP form.
The design variable $\mathbf{z}_{\mathrm{s}}$ has states and inputs
(excluding $\mathbf{x}_0$ since it is fixed and not optimized),
so we group all of those on one side of the equation:

$$
-\mathbf{w} = -\mathbf{x}_{k+1} + A \mathbf{x}_k + B \mathbf{u}_k
$$

Now we'll convert to matrix form:

$$
-\begin{bmatrix} \mathbf{w} \\ \vdots \\ \mathbf{w} \end{bmatrix}
-\begin{bmatrix} A \mathbf{x}_0 \\ 0 \\ \vdots \end{bmatrix}
    =
    \begin{bmatrix}
        -I         & 0      & \dots  & \mathbf{0}  \\
        A          & -I     & \ddots & \vdots      \\
                   & \ddots & \ddots & \mathbf{0}  \\
        \mathbf{0} &        & A      & -I
    \end{bmatrix}
    \begin{bmatrix}
        \mathbf{x}_1 \\ \vdots \\ \mathbf{x}_T
    \end{bmatrix}
    +
    \begin{bmatrix}
        B          &        & \mathbf{0} \\
                   & \ddots &            \\
        \mathbf{0} &        & B
    \end{bmatrix}
    \begin{bmatrix}
        \mathbf{u}_0 \\ \vdots \\ \mathbf{u}_{T-1}
    \end{bmatrix},
$$

$$\Downarrow$$

$$
-\underbrace{
    (\mathbf{1}_T \otimes \mathbf{w} + \mathbf{e}_1 \otimes A \mathbf{x}_0)
}_{\mathbf{s}_w}
    =
    \underbrace{
        ( I_T^{(-1)} \otimes A - I_{nT} )
    }_{S_x}
    \mathbf{x}
    +
    \underbrace{
        ( I_T \otimes B )
    }_{S_u}
    \mathbf{u}.
$$

$$\Downarrow$$

$$
-\mathbf{s}_w = S_x \mathbf{x} + S_u \mathbf{u}
$$

Substituting in the input parameterization gives

$$
-\mathbf{s}_w = S_x \mathbf{x} + S_u \Psi^p(\boldsymbol{\tau}) \mathbf{c}.
$$

Finally, we can substitute in the design variable and get the model constraint:

$$
\underbrace{-\mathbf{s}_w}_{\boldsymbol{\ell}_{\mathrm{model}}}
    =
    \underbrace{
        \begin{bmatrix} S_u \Psi^p(\boldsymbol{\tau}) & S_x \end{bmatrix}
    }_{A_{\mathrm{model}}}
    \mathbf{z}_{\mathrm{s}}
    =
    \underbrace{-\mathbf{s}_w}_{\boldsymbol{\upsilon}_{\mathrm{model}}}
$$

### Cost Terms

Starting with the cost function we derived above:

$$
\begin{align}
J &= (\mathbf{x} - \mathbf{x}_{\mathrm{ref}})^\top \bar{Q} (\mathbf{x} - \mathbf{x}_{\mathrm{ref}})
    + (\mathbf{c} - \mathbf{c}_{\mathrm{ref}})^\top \bar{R} (\mathbf{c} - \mathbf{c}_{\mathrm{ref}}) \\
  &=(\mathbf{z}_{\mathrm{s}} - \mathbf{z}_{\mathrm{s},\mathrm{ref}})^\top
        \underbrace{
            \begin{bmatrix} \bar{R} & \mathbf{0} \\ \mathbf{0} & \bar{Q} \end{bmatrix}
        }_{P}
    (\mathbf{z}_{\mathrm{s}} - \mathbf{z}_{\mathrm{s},\mathrm{ref}}) \\
  &= \mathbf{z}_{\mathrm{s}}^\top P \mathbf{z}_{\mathrm{s}}
    -2 \mathbf{z}_{\mathrm{s},\mathrm{ref}}^\top P \mathbf{z}_{\mathrm{s}}
    + \underbrace{\mathbf{z}_{\mathrm{s},\mathrm{ref}}^\top P \mathbf{z}_{\mathrm{s},\mathrm{ref}}}_{\text{drop}}
\end{align}
$$

Here we see the last term has no design variables in it,
so it is constant in terms of the optimization and can be dropped since we only care about the argmin.
Scaling the cost by $\frac{1}{2}$ also leaves the argmin unchanged and matches the QP form:

$$
\frac{1}{2} J
    = \frac{1}{2} \mathbf{z}_{\mathrm{s}}^\top P \mathbf{z}_{\mathrm{s}}
    + (\, \underbrace{-P \mathbf{z}_{\mathrm{s},\mathrm{ref}}}_{\mathbf{q}} \,)^\top \mathbf{z}_{\mathrm{s}}
    + \text{const}
$$

Now we have the definition of the QP cost terms:

$$
P = \mathrm{blkdiag}(\bar{R}, \bar{Q})
    \quad \quad \quad \mathbf{q} = -P \mathbf{z}_{\mathrm{s},\mathrm{ref}}
$$

### Input Saturation

**Saturate control points:**

$$
\begin{bmatrix}
    \mathbf{u}_{\mathrm{min}} \\ \vdots \\ \mathbf{u}_{\mathrm{min}}
\end{bmatrix}
\leq
\mathbf{c}
\leq
\begin{bmatrix}
    \mathbf{u}_{\mathrm{max}} \\ \vdots \\ \mathbf{u}_{\mathrm{max}}
\end{bmatrix}
$$

$$
\Downarrow
$$

$$
\underbrace{\mathbf{1}_\eta \otimes \mathbf{u}_{\mathrm{min}}}_{\boldsymbol{\ell}_{\mathrm{usat}}}
\leq
\underbrace{
    \begin{bmatrix} I_{m\eta} & \mathbf{0}_{m\eta \times nT} \end{bmatrix}
}_{A_{\mathrm{usat}}} \mathbf{z}_{\mathrm{s}}
\leq
\underbrace{\mathbf{1}_\eta \otimes \mathbf{u}_{\mathrm{max}}}_{\boldsymbol{\upsilon}_{\mathrm{usat}}}
$$

**Saturate input trajectory:**

$$
\begin{bmatrix}
    \mathbf{u}_{\mathrm{min}} \\ \vdots \\ \mathbf{u}_{\mathrm{min}}
\end{bmatrix}
\leq
\mathbf{u}
\leq
\begin{bmatrix}
    \mathbf{u}_{\mathrm{max}} \\ \vdots \\ \mathbf{u}_{\mathrm{max}}
\end{bmatrix}
$$

$$
\Downarrow
$$

$$
\underbrace{\mathbf{1}_T \otimes \mathbf{u}_{\mathrm{min}}}_{\boldsymbol{\ell}_{\mathrm{usat}}}
\leq
\underbrace{
    \begin{bmatrix} \Psi^p(\boldsymbol{\tau}) & \mathbf{0}_{mT \times nT} \end{bmatrix}
}_{A_{\mathrm{usat}}} \mathbf{z}_{\mathrm{s}}
\leq
\underbrace{\mathbf{1}_T \otimes \mathbf{u}_{\mathrm{max}}}_{\boldsymbol{\upsilon}_{\mathrm{usat}}}
$$

For degree $p = 1$, only the rows of $\Psi^1(\boldsymbol{\tau})$ (and bounds) for the samples adjacent to each active knot are kept, which is exactly equivalent since the spline is linear between knots.
See [Input Trajectory Saturation Constraint](problem-formulation.md#input-trajectory-saturation-constraint).

### Slew Initial Input

$$
| \mathbf{u}_0 - \mathbf{u}_{-1} | \leq \mathbf{u}_{0,\mathrm{slew}}
$$

$$
\Downarrow
$$

$$
\underbrace{\mathbf{u}_{-1} - \mathbf{u}_{0,\mathrm{slew}}}_{\boldsymbol{\ell}_{\mathrm{slew0}}}
\leq
\underbrace{
    \begin{bmatrix}
        (\mathbf{e}_1^\top \otimes I_m) \Psi^p(\boldsymbol{\tau}) & \mathbf{0}_{m \times nT}
    \end{bmatrix}
}_{A_{\mathrm{slew0}}} \mathbf{z}_{\mathrm{s}}
\leq
\underbrace{\mathbf{u}_{-1} + \mathbf{u}_{0,\mathrm{slew}}}_{\boldsymbol{\upsilon}_{\mathrm{slew0}}}
$$

### Slew Control Points

$$
| \mathbf{c}_i - \mathbf{c}_{i-1} | \leq \mathbf{c}_{\mathrm{slew}} \quad i=1, \dots, \eta-1
$$

$$
\Downarrow
$$

$$
\underbrace{-\mathbf{1}_{\eta-1} \otimes \mathbf{c}_{\mathrm{slew}}}_{\boldsymbol{\ell}_{\mathrm{slew}}}
\leq
\underbrace{
    \begin{bmatrix}
        D_\eta \otimes I_m & \mathbf{0}_{m(\eta-1) \times nT} \\
    \end{bmatrix}
}_{A_{\mathrm{slew}}} \mathbf{z}_{\mathrm{s}}
\leq
\underbrace{\mathbf{1}_{\eta-1} \otimes \mathbf{c}_{\mathrm{slew}}}_{\boldsymbol{\upsilon}_{\mathrm{slew}}}
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
    \in \mathbb{R}^{(\eta-1) \times \eta}
$$

is the first-difference matrix, so the row blocks of $(D_\eta \otimes I_m) \mathbf{c}$ are $\mathbf{c}_i - \mathbf{c}_{i-1}$ for $i = 1, \dots, \eta-1$.

### State Saturation

$$
\begin{bmatrix}
    \mathbf{x}_{\mathrm{min}} \\ \vdots \\ \mathbf{x}_{\mathrm{min}}
\end{bmatrix}
\leq
\mathbf{x}
\leq
\begin{bmatrix}
    \mathbf{x}_{\mathrm{max}} \\ \vdots \\ \mathbf{x}_{\mathrm{max}}
\end{bmatrix}
$$

$$
\Downarrow
$$

$$
\underbrace{\mathbf{1}_T \otimes \mathbf{x}_{\mathrm{min}}}_{\boldsymbol{\ell}_{\mathrm{xsat}}}
\leq
\underbrace{
    \begin{bmatrix} \mathbf{0}_{nT \times m\eta} & I_{nT} \end{bmatrix}
}_{A_{\mathrm{xsat}}} \mathbf{z}_{\mathrm{s}}
\leq
\underbrace{\mathbf{1}_T \otimes \mathbf{x}_{\mathrm{max}}}_{\boldsymbol{\upsilon}_{\mathrm{xsat}}}
$$

## Condensed Formulation

This formulation cannot use predicted states in its cost or constraint functions since they are not part of the design variable.
Thus, the condensed formulation does not have the model as an explicit constraint function;
instead, the model is implicitly wrapped into the cost function.
This also means that state saturation cannot be directly performed on states in the condensed case.

The condensed formulation may be preferable when:

- the horizon is short
- the state dimension is small
- state saturation is not enabled

!!! note

    In our limited testing thus far, the condensed formulation seems to be generally faster.

### Model Constraint

To convert the model prediction into a single matrix multiply, we'll start writing out predicted states:

$$
\begin{align}
    \mathbf{x}_1 &= A \mathbf{x}_0 + B \mathbf{u}_0 + \mathbf{w} \\
    \mathbf{x}_2 &= A \mathbf{x}_1 + B \mathbf{u}_1 + \mathbf{w} \\
        &= A (A \mathbf{x}_0 + B \mathbf{u}_0 + \mathbf{w}) + B \mathbf{u}_1 + \mathbf{w} \\
        &= A^2 \mathbf{x}_0 + A (B \mathbf{u}_0 + \mathbf{w}) + (B \mathbf{u}_1 + \mathbf{w}) \\
        &\;\; \vdots \\
    \mathbf{x}_k &= A^k \mathbf{x}_0 + \sum_{i=1}^k A^{i-1} ( B \mathbf{u}_{k-i} + \mathbf{w} ) \\
\end{align}
$$

Writing this in matrix form yields

$$
\mathbf{x}
    =
    \begin{bmatrix}
        A \mathbf{x}_0 \\ A^2 \mathbf{x}_0 \\ \vdots \\ A^T \mathbf{x}_0
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
    \mathbf{u}
    +
    \begin{bmatrix}
        \mathbf{w} \\ \vdots \\ \mathbf{w}
    \end{bmatrix}
    \right)
$$

$$\Downarrow$$

$$
\mathbf{x}
    =
    % \mathrm{blkdiag}(A, A^2, \dots, A^T) (\mathbf{1}_T \otimes \mathbf{x}_0)
    \sum_{i=1}^{T}(\mathbf{e}_i \otimes A^i \mathbf{x}_0)
    +
    \left( \sum_{i=0}^{T-1} I_T^{(-i)} \otimes A^i \right)
    \left( (I_T \otimes B) \mathbf{u} + \mathbf{1}_T \otimes \mathbf{w} \right)
$$

Now if we let

$$
S = \left( \sum_{i=0}^{T-1} I_T^{(-i)} \otimes A^i \right) (I_T \otimes B) \Psi^p(\boldsymbol{\tau})
$$

$$
\mathbf{v}
    =
    % \mathrm{blkdiag}(A, A^2, \dots, A^T) (\mathbf{1}_T \otimes \mathbf{x}_0)
    \sum_{i=1}^{T}(\mathbf{e}_i \otimes A^i \mathbf{x}_0)
    +
    \left( \sum_{i=0}^{T-1} I_T^{(-i)} \otimes A^i \right) (\mathbf{1}_T \otimes \mathbf{w})
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
        \Psi^p(\boldsymbol{\tau})
    $$

    $$
    \mathbf{v} =
        \begin{bmatrix}
            A \mathbf{x}_0   + \mathbf{w} \\
            A^2 \mathbf{x}_0 + A \mathbf{w} + \mathbf{w} \\
            \vdots \\
            A^T \mathbf{x}_0 + \sum_{i=0}^{T-1} A^i \mathbf{w}
        \end{bmatrix}
    $$

then the final condensed form is

$$
\mathbf{x} = S \mathbf{z}_{\mathrm{c}} + \mathbf{v}.
$$

This gives us a model function we can use to substitute out predicted states from the cost function.
The condensed formulation does not have an explicit model constraint, so
$A_{\mathrm{model}}$,
$\boldsymbol{\ell}_{\mathrm{model}}$,
and $\boldsymbol{\upsilon}_{\mathrm{model}}$ are all empty.

### Cost Terms

Starting with the cost function we derived above:

$$
\begin{align}
J
    &=
    (\mathbf{x} - \mathbf{x}_{\mathrm{ref}})^\top \bar{Q} (\mathbf{x} - \mathbf{x}_{\mathrm{ref}})
    + (\mathbf{c} - \mathbf{c}_{\mathrm{ref}})^\top \bar{R} (\mathbf{c} - \mathbf{c}_{\mathrm{ref}}) \\
    &=
    \mathbf{x}^\top \bar{Q} \mathbf{x}
    - 2 \mathbf{x}^\top \bar{Q} \mathbf{x}_{\mathrm{ref}}
    + \underbrace{\mathbf{x}_{\mathrm{ref}}^\top \bar{Q} \mathbf{x}_{\mathrm{ref}}}_{\text{drop}}
    + \mathbf{c}^\top \bar{R} \mathbf{c}
    - 2 \mathbf{c}^\top \bar{R} \mathbf{c}_{\mathrm{ref}}
    + \underbrace{\mathbf{c}_{\mathrm{ref}}^\top \bar{R} \mathbf{c}_{\mathrm{ref}}}_{\text{drop}} \\
    &=
    \underbrace{\mathbf{x}^\top \bar{Q} \mathbf{x}
    - 2 \mathbf{x}^\top \bar{Q} \mathbf{x}_{\mathrm{ref}}}_{J_x}
    + \underbrace{\mathbf{c}^\top \bar{R} \mathbf{c}
    - 2 \mathbf{c}_{\mathrm{ref}}^\top \bar{R} \mathbf{c}}_{J_c}
\end{align}
$$

Substituting in the condensed model for the predicted state trajectory yields

$$
\begin{align}
J_x
    &=
    (S \mathbf{z}_{\mathrm{c}} + \mathbf{v})^\top \bar{Q} (S \mathbf{z}_{\mathrm{c}} + \mathbf{v})
    - 2 (S \mathbf{z}_{\mathrm{c}} + \mathbf{v})^\top \bar{Q} \mathbf{x}_{\mathrm{ref}} \\
    &=
    \mathbf{z}_{\mathrm{c}}^\top (S^\top \bar{Q} S) \mathbf{z}_{\mathrm{c}}
    + 2 \mathbf{v}^\top \bar{Q} S \mathbf{z}_{\mathrm{c}}
    + \underbrace{\mathbf{v}^\top \bar{Q} \mathbf{v}}_{\text{drop}}
    - 2 \mathbf{x}_{\mathrm{ref}}^\top \bar{Q} S \mathbf{z}_{\mathrm{c}}
    - \underbrace{2 \mathbf{v}^\top \bar{Q} \mathbf{x}_{\mathrm{ref}}}_{\text{drop}} \\
    &=
    \mathbf{z}_{\mathrm{c}}^\top (S^\top \bar{Q} S) \mathbf{z}_{\mathrm{c}}
    + 2 (S^\top \bar{Q} (\mathbf{v} - \mathbf{x}_{\mathrm{ref}}))^\top \mathbf{z}_{\mathrm{c}}
    + \text{const}
\end{align}
$$

and

$$
\begin{align}
J_c
    &=
    \mathbf{z}_{\mathrm{c}}^\top \bar{R} \mathbf{z}_{\mathrm{c}}
    - 2 \mathbf{c}_{\mathrm{ref}}^\top \bar{R} \mathbf{z}_{\mathrm{c}} \\
    &=
    \mathbf{z}_{\mathrm{c}}^\top \bar{R} \mathbf{z}_{\mathrm{c}}
    - 2 (\bar{R} \mathbf{c}_{\mathrm{ref}})^\top \mathbf{z}_{\mathrm{c}} \\
\end{align}
$$

Combining $J_x$ and $J_c$ back together and scaling by $\frac{1}{2}$
(which, like dropping constants, leaves the argmin unchanged) gives the final form

$$
\begin{align}
\frac{1}{2} J
    &=
    \frac{1}{2} \mathbf{z}_{\mathrm{c}}^\top
    \underbrace{(S^\top \bar{Q} S + \bar{R})}_{P}
    \mathbf{z}_{\mathrm{c}}
    +
    (\underbrace{
        S^\top \bar{Q} (\mathbf{v} - \mathbf{x}_{\mathrm{ref}}) - \bar{R} \mathbf{c}_{\mathrm{ref}}
    }_{\mathbf{q}})^\top \mathbf{z}_{\mathrm{c}}
    + \text{const}
\end{align}
$$

where we see that

$$
P = (S^\top \bar{Q} S + \bar{R})
$$

$$
\mathbf{q} =  S^\top \bar{Q} (\mathbf{v} - \mathbf{x}_{\mathrm{ref}}) - \bar{R} \mathbf{c}_{\mathrm{ref}}
$$

### Input Saturation

**Saturate control points:**

$$
\begin{bmatrix}
    \mathbf{u}_{\mathrm{min}} \\ \vdots \\ \mathbf{u}_{\mathrm{min}}
\end{bmatrix}
\leq
\mathbf{c}
\leq
\begin{bmatrix}
    \mathbf{u}_{\mathrm{max}} \\ \vdots \\ \mathbf{u}_{\mathrm{max}}
\end{bmatrix}
$$

$$
\Downarrow
$$

$$
\underbrace{\mathbf{1}_\eta \otimes \mathbf{u}_{\mathrm{min}}}_{\boldsymbol{\ell}_{\mathrm{usat}}}
\leq
\underbrace{I_{m\eta}}_{A_{\mathrm{usat}}} \mathbf{z}_{\mathrm{c}}
\leq
\underbrace{\mathbf{1}_\eta \otimes \mathbf{u}_{\mathrm{max}}}_{\boldsymbol{\upsilon}_{\mathrm{usat}}}
$$

**Saturate input trajectory:**

$$
\begin{bmatrix}
    \mathbf{u}_{\mathrm{min}} \\ \vdots \\ \mathbf{u}_{\mathrm{min}}
\end{bmatrix}
\leq
\mathbf{u}
\leq
\begin{bmatrix}
    \mathbf{u}_{\mathrm{max}} \\ \vdots \\ \mathbf{u}_{\mathrm{max}}
\end{bmatrix}
$$

$$
\Downarrow
$$

$$
\underbrace{\mathbf{1}_T \otimes \mathbf{u}_{\mathrm{min}}}_{\boldsymbol{\ell}_{\mathrm{usat}}}
\leq
\underbrace{ \Psi^p(\boldsymbol{\tau}) }_{A_{\mathrm{usat}}} \mathbf{z}_{\mathrm{c}}
\leq
\underbrace{\mathbf{1}_T \otimes \mathbf{u}_{\mathrm{max}}}_{\boldsymbol{\upsilon}_{\mathrm{usat}}}
$$

For degree $p = 1$, only the rows of $\Psi^1(\boldsymbol{\tau})$ (and bounds) for the samples adjacent to each active knot are kept, which is exactly equivalent since the spline is linear between knots.
See [Input Trajectory Saturation Constraint](problem-formulation.md#input-trajectory-saturation-constraint).

### Slew Initial Input

$$
| \mathbf{u}_0 - \mathbf{u}_{-1} | \leq \mathbf{u}_{0,\mathrm{slew}}
$$

$$
\Downarrow
$$

$$
\underbrace{\mathbf{u}_{-1} - \mathbf{u}_{0,\mathrm{slew}}}_{\boldsymbol{\ell}_{\mathrm{slew0}}}
\leq
\underbrace{
        (\mathbf{e}_1^\top \otimes I_m) \Psi^p(\boldsymbol{\tau})
}_{A_{\mathrm{slew0}}} \mathbf{z}_{\mathrm{c}}
\leq
\underbrace{\mathbf{u}_{-1} + \mathbf{u}_{0,\mathrm{slew}}}_{\boldsymbol{\upsilon}_{\mathrm{slew0}}}
$$

### Slew Control Points

$$
| \mathbf{c}_i - \mathbf{c}_{i-1} | \leq \mathbf{c}_{\mathrm{slew}} \quad i=1, \dots, \eta-1
$$

$$
\Downarrow
$$

$$
\underbrace{-\mathbf{1}_{\eta-1} \otimes \mathbf{c}_{\mathrm{slew}}}_{\boldsymbol{\ell}_{\mathrm{slew}}}
\leq
\underbrace{
    (D_\eta \otimes I_m)
}_{A_{\mathrm{slew}}} \mathbf{z}_{\mathrm{c}}
\leq
\underbrace{\mathbf{1}_{\eta-1} \otimes \mathbf{c}_{\mathrm{slew}}}_{\boldsymbol{\upsilon}_{\mathrm{slew}}}
$$

where $D_\eta$ is the first-difference matrix defined in the [sparse formulation](#slew-control-points).

### State Saturation

$$
\begin{bmatrix}
    \mathbf{x}_{\mathrm{min}} \\ \vdots \\ \mathbf{x}_{\mathrm{min}}
\end{bmatrix}
\leq
\mathbf{x}
\leq
\begin{bmatrix}
    \mathbf{x}_{\mathrm{max}} \\ \vdots \\ \mathbf{x}_{\mathrm{max}}
\end{bmatrix}
$$

$$
\Downarrow
$$

$$
\begin{bmatrix}
    \mathbf{x}_{\mathrm{min}} \\ \vdots \\ \mathbf{x}_{\mathrm{min}}
\end{bmatrix}
\leq
S \mathbf{z}_{\mathrm{c}} + \mathbf{v}
\leq
\begin{bmatrix}
    \mathbf{x}_{\mathrm{max}} \\ \vdots \\ \mathbf{x}_{\mathrm{max}}
\end{bmatrix}
$$

$$
\Downarrow
$$

$$
\underbrace{\mathbf{1}_T \otimes \mathbf{x}_{\mathrm{min}} - \mathbf{v}}_{\boldsymbol{\ell}_{\mathrm{xsat}}}
\leq
\underbrace{ S }_{A_{\mathrm{xsat}}} \mathbf{z}_{\mathrm{c}}
\leq
\underbrace{\mathbf{1}_T \otimes \mathbf{x}_{\mathrm{max}} - \mathbf{v}}_{\boldsymbol{\upsilon}_{\mathrm{xsat}}}
$$

!!! note

    As shown here, state saturation in the condensed formulation adds $S$ to $A_{\mathrm{qp}}$ rather than $I$.
    This makes the constraint matrix less sparse.
    Thus, it may be more efficient to use the sparse formulation when saturating states,
    but the best way to find out for your system is to try both!
    Bounding only the states that need limits (below) keeps only their rows of $S$.

## Selected and Soft State Limits

With `bounded_states` listing $n_b$ of the $n$ states, let $E \in \mathbb{R}^{n_b \times n}$ hold the matching rows of $I_n$.
The state saturation rows become $(I_T \otimes E)$ times the state rows above, with bounds $\mathbf{1}_T \otimes \mathbf{x}_{\mathrm{min}}$ and $\mathbf{1}_T \otimes \mathbf{x}_{\mathrm{max}}$ of size $n_b$ (and $-(I_T \otimes E)\mathbf{v}$ in the condensed formulation): $n_b T$ rows instead of $nT$.
In the condensed formulation these are the selected rows of $S$, so no product is needed.

Soft limits add one slack per bounded state, $\mathbf{s} \in \mathbb{R}^{n_b}$, after the formulation's variables.
A row can not hold one slack on both of its sides, so each limit becomes its own one-sided row:

$$
(I_T \otimes E)\, \mathbf{x} - \mathbf{1}_T \otimes \mathbf{s} \leq \mathbf{1}_T \otimes \mathbf{x}_{\mathrm{max}},
\qquad
(I_T \otimes E)\, \mathbf{x} + \mathbf{1}_T \otimes \mathbf{s} \geq \mathbf{1}_T \otimes \mathbf{x}_{\mathrm{min}},
\qquad
\mathbf{s} \geq \mathbf{0},
$$

which is $2 n_b T + n_b$ rows.
The penalty $J_s = \boldsymbol{\rho}_1^\top \mathbf{s} + \mathbf{s}^\top \operatorname{diag}(\boldsymbol{\rho}_2) \mathbf{s}$ is scaled by $\frac{1}{2}$ like the rest of the cost, so the slack block of $P$ is $\operatorname{diag}(\boldsymbol{\rho}_2)$ and the slack part of $\mathbf{q}$ is $\frac{1}{2} \boldsymbol{\rho}_1$.

## Input Rate Cost

The input rate cost only involves the control points, which come first in both $\mathbf{z}_{\mathrm{s}}$ and $\mathbf{z}_{\mathrm{c}}$, so it adds the same terms to both formulations.
With the difference matrix $D = (I_T - I_T^{(-1)}) \otimes I_m$, the input changes are

$$
\begin{bmatrix} \mathbf{u}_0 - \mathbf{u}_{-1} \\ \vdots \\ \mathbf{u}_{T-1} - \mathbf{u}_{T-2} \end{bmatrix}
= D \Psi^p(\boldsymbol{\tau}) \, \mathbf{c} - \mathbf{e}_1 \otimes \mathbf{u}_{-1},
$$

so with $\bar{R}_\Delta = I_T \otimes R_\Delta$,

$$
J_{\Delta u}
    =
    \mathbf{c}^\top (D \Psi^p)^\top \bar{R}_\Delta (D \Psi^p) \, \mathbf{c}
    - 2 \left( \Psi^{p\top} (\mathbf{e}_1 \otimes R_\Delta \mathbf{u}_{-1}) \right)^\top \mathbf{c}
    + \text{const},
$$

using $D^\top (\mathbf{e}_1 \otimes \mathbf{v}) = \mathbf{e}_1 \otimes \mathbf{v}$.
Scaling by $\frac{1}{2}$ as before, the control point block of $P$ gains $(D \Psi^p)^\top \bar{R}_\Delta (D \Psi^p)$ and the control point part of $\mathbf{q}$ gains $-\Psi^{p\top} (\mathbf{e}_1 \otimes R_\Delta \mathbf{u}_{-1})$, which only involves the control points that shape $\mathbf{u}_0$.

With one basis shared by every input, $(D \Psi^p)^\top \bar{R}_\Delta (D \Psi^p) = G \otimes R_\Delta$ with $G = (D_1 \Phi)^\top (D_1 \Phi)$, where $\Phi$ is the $T \times \eta$ basis and $D_1 = I_T - I_T^{(-1)}$.
$G$ is banded and depends only on the knots, so it is computed once; a weight change only rescales it.
The previous input $\mathbf{u}_{-1}$ changes after every solve, so $\mathbf{q}$ is updated at every solve when this cost is enabled.

## Implementation Mapping

At a high level,
the derivation maps to the implementation as follows:

- `Parameterization` defines the evaluation map $\Psi^p(\boldsymbol{\tau})$
- `CondensedMPC` uses the condensed relation between $\mathbf{c}$ and $\mathbf{x}$
- `SparseMPC` keeps $\mathbf{x}$ directly in the QP decision vector
- `MPCBase` manages the shared model, references, weights, constraints, and solver lifecycle

The final mathematical ingredient needed for robust usage is understanding what may be updated after initialization and why OSQP imposes fixed sparsity constraints.
That is covered in [Runtime Updates](runtime-updates.md).
