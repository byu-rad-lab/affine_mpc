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
\mathbf{z}_c
    = \mathbf{c} \quad \quad \quad \mathbf{z}_s
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
J = \left\lVert \bar{\mathbf{x}}_T - \mathbf{x}_T \right\rVert^2_{Q_f}
    + \sum_{k=1}^{T-1} \left\lVert \bar{\mathbf{x}}_k - \mathbf{x}_k \right\rVert^2_Q
    + \sum_{i=0}^{\eta-1} \left\lVert \bar{\mathbf{c}}_i - \mathbf{c}_i \right\rVert^2_R
$$

Since we're optimizing a norm, it doesn't matter which term is being subtracted.
We will switch to use value minus reference in the QP conversion just to make the value positive,
but it can be done either way.

First, we will create block diagonal matrices using the weights:

$$
\bar{Q} = \mathrm{blkdiag}(Q, \dots, Q, Q_f)
    \quad \quad \quad \bar{R} = R \otimes I_\eta
$$

Now the cost function can be written as

$$
J = (\mathbf{x} - \bar{\mathbf{x}})^\top \bar{Q} (\mathbf{x} - \bar{\mathbf{x}})
    + (\mathbf{c} - \bar{\mathbf{c}})^\top \bar{R} (\mathbf{c} - \bar{\mathbf{c}})
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
The design variable $\mathbf{z}_s$ has states and inputs
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
        \mathbf{u}_1 \\ \vdots \\ \mathbf{u}_T
    \end{bmatrix},
$$

$$\Downarrow$$

$$
-\underbrace{
    (\mathbf{w} \otimes \mathbf{1}_T + A \mathbf{x}_0 \otimes \mathbf{e}_1)
}_{\mathbf{s}_w}
    =
    \underbrace{
        ( A \otimes I_T^{(-1)} - I_{nT} )
    }_{S_x}
    \mathbf{x}
    +
    \underbrace{
        ( B \otimes I_T )
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
    \mathbf{z}_s
    =
    \underbrace{-\mathbf{s}_w}_{\boldsymbol{\upsilon}_{\mathrm{model}}}
$$

### Cost Terms

Starting with the cost function we derived above:

$$
\begin{align}
J &= (\mathbf{x} - \bar{\mathbf{x}})^\top \bar{Q} (\mathbf{x} - \bar{\mathbf{x}})
    + (\mathbf{c} - \bar{\mathbf{c}})^\top \bar{R} (\mathbf{c} - \bar{\mathbf{c}}) \\
  &=(\mathbf{z}_s - \bar{\mathbf{z}}_s)^\top
        \underbrace{
            \begin{bmatrix} \bar{R} & \mathbf{0} \\ \mathbf{0} & \bar{Q} \end{bmatrix}
        }_{P}
    (\mathbf{z}_s - \bar{\mathbf{z}}_s) \\
  &= \mathbf{z}_s^\top P \mathbf{z}_s
    -2 \bar{\mathbf{z}}_s^\top P \mathbf{z}_s
    + \underbrace{\bar{\mathbf{z}}_s^\top P \bar{\mathbf{z}}_s}_{\text{drop}} \\
  &= \frac{1}{2} \mathbf{z}_s^\top P \mathbf{z}_s
    + (\underbrace{-P \bar{\mathbf{z}}_s}_{\mathbf{q}})^\top \mathbf{z}_s
\end{align}
$$

Here we see the last term has no design variables in it,
so it is constant in terms of the optimization and can be dropped since we only care about the argmin.
Now we have the definition of the QP cost terms
(note that the 2 is constant and can also be dropped):

$$
P = \mathrm{blkdiag}(\bar{R}, \bar{Q})
    \quad \quad \quad \mathbf{q} = -P \bar{\mathbf{z}}_s
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
\underbrace{\mathbf{u}_{\mathrm{min}} \otimes \mathbf{1}_\eta}_{\boldsymbol{\ell}_{\mathrm{usat}}}
\leq
\underbrace{
    \begin{bmatrix} I_{m\eta} & \mathbf{0}_{m\eta \times nT} \end{bmatrix}
}_{A_{\mathrm{usat}}} \mathbf{z}_s
\leq
\underbrace{\mathbf{u}_{\mathrm{max}} \otimes \mathbf{1}_\eta}_{\boldsymbol{\upsilon}_{\mathrm{usat}}}
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
\underbrace{\mathbf{u}_{\mathrm{min}} \otimes \mathbf{1}_T}_{\boldsymbol{\ell}_{\mathrm{usat}}}
\leq
\underbrace{
    \begin{bmatrix} \Psi^p(\boldsymbol{\tau}) & \mathbf{0}_{mT \times nT} \end{bmatrix}
}_{A_{\mathrm{usat}}} \mathbf{z}_s
\leq
\underbrace{\mathbf{u}_{\mathrm{max}} \otimes \mathbf{1}_T}_{\boldsymbol{\upsilon}_{\mathrm{usat}}}
$$

### Slew Initial Input

$$
| \mathbf{u}_0 - \mathbf{u}_{-1} | \leq \mathbf{u}_{0,\mathrm{slew}}
$$

$$
\Downarrow
$$

$$
\underbrace{-\mathbf{u}_{0,\mathrm{slew}}}_{\boldsymbol{\ell}_{\mathrm{slew0}}}
\leq
\underbrace{
    \begin{bmatrix}
        I_{m \times mT} \Psi^p(\boldsymbol{\tau}) & \mathbf{0}_{m \times nT}
    \end{bmatrix}
}_{A_{\mathrm{slew0}}} \mathbf{z}_s
\leq
\underbrace{\mathbf{u}_{0,\mathrm{slew}}}_{\boldsymbol{\upsilon}_{\mathrm{slew0}}}
$$

### Slew Control Points

$$
| \mathbf{c}_i - \mathbf{c}_{i-1} | \leq \mathbf{c}_{\mathrm{slew}} \quad i=1, \dots, \eta-1
$$

$$
\Downarrow
$$

$$
\underbrace{-\mathbf{c}_{\mathrm{slew}} \otimes \mathbf{1}_{\eta-1}}_{\boldsymbol{\ell}_{\mathrm{slew}}}
\leq
\underbrace{
    \begin{bmatrix}
        -I_{m(\eta-1)} + I_{m(\eta-1)}^{(1)} & \mathbf{0}_{m(\eta-1) \times nT} \\
    \end{bmatrix}
}_{A_{\mathrm{slew}}} \mathbf{z}_s
\leq
\underbrace{\mathbf{c}_{\mathrm{slew}} \otimes \mathbf{1}_{\eta-1}}_{\boldsymbol{\upsilon}_{\mathrm{slew}}}
$$

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
\underbrace{\mathbf{x}_{\mathrm{min}} \otimes \mathbf{1}_T}_{\boldsymbol{\ell}_{\mathrm{xsat}}}
\leq
\underbrace{
    \begin{bmatrix} \mathbf{0}_{nT \times m\eta} & I_{nT} \end{bmatrix}
}_{A_{\mathrm{xsat}}} \mathbf{z}_s
\leq
\underbrace{\mathbf{x}_{\mathrm{max}} \otimes \mathbf{1}_T}_{\boldsymbol{\upsilon}_{\mathrm{xsat}}}
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
    % \mathrm{blkdiag}(A, A^2, \dots, A^T) (\mathbf{x}_0 \otimes \mathbf{1}_T)
    \sum_{i=1}^{T}(A^i \mathbf{x}_0 \otimes \mathbf{e}_i)
    +
    \left( \sum_{i=0}^{T-1} A^i \otimes I_T^{(-i)} \right)
    \left( (B \otimes I_T) \mathbf{u} + \mathbf{w} \otimes \mathbf{1}_T \right)
$$

Now if we let

$$
S = \left( \sum_{i=0}^{T-1} A^i \otimes I_T^{(-i)} \right) (B \otimes I_T) \Psi^p(\boldsymbol{\tau})
$$

$$
\mathbf{v}
    =
    % \mathrm{blkdiag}(A, A^2, \dots, A^T) (\mathbf{x}_0 \otimes \mathbf{1}_T)
    \sum_{i=1}^{T}(A^i \mathbf{x}_0 \otimes \mathbf{e}_i)
    +
    \left( \sum_{i=0}^{T-1} A^i \otimes I_T^{(-i)} \right) (\mathbf{w} \otimes \mathbf{1}_T)
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
\mathbf{x} = S \mathbf{z}_c + \mathbf{v}.
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
    (\mathbf{x} - \bar{\mathbf{x}})^\top \bar{Q} (\mathbf{x} - \bar{\mathbf{x}})
    + (\mathbf{c} - \bar{\mathbf{c}})^\top \bar{R} (\mathbf{c} - \bar{\mathbf{c}}) \\
    &=
    \mathbf{x}^\top \bar{Q} \mathbf{x}
    - 2 \mathbf{x}^\top \bar{Q} \bar{\mathbf{x}}
    + \underbrace{\bar{\mathbf{x}}^\top \bar{Q} \bar{\mathbf{x}}}_{\text{drop}}
    + \mathbf{c}^\top \bar{R} \mathbf{c}
    - 2 \mathbf{c}^\top \bar{R} \bar{\mathbf{c}}
    + \underbrace{\bar{\mathbf{c}}^\top \bar{R} \bar{\mathbf{c}}}_{\text{drop}} \\
    &=
    \underbrace{\mathbf{x}^\top \bar{Q} \mathbf{x}
    - 2 \mathbf{x}^\top \bar{Q} \bar{\mathbf{x}}}_{J_x}
    + \underbrace{\mathbf{c}^\top \bar{R} \mathbf{c}
    - 2 \bar{\mathbf{c}}^\top \bar{R} \mathbf{c}}_{J_c}
\end{align}
$$

Substituting in the condensed model for the predicted state trajectory yields

$$
\begin{align}
J_x
    &=
    (S \mathbf{z}_c + \mathbf{v})^\top \bar{Q} (S \mathbf{z}_c + \mathbf{v})
    - 2 (S \mathbf{z}_c + \mathbf{v})^\top \bar{Q} \bar{\mathbf{x}} \\
    &=
    \mathbf{z}_c^\top (S^T \bar{Q} S) \mathbf{z}_c
    + 2 \mathbf{v}^\top \bar{Q} S \mathbf{z}_c
    + \underbrace{\mathbf{v}^\top \bar{Q} \mathbf{v}}_{\text{drop}}
    - 2 \bar{\mathbf{x}}^\top \bar{Q} S \mathbf{z}_c
    - \underbrace{2 \mathbf{v}^\top \bar{Q} \bar{\mathbf{x}}}_{\text{drop}} \\
    &=
    \frac{1}{2} \mathbf{z}_c^\top (S^T \bar{Q} S) \mathbf{z}_c
    + (S^\top \bar{Q} (\mathbf{v} - \bar{\mathbf{x}}))^\top \mathbf{z}_c
\end{align}
$$

and

$$
\begin{align}
J_c
    &=
    \mathbf{z}_c^\top \bar{R} \mathbf{z}_c
    - 2 \bar{\mathbf{c}}^\top \bar{R} \mathbf{z}_c \\
    &=
    \frac{1}{2} \mathbf{z}_c^\top \bar{R} \mathbf{z}_c
    - (\bar{R} \bar{\mathbf{c}})^\top \mathbf{z}_c \\
\end{align}
$$

Combining $J_x$ and $J_c$ back together gives the final form

$$
\begin{align}
J
    &=
    \frac{1}{2} \mathbf{z}_c^\top
    \underbrace{(S^T \bar{Q} S + \bar{R})}_{P}
    \mathbf{z}_c
    +
    (\underbrace{
        S^\top \bar{Q} (\mathbf{v} - \bar{\mathbf{x}}) - \bar{R} \bar{\mathbf{c}}
    }_{\mathbf{q}})^\top \mathbf{z}_c
\end{align}
$$

where we see that

$$
P = (S^T \bar{Q} S + \bar{R})
$$

$$
\mathbf{q} =  S^\top \bar{Q} (\mathbf{v} - \bar{\mathbf{x}}) - \bar{R} \bar{\mathbf{c}}
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
\underbrace{\mathbf{u}_{\mathrm{min}} \otimes \mathbf{1}_\eta}_{\boldsymbol{\ell}_{\mathrm{usat}}}
\leq
\underbrace{I_{m\eta}}_{A_{\mathrm{usat}}} \mathbf{z}_c
\leq
\underbrace{\mathbf{u}_{\mathrm{max}} \otimes \mathbf{1}_\eta}_{\boldsymbol{\upsilon}_{\mathrm{usat}}}
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
\underbrace{\mathbf{u}_{\mathrm{min}} \otimes \mathbf{1}_T}_{\boldsymbol{\ell}_{\mathrm{usat}}}
\leq
\underbrace{ \Psi^p(\boldsymbol{\tau}) }_{A_{\mathrm{usat}}} \mathbf{z}_c
\leq
\underbrace{\mathbf{u}_{\mathrm{max}} \otimes \mathbf{1}_T}_{\boldsymbol{\upsilon}_{\mathrm{usat}}}
$$

### Slew Initial Input

$$
| \mathbf{u}_0 - \mathbf{u}_{-1} | \leq \mathbf{u}_{0,\mathrm{slew}}
$$

$$
\Downarrow
$$

$$
\underbrace{-\mathbf{u}_{0,\mathrm{slew}}}_{\boldsymbol{\ell}_{\mathrm{slew0}}}
\leq
\underbrace{
        I_{m \times mT} \Psi^p(\boldsymbol{\tau})
}_{A_{\mathrm{slew0}}} \mathbf{z}_c
\leq
\underbrace{\mathbf{u}_{0,\mathrm{slew}}}_{\boldsymbol{\upsilon}_{\mathrm{slew0}}}
$$

### Slew Control Points

$$
| \mathbf{c}_i - \mathbf{c}_{i-1} | \leq \mathbf{c}_{\mathrm{slew}} \quad i=1, \dots, \eta-1
$$

$$
\Downarrow
$$

$$
\underbrace{-\mathbf{c}_{\mathrm{slew}} \otimes \mathbf{1}_{\eta-1}}_{\boldsymbol{\ell}_{\mathrm{slew}}}
\leq
\underbrace{
    \begin{bmatrix}
        -I_{m(\eta-1)} + I_{m(\eta-1)}^{(1)}
    \end{bmatrix}
}_{A_{\mathrm{slew}}} \mathbf{z}_c
\leq
\underbrace{\mathbf{c}_{\mathrm{slew}} \otimes \mathbf{1}_{\eta-1}}_{\boldsymbol{\upsilon}_{\mathrm{slew}}}
$$

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
S \mathbf{z}_c + \mathbf{v}
\leq
\begin{bmatrix}
    \mathbf{x}_{\mathrm{max}} \\ \vdots \\ \mathbf{x}_{\mathrm{max}}
\end{bmatrix}
$$

$$
\Downarrow
$$

$$
\underbrace{\mathbf{x}_{\mathrm{min}} \otimes \mathbf{1}_T - \mathbf{v}}_{\boldsymbol{\ell}_{\mathrm{xsat}}}
\leq
\underbrace{ S }_{A_{\mathrm{xsat}}} \mathbf{z}_c
\leq
\underbrace{\mathbf{x}_{\mathrm{max}} \otimes \mathbf{1}_T - \mathbf{v}}_{\boldsymbol{\upsilon}_{\mathrm{xsat}}}
$$

!!! note

    As shown here, state saturation in the condensed formulation adds $S$ to $A_{\mathrm{qp}}$ rather than $I$.
    This makes the constraint matrix less sparse.
    Thus, it may be more efficient to use the sparse formulation when saturating states,
    but the best way to find out for your system is to try both!

## Implementation Mapping

At a high level,
the derivation maps to the implementation as follows:

- `Parameterization` defines the evaluation map $\Psi^p(\boldsymbol{\tau})$
- `CondensedMPC` uses the condensed relation between $\mathbf{c}$ and $\mathbf{x}$
- `SparseMPC` keeps $\mathbf{x}$ directly in the QP decision vector
- `MPCBase` manages the shared model, references, weights, constraints, and solver lifecycle

The final mathematical ingredient needed for robust usage is understanding what may be updated after initialization and why OSQP imposes fixed sparsity constraints.
That is covered in [Runtime Updates](runtime-updates.md).
