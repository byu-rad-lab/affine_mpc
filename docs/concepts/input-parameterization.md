# Input Parameterization

We parameterize the dense input trajectory with a B-spline,
which reduces the number of design variables in the MPC optimization.
For a fixed number of control points,
the number of optimization variables contributed by the input trajectory is independent of the horizon length.

This can significantly reduce solve time for long horizons.
There is still computational cost in lengthening the horizon,
since the model is still propagated across all horizon steps,
but the optimization search space can be much smaller than it would be without parameterization.

For example,
a non-parameterized horizon with $T = 100$ steps and input dimension $m = 4$ yields at least $mT = 400$ input design variables.
If the same horizon is parameterized with only $\eta = 5$ control points,
that number drops to $m\eta = 20$.

Parameterization constrains how the input can vary over time,
so the optimizer is solving over a smaller and more structured family of trajectories.
That is not necessarily a drawback.
B-splines are built from continuous polynomial segments,
so they can produce smoother input trajectories and may reduce or eliminate the need for explicit slew-rate constraints in some applications.

This page gives the mathematical definition of the parameterization,
its sampled matrix form,
and the API-level factory methods provided by `Parameterization`.

## Nomenclature

### System Quantities

- $\mathbf{x}_k \in \mathbb{R}^n$: state at horizon step $k$
- $\mathbf{u}_k \in \mathbb{R}^m$: dense input applied at horizon step $k$
- $\bar{\mathbf{x}}_k \in \mathbb{R}^n$: state reference at step $k$
- $n$: state dimension
- $m$: input dimension
- $T$: number of steps in the prediction horizon

### Parameterization Quantities

- $\boldsymbol{\nu}_i \in \mathbb{R}^m$: control point at index $i$
- $\bar{\boldsymbol{\nu}}_i \in \mathbb{R}^m$: control-point reference at index $i$ when input cost is enabled
- $\eta$: number of control points
- $p$: spline degree
- $\boldsymbol{\tau} = (\tau_0, \dots, \tau_{\mu-1})$: knot vector
- $\mu$: number of knots
- $b_i^p(k; \boldsymbol{\tau})$: $i$th B-spline basis function of degree $p$, evaluated at horizon index $k$
- $M \in \mathbb{R}^{mT \times m\eta}$: sampled spline evaluation matrix mapping stacked control points to the stacked dense input trajectory

### Stacked Trajectories

- $\mathbf{X} = [\mathbf{x}_1^\top \; \mathbf{x}_2^\top \; \cdots \; \mathbf{x}_T^\top]^\top \in \mathbb{R}^{nT}$: stacked predicted state trajectory
- $\mathbf{U} = [\mathbf{u}_0^\top \; \mathbf{u}_1^\top \; \cdots \; \mathbf{u}_{T-1}^\top]^\top \in \mathbb{R}^{mT}$: stacked dense input trajectory
- $\boldsymbol{\nu} = [\boldsymbol{\nu}_0^\top \; \boldsymbol{\nu}_1^\top \; \cdots \; \boldsymbol{\nu}_{\eta-1}^\top]^\top \in \mathbb{R}^{m\eta}$: stacked control-point vector

The dense input trajectory is obtained from the control points through

$$
\mathbf{U} = M \boldsymbol{\nu}.
$$

## B-Splines

B-splines are continuous in nature,
but the MPC input trajectory

$$
\{\mathbf{u}_k\}_{k=0}^{T-1}
$$

is discrete.
We therefore define a spline over the interval $t \in [0, T-1]$ and evaluate it at the integer horizon indices.

The parameterized input trajectory is

$$
\mathbf{u}_k = \sum_{i=0}^{\eta-1} b_i^p(k; \boldsymbol{\tau}) \, \boldsymbol{\nu}_i,
$$

where $\boldsymbol{\tau}$ is a nondecreasing knot vector.
The number of knots is

$$
\mu = \eta + p + 1.
$$

The spline is valid on the active interval $[\tau_p, \tau_{\mu-p-1}]$.
For MPC,
we choose the active interval to match the horizon,
so that

$$
\tau_p = 0,
\qquad
\tau_{\mu-p-1} = T-1.
$$

The basis functions are defined by the Cox-de Boor recursion formula.

For degree zero,

$$
b_i^0(k; \boldsymbol{\tau}) =
\begin{cases}
1, & \tau_i \le k < \tau_{i+1}, \\
0, & \text{otherwise},
\end{cases}
$$

and for $p > 0$,

$$
b_i^p(k; \boldsymbol{\tau}) =
\frac{k - \tau_i}{\tau_{i+p} - \tau_i} b_i^{p-1}(k; \boldsymbol{\tau})
+ \frac{\tau_{i+p+1} - k}{\tau_{i+p+1} - \tau_{i+1}} b_{i+1}^{p-1}(k; \boldsymbol{\tau}),
$$

with the convention that terms with zero denominators are treated as zero.

## Discrete Evaluation for MPC

The spline is continuous in $t$,
but the MPC problem only needs the input at the discrete horizon indices

$$
k = 0, 1, \dots, T-1.
$$

Sampling the spline at those indices produces the dense input sequence

$$
\{\mathbf{u}_k\}_{k=0}^{T-1}.
$$

This lets the spline machinery remain continuous while matching the discrete horizon used by the optimizer and model.

## Matrix Form

The sampled spline evaluation can be written as a matrix map from the stacked control-point vector to the stacked dense input trajectory:

$$
\mathbf{U} = M \boldsymbol{\nu}.
$$

Here,
$M$ is the sampled spline evaluation matrix.
Its row blocks are built by evaluating the basis functions $b_i^p(k; \boldsymbol{\tau})$ at the horizon indices.

This matrix form is the bridge from spline parameterization to the QP derivation.
The next concepts page uses it to write the predicted state trajectory and cost in quadratic form.

## Provided Factory Methods

### Move-Blocking

Move-blocking holds the input constant over specified intervals.
It is the degree-zero special case of the B-spline framework.
The `Parameterization` class provides `moveBlocking()` for this purpose.

The two overloads are:

=== "Python"

    ```python
    Parameterization.moveBlocking(horizon_steps,
                                  num_control_points)
    Parameterization.moveBlocking(horizon_steps,
                                  change_points)
    ```

=== "C++"

    ```cpp
    Parameterization::moveBlocking(horizon_steps,
                                   num_control_points)
    Parameterization::moveBlocking(horizon_steps,
                                   change_points)
    ```

The first creates a uniform parameterization with equally sized blocks.
The second uses explicitly specified change points and therefore allows variable block widths.

### Linear Interpolation

Linear interpolation is the degree-one special case.
The input becomes piecewise linear over the horizon.
The `Parameterization` class provides `linearInterp()` for this purpose.

The two overloads are:

=== "Python"

    ```python
    Parameterization.linearInterp(horizon_steps,
                                  num_control_points)
    Parameterization.linearInterp(horizon_steps,
                                  endpoints)
    ```

=== "C++"

    ```cpp
    Parameterization::linearInterp(horizon_steps,
                                   num_control_points)
    Parameterization::linearInterp(horizon_steps,
                                   endpoints)
    ```

The first creates equally spaced segment endpoints.
The second accepts custom endpoints,
which allows variable segment lengths.

### Clamped B-Spline

The general `bspline()` factory method creates clamped B-splines of the specified degree.
Clamping means the endpoint knots are repeated,
so the trajectory begins at the first control point and ends at the last.
In general,
a B-spline does not pass through all control points;
it only interpolates control points where the knot multiplicity is high enough.

The two overloads are:

=== "Python"

    ```python
    Parameterization.bspline(horizon_steps,
                             degree,
                             num_control_points)
    Parameterization.bspline(horizon_steps,
                             degree,
                             active_knots)
    ```

=== "C++"

    ```cpp
    Parameterization::bspline(horizon_steps,
                              degree,
                              num_control_points)
    Parameterization::bspline(horizon_steps,
                              degree,
                              active_knots)
    ```

The first creates a uniform clamped spline.
The second accepts custom active knots,
which allows nonuniform knot spacing.
The active knots must start at $0$ and end at $T-1$ so that the spline is valid over the MPC horizon.

## Tradeoffs and Interpretation

### Number of control points $\eta$

- Smaller $\eta$ reduces the optimization dimension and can significantly improve solve time.
- Larger $\eta$ gives the optimizer more freedom to shape the dense input trajectory.

### Degree $p$

- Lower degree gives a more local and less smooth trajectory representation.
- Higher degree gives smoother trajectories and broader basis support.

### Knot placement

- Uniform knots distribute flexibility evenly across the horizon.
- Custom knots or change points concentrate flexibility where it is most needed.

Parameterization is therefore both a dimensionality-reduction tool and a way to shape the family of trajectories the optimizer may select.

## Next Step

The next step is to substitute the relation

$$
\mathbf{U} = M \boldsymbol{\nu}
$$

into the stacked prediction model and derive the QP matrices.
That derivation is given in [QP Formulation](qp-formulation.md).
