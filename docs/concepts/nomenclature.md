## Nomenclature

<!-- --8<-- [start:mpc-terms] -->

??? info "MPC Terms"

    | Symbol | Meaning |
    | ---    | ---     |
    | $n$    | state dimension |
    | $m$    | input dimension |
    | $T$    | number of steps in the prediction horizon |
    | $\eta$ | number of control points that parameterize the input trajectory |
    | $\mathbf{x}_k \in \mathbb{R}^n$ | state vector at horizon step $k$ |
    | $\mathbf{u}_k \in \mathbb{R}^m$ | input vector applied at horizon step $k$ |
    | $\mathbf{c}_i \in \mathbb{R}^m$ | control point vector at index $i$ |
    | $\bar{\mathbf{x}}_k \in \mathbb{R}^n$ | reference state at horizon step $k$ |
    | $\bar{\mathbf{c}}_i \in \mathbb{R}^m$ | reference control point at index $i$ |
    | $Q \in \mathbb{R}^{n \times n} \succeq 0$ | diagonal state weights |
    | $Q_f \in \mathbb{R}^{n \times n} \succeq 0$ | diagonal terminal state weights |
    | $R \in \mathbb{R}^{m \times m} \succeq 0$ | diagonal input (control point) weights |

<!-- --8<-- [end:mpc-terms] -->

<!-- --8<-- [start:parameterization-terms] -->

??? info "Parameterization Terms"

    | Symbol | Meaning |
    | ---    | ---     |
    | $\eta$ | number of control points |
    | $\mu$  | number of knots |
    | $p$    | degree |
    | $\mathbf{c}_i \in \mathbb{R}^m$ | control point at index $i$ |
    | $\bar{\mathbf{c}}_i \in \mathbb{R}^m$ | reference control point at index $i$ |
    | $\boldsymbol{\tau} = [\tau_0, \dots, \tau_{\mu-1}]$ | knot vector |
    | $b_i^p(k, \boldsymbol{\tau})$ | $i$th B-spline basis function evaluated at horizon step $k$ |
    | $\Psi^p(\boldsymbol{\tau})$ | B-spline basis in matrix form |

<!-- --8<-- [end:parameterization-terms] -->

<!-- --8<-- [start:stacked-terms] -->

???+ info "Stacked Terms"

    | Symbol | Meaning |
    | ---    | ---     |
    | $\mathbf{x} = [\mathbf{x}_1^\top,\dots,\mathbf{x}_T^\top]^\top \in \mathbb{R}^{nT}$ | stacked state trajectory |
    | $\mathbf{u} = [\mathbf{u}_0^\top,\dots,\mathbf{u}_{T-1}^\top]^\top \in \mathbb{R}^{mT}$ | stacked input trajectory |
    | $\mathbf{c} = [\mathbf{c}_0^\top,\dots,\mathbf{c}_{\eta-1}^\top]^\top \in \mathbb{R}^{m\eta}$ | stacked control points   |
    | $\bar{\mathbf{x}} = [\bar{\mathbf{x}}_1^\top,\dots,\bar{\mathbf{x}}_T^\top]^\top \in \mathbb{R}^{nT}$ | stacked reference state trajectory |
    | $\bar{\mathbf{c}} = [\bar{\mathbf{c}}_0^\top,\dots,\bar{\mathbf{c}}_{\eta-1}^\top]^\top \in \mathbb{R}^{m\eta}$ | stacked reference control points   |

<!-- --8<-- [end:stacked-terms] -->

<!-- ### MPC Terms -->
<!---->
<!-- - $n$: state dimension -->
<!-- - $m$: input dimension -->
<!-- - $T$: number of steps in the prediction horizon -->
<!-- - $\eta$: number of control points -->
<!-- - $\mathbf{x}_k \in \mathbb{R}^n$: state vector at horizon step $k$ -->
<!-- - $\mathbf{u}_k \in \mathbb{R}^m$: input vector applied at horizon step $k$ -->
<!-- - $\mathbf{c}_i \in \mathbb{R}^m$: control point vector at index $i$ -->
<!-- - $\bar{\mathbf{x}}_k \in \mathbb{R}^n$: reference state at step $k$ -->
<!-- - $\bar{\mathbf{c}}_i \in \mathbb{R}^m$: reference control point at step $i$ -->
<!-- - $Q \in \mathbb{R}^{n \times n} \succeq 0$: diagonal state weights -->
<!-- - $Q_f \in \mathbb{R}^{n \times n} \succeq 0$: diagonal terminal state weights -->
<!-- - $R \in \mathbb{R}^{m \times m} \succeq 0$: diagonal input (control point) weights -->
<!---->
<!-- ### Parameterization Quantities -->
<!---->
<!-- - $\eta$: number of control points -->
<!-- - $p$: spline degree -->
<!-- - $\mu$: number of knots -->
<!-- - $\mathbf{c}_i \in \mathbb{R}^m$: control point at index $i$ -->
<!-- - $\bar{\mathbf{c}}_i \in \mathbb{R}^m$: reference control point at index $i$ -->
<!-- - $\boldsymbol{\tau} = [\tau_0, \dots, \tau_{\mu-1}]$: knot vector -->
<!-- - $b_i^p(k, \boldsymbol{\tau})$: $i$th B-spline basis function of degree $p$, evaluated at horizon index $k$ -->
<!-- - $\Psi^p(\boldsymbol{\tau}) \in \mathbb{R}^{mT \times m\eta}$: sampled spline evaluation matrix mapping stacked control points to the stacked input trajectory -->
<!---->
<!-- - $\eta$: number of control points -->
<!-- - $p$: spline degree -->
<!-- - $\mu$: number of knots -->
<!-- - $\boldsymbol{\nu}_i \in \mathbb{R}^m$: control point at index $i$ -->
<!-- - $\bar{\boldsymbol{\nu}}_i \in \mathbb{R}^m$: reference control point at index $i$ -->
<!-- - $\boldsymbol{\tau} = [\tau_0, \dots, \tau_{\mu-1}]$: knot vector -->
<!-- - $b_i^p(k, \boldsymbol{\tau})$: $i$th B-spline basis function of degree $p$, evaluated at horizon index $k$ -->
<!-- - $\Psi^p(\boldsymbol{\tau}) \in \mathbb{R}^{mT \times m\eta}$: sampled spline evaluation matrix mapping stacked control points to the stacked input trajectory -->
<!---->
<!-- ### Stacked Trajectories -->
<!---->
<!-- - $X = [\mathbf{x}_1^\top \; \mathbf{x}_2^\top \; \cdots \; \mathbf{x}_T^\top]^\top \in \mathbb{R}^{nT}$: stacked predicted state trajectory -->
<!-- - $U = [\mathbf{u}_0^\top \; \mathbf{u}_1^\top \; \cdots \; \mathbf{u}_{T-1}^\top]^\top \in \mathbb{R}^{mT}$: stacked input trajectory -->
<!-- - $\mathcal{V} = [\boldsymbol{\nu}_0^\top \; \boldsymbol{\nu}_1^\top \; \cdots \; \boldsymbol{\nu}_{\eta-1}^\top]^\top \in \mathbb{R}^{m\eta}$: stacked control-point vector -->
<!---->
<!-- - $X = [\mathbf{x}_1^\top \; \mathbf{x}_2^\top \; \cdots \; \mathbf{x}_T^\top]^\top \in \mathbb{R}^{nT}$: stacked predicted state trajectory -->
<!-- - $U = [\mathbf{u}_0^\top \; \mathbf{u}_1^\top \; \cdots \; \mathbf{u}_{T-1}^\top]^\top \in \mathbb{R}^{mT}$: stacked input trajectory -->
<!-- - $C = [\mathbf{c}_0^\top \; \mathbf{c}_1^\top \; \cdots \; \mathbf{c}_{\eta-1}^\top]^\top \in \mathbb{R}^{m\eta}$: stacked control-point vector -->
