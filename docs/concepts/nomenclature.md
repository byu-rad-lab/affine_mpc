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
