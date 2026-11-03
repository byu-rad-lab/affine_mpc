## Nomenclature

<!-- --8<-- [start:mpc-terms] -->

??? info "MPC Terms"

    | Symbol | Meaning |
    | ---    | ---     |
    | $n$    | state dimension |
    | $m$    | input dimension |
    | $n_b$  | number of bounded states (`bounded_states`; $n_b = n$ by default) |
    | $T$    | number of steps in the prediction horizon |
    | $\eta$ | number of control points that parameterize the input trajectory |
    | $\mathbf{x}_k \in \mathbb{R}^n$ | state vector at horizon step $k$ |
    | $\mathbf{u}_k \in \mathbb{R}^m$ | input vector applied at horizon step $k$ |
    | $A \in \mathbb{R}^{n \times n}$ | discrete-time state matrix |
    | $B \in \mathbb{R}^{n \times m}$ | discrete-time input matrix |
    | $\mathbf{w} \in \mathbb{R}^n$ | discrete-time model offset |
    | $\mathbf{u}_{-1} \in \mathbb{R}^m$ | previous input, applied before the current horizon |
    | $\mathbf{c}_i \in \mathbb{R}^m$ | control point vector at index $i$ |
    | $\mathbf{x}_{\mathrm{ref},k} \in \mathbb{R}^n$ | reference state at horizon step $k$ |
    | $\mathbf{c}_{\mathrm{ref},i} \in \mathbb{R}^m$ | reference control point at index $i$ |
    | $\mathbf{u}_{\mathrm{ref},k} \in \mathbb{R}^m$ | reference input at horizon step $k$ (`InputCostMode::Trajectory`) |
    | $Q \in \mathbb{R}^{n \times n} \succeq 0$ | diagonal state weights |
    | $Q_{\mathrm{f}} \in \mathbb{R}^{n \times n} \succeq 0$ | diagonal terminal state weights |
    | $R \in \mathbb{R}^{m \times m} \succeq 0$ | diagonal input weights, on the control points or the sampled inputs (`input_cost`) |
    | $R_\Delta \in \mathbb{R}^{m \times m} \succeq 0$ | diagonal input rate weights |
    | $\mathbf{u}_{\min}, \mathbf{u}_{\max} \in \mathbb{R}^m$ | input limits |
    | $\mathbf{x}_{\min}, \mathbf{x}_{\max} \in \mathbb{R}^{n_b}$ | state limits of the bounded states |
    | $\mathbf{u}_{0,\mathrm{slew}} \in \mathbb{R}^m$ | slew rate limit of the initial input |
    | $\mathbf{c}_{\mathrm{slew}} \in \mathbb{R}^m$ | slew rate limit between consecutive control points |
    | $\mathbf{s} \in \mathbb{R}^{n_b}$ | slack of the soft state bounds, one per bounded state |
    | $\boldsymbol{\rho}_1, \boldsymbol{\rho}_2 \in \mathbb{R}^{n_b}$ | L1 and L2 penalties of the slack |
    | $n_h$, $n_{h,\mathrm{f}}$ | number of general and terminal constraints |
    | $H_x \in \mathbb{R}^{n_h \times n}$, $H_u \in \mathbb{R}^{n_h \times m}$ | state and input matrices of the general constraints |
    | $\mathbf{h}_{\min}, \mathbf{h}_{\max} \in \mathbb{R}^{n_h}$ | bounds of the general constraints |
    | $H_{\mathrm{f}} \in \mathbb{R}^{n_{h,\mathrm{f}} \times n}$ | state matrix of the terminal constraints |
    | $\mathbf{h}_{\mathrm{f},\min}, \mathbf{h}_{\mathrm{f},\max} \in \mathbb{R}^{n_{h,\mathrm{f}}}$ | bounds of the terminal constraints |

<!-- --8<-- [end:mpc-terms] -->

<!-- --8<-- [start:parameterization-terms] -->

??? info "Parameterization Terms"

    | Symbol | Meaning |
    | ---    | ---     |
    | $\eta$ | number of control points |
    | $\mu$  | number of knots |
    | $p$    | degree |
    | $\mathbf{c}_i \in \mathbb{R}^m$ | control point at index $i$ |
    | $\mathbf{c}_{\mathrm{ref},i} \in \mathbb{R}^m$ | reference control point at index $i$ |
    | $\boldsymbol{\tau} = [\tau_0, \dots, \tau_{\mu-1}]$ | knot vector |
    | $b_i^p(k, \boldsymbol{\tau})$ | $i$th B-spline basis function evaluated at horizon step $k$ |
    | $\Psi^p(\boldsymbol{\tau}) \in \mathbb{R}^{mT \times m\eta}$ | B-spline basis in matrix form, mapping the stacked control points to the stacked inputs; written $\Psi$ when the arguments are clear |
    | $\Phi^p(\boldsymbol{\tau}) \in \mathbb{R}^{T \times \eta}$ | basis of one input channel, so $\Psi^p(\boldsymbol{\tau}) = \Phi^p(\boldsymbol{\tau}) \otimes I_m$; written $\Phi$ when the arguments are clear or for a custom basis |
    | $\eta_{\mathrm{r}}$ | number of refined control points bounded by `InputBoundMode::Refined` |
    | $\Omega \in \mathbb{R}^{\eta_{\mathrm{r}} \times \eta}$ | knot insertion matrix: control points to refined control points |

<!-- --8<-- [end:parameterization-terms] -->

<!-- --8<-- [start:stacked-terms] -->

???+ info "Stacked Terms"

    | Symbol | Meaning |
    | ---    | ---     |
    | $\mathbf{x} = [\mathbf{x}_1^\top,\dots,\mathbf{x}_T^\top]^\top \in \mathbb{R}^{nT}$ | stacked state trajectory |
    | $\mathbf{u} = [\mathbf{u}_0^\top,\dots,\mathbf{u}_{T-1}^\top]^\top \in \mathbb{R}^{mT}$ | stacked input trajectory |
    | $\mathbf{c} = [\mathbf{c}_0^\top,\dots,\mathbf{c}_{\eta-1}^\top]^\top \in \mathbb{R}^{m\eta}$ | stacked control points   |
    | $\mathbf{x}_{\mathrm{ref}} = [\mathbf{x}_{\mathrm{ref},1}^\top,\dots,\mathbf{x}_{\mathrm{ref},T}^\top]^\top \in \mathbb{R}^{nT}$ | stacked reference state trajectory |
    | $\mathbf{c}_{\mathrm{ref}} = [\mathbf{c}_{\mathrm{ref},0}^\top,\dots,\mathbf{c}_{\mathrm{ref},\eta-1}^\top]^\top \in \mathbb{R}^{m\eta}$ | stacked reference control points   |
    | $\mathbf{u}_{\mathrm{ref}} = [\mathbf{u}_{\mathrm{ref},0}^\top,\dots,\mathbf{u}_{\mathrm{ref},T-1}^\top]^\top \in \mathbb{R}^{mT}$ | stacked reference input trajectory |
    | $S \in \mathbb{R}^{nT \times m\eta}$, $\mathbf{v} \in \mathbb{R}^{nT}$ | condensed prediction $\mathbf{x} = S\mathbf{c} + \mathbf{v}$ (see [QP Formulation](qp-formulation.md#condensed-formulation)) |
    | $E_b \in \mathbb{R}^{n_b \times n}$ | rows of $I_n$ that select the bounded states |

<!-- --8<-- [end:stacked-terms] -->

<!-- --8<-- [start:matrix-terms] -->

???+ info "Matrix Notation"

    | Symbol | Meaning |
    | ---    | ---     |
    | $I_a$ | $a \times a$ identity matrix |
    | $I_a^{(k)}$ | $a \times a$ matrix of ones on the $k$th diagonal (below the main diagonal for $k < 0$, above for $k > 0$), so $I_n^{(0)} = I_n$ |
    | $\mathbf{0}_{a \times b}$ | $a \times b$ zero matrix |
    | $\mathbf{1}_a$ | length-$a$ vector of ones |
    | $D_\eta \in \mathbb{R}^{(\eta-1) \times \eta}$ | first-difference matrix of the control points, $(D_\eta \otimes I_m)\,\mathbf{c}$ stacks $\mathbf{c}_i - \mathbf{c}_{i-1}$ |
    | $D_T = I_T - I_T^{(-1)}$ | $T \times T$ difference matrix of the sampled inputs, whose first row keeps $\mathbf{u}_0$ |
    | $\mathbf{e}_i \in \mathbb{R}^T$ | $i$th standard basis vector ($i = 1, \dots, T$) |
    | $\mathrm{blkdiag}(\cdot)$ | block diagonal matrix of its arguments |
    | $A \otimes B$ | Kronecker product: the block matrix whose $(i,j)$ block is $a_{ij} B$ |

    In a Kronecker product the first factor sets the block pattern and the second is the repeated block.
    The stacked vectors are ordered by time step, so the identity or ones vector comes first:
    $I_T \otimes Q = \mathrm{blkdiag}(Q, \dots, Q)$ and $\mathbf{1}_T \otimes \mathbf{w} = [\mathbf{w}^\top, \dots, \mathbf{w}^\top]^\top$.

<!-- --8<-- [end:matrix-terms] -->
