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
    | $\bm{x}_k \in \R^n$ | state vector at horizon step $k$ |
    | $\bm{u}_k \in \R^m$ | input vector applied at horizon step $k$ |
    | $A \in \R^{n \times n}$ | discrete-time state matrix |
    | $B \in \R^{n \times m}$ | discrete-time input matrix |
    | $\bm{w} \in \R^n$ | discrete-time model offset |
    | $\bm{u}_{-1} \in \R^m$ | previous input, applied before the current horizon |
    | $\bm{c}_i \in \R^m$ | control point vector at index $i$ |
    | $\bm{x}_{\mathrm{ref},k} \in \R^n$ | reference state at horizon step $k$ |
    | $\bm{c}_{\mathrm{ref},i} \in \R^m$ | reference control point at index $i$ |
    | $\bm{u}_{\mathrm{ref},k} \in \R^m$ | reference input at horizon step $k$ (`InputCostMode::Trajectory`) |
    | $Q \in \R^{n \times n} \succeq 0$ | diagonal state weights |
    | $Q_{\mathrm{f}} \in \R^{n \times n} \succeq 0$ | diagonal terminal state weights |
    | $R \in \R^{m \times m} \succeq 0$ | diagonal input weights, on the control points or the sampled inputs (`input_cost`) |
    | $R_\Delta \in \R^{m \times m} \succeq 0$ | diagonal input rate weights |
    | $\bm{u}_{\min}, \bm{u}_{\max} \in \R^m$ | input limits |
    | $\bm{x}_{\min}, \bm{x}_{\max} \in \R^{n_b}$ | state limits of the bounded states |
    | $\bm{u}_{0,\mathrm{slew}} \in \R^m$ | slew rate limit of the initial input |
    | $\bm{u}_{\mathrm{slew}} \in \R^m$ | slew rate limit per sample step, on $\bm{u}_k - \bm{u}_{k-1}$ for $k \geq 1$ |
    | $\bm{s} \in \R^{n_b}$ | slack of the soft state bounds, one per bounded state |
    | $\bm{\rho}_1, \bm{\rho}_2 \in \R^{n_b}$ | L1 and L2 penalties of the slack |
    | $n_h$, $n_{h,\mathrm{f}}$ | number of general and terminal constraints |
    | $H_x \in \R^{n_h \times n}$, $H_u \in \R^{n_h \times m}$ | state and input matrices of the general constraints |
    | $\bm{h}_{\min}, \bm{h}_{\max} \in \R^{n_h}$ | bounds of the general constraints |
    | $H_{\mathrm{f}} \in \R^{n_{h,\mathrm{f}} \times n}$ | state matrix of the terminal constraints |
    | $\bm{h}_{\mathrm{f},\min}, \bm{h}_{\mathrm{f},\max} \in \R^{n_{h,\mathrm{f}}}$ | bounds of the terminal constraints |

<!-- --8<-- [end:mpc-terms] -->

<!-- --8<-- [start:parameterization-terms] -->

??? info "Parameterization Terms"

    | Symbol | Meaning |
    | ---    | ---     |
    | $\eta$ | number of control points |
    | $\mu$  | number of knots |
    | $p$    | degree |
    | $\bm{c}_i \in \R^m$ | control point at index $i$ (the coefficient of the $i$th basis column for a custom basis) |
    | $\bm{c}_{\mathrm{ref},i} \in \R^m$ | reference control point at index $i$ |
    | $\bm{\tau} = [\tau_0, \dots, \tau_{\mu-1}]$ | knot vector |
    | $b_i^p(k, \bm{\tau})$ | $i$th B-spline basis function evaluated at horizon step $k$ |
    | $\Psi^p(\bm{\tau}) \in \R^{mT \times m\eta}$ | B-spline basis in matrix form, mapping the stacked control points to the stacked inputs; written $\Psi$ when the arguments are clear |
    | $\Phi^p(\bm{\tau}) \in \R^{T \times \eta}$ | basis of one input channel, so $\Psi^p(\bm{\tau}) = \Phi^p(\bm{\tau}) \otimes I_m$; written $\Phi$ when the arguments are clear or for a custom basis |
    | $\eta_{\mathrm{r}}$ | number of refined control points bounded by `InputBoundMode::Refined` |
    | $\Omega \in \R^{\eta_{\mathrm{r}} \times \eta}$ | knot insertion matrix: control points to refined control points |
    | $\lambda_i$ | slew bound factor of $\bm{c}_i - \bm{c}_{i-1}$: $(\tau_{i+p} - \tau_i) / p$, or 1 for $p = 0$ |

<!-- --8<-- [end:parameterization-terms] -->

<!-- --8<-- [start:stacked-terms] -->

???+ info "Stacked Terms"

    | Symbol | Meaning |
    | ---    | ---     |
    | $\bm{x} = [\bm{x}_1^\T,\dots,\bm{x}_T^\T]^\T \in \R^{nT}$ | stacked state trajectory |
    | $\bm{u} = [\bm{u}_0^\T,\dots,\bm{u}_{T-1}^\T]^\T \in \R^{mT}$ | stacked input trajectory |
    | $\bm{c} = [\bm{c}_0^\T,\dots,\bm{c}_{\eta-1}^\T]^\T \in \R^{m\eta}$ | stacked control points   |
    | $\bm{x}_{\mathrm{ref}} = [\bm{x}_{\mathrm{ref},1}^\T,\dots,\bm{x}_{\mathrm{ref},T}^\T]^\T \in \R^{nT}$ | stacked reference state trajectory |
    | $\bm{c}_{\mathrm{ref}} = [\bm{c}_{\mathrm{ref},0}^\T,\dots,\bm{c}_{\mathrm{ref},\eta-1}^\T]^\T \in \R^{m\eta}$ | stacked reference control points   |
    | $\bm{u}_{\mathrm{ref}} = [\bm{u}_{\mathrm{ref},0}^\T,\dots,\bm{u}_{\mathrm{ref},T-1}^\T]^\T \in \R^{mT}$ | stacked reference input trajectory |
    | $S \in \R^{nT \times m\eta}$, $\bm{v} \in \R^{nT}$ | condensed prediction $\bm{x} = S\bm{c} + \bm{v}$ (see [QP Formulation](qp-formulation.md#condensed-formulation)) |
    | $E_b \in \R^{n_b \times n}$ | rows of $I_n$ that select the bounded states |

<!-- --8<-- [end:stacked-terms] -->

<!-- --8<-- [start:matrix-terms] -->

???+ info "Matrix Notation"

    | Symbol | Meaning |
    | ---    | ---     |
    | $I_a$ | $a \times a$ identity matrix |
    | $I_a^{(k)}$ | $a \times a$ matrix of ones on the $k$th diagonal (below the main diagonal for $k < 0$, above for $k > 0$), so $I_n^{(0)} = I_n$ |
    | $\mathbf{0}_{a \times b}$ | $a \times b$ zero matrix |
    | $\mathbf{1}_a$ | length-$a$ vector of ones |
    | $D_\eta \in \R^{(\eta-1) \times \eta}$ | first-difference matrix of the control points, $(D_\eta \otimes I_m)\,\bm{c}$ stacks $\bm{c}_i - \bm{c}_{i-1}$ |
    | $D_T = I_T - I_T^{(-1)}$ | $T \times T$ difference matrix of the sampled inputs, whose first row keeps $\bm{u}_0$ |
    | $\bm{e}_i \in \R^T$ | $i$th standard basis vector ($i = 1, \dots, T$) |
    | $\mathrm{blkdiag}(\cdot)$ | block diagonal matrix of its arguments |
    | $A \otimes B$ | Kronecker product: the block matrix whose $(i,j)$ block is $a_{ij} B$ |

    In a Kronecker product the first factor sets the block pattern and the second is the repeated block.
    The stacked vectors are ordered by time step, so the identity or ones vector comes first:
    $I_T \otimes Q = \mathrm{blkdiag}(Q, \dots, Q)$ and $\mathbf{1}_T \otimes \bm{w} = [\bm{w}^\T, \dots, \bm{w}^\T]^\T$.

<!-- --8<-- [end:matrix-terms] -->
