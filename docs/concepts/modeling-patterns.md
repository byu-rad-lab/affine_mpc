# Modeling Patterns

`affine_mpc` takes an affine model $\mathbf{x}_{k+1} = A \mathbf{x}_k + B \mathbf{u}_k + \mathbf{w}$ and absolute inputs, states, limits, and references.
Several common control problems fit this form without any special library support.
This page collects those patterns.

## Operating Points

To control a nonlinear plant $\mathbf{x}_{k+1} = f(\mathbf{x}_k, \mathbf{u}_k)$ near an operating point $(\mathbf{x}_{\mathrm{p}}, \mathbf{u}_{\mathrm{p}})$, approximate it there with its first-order (affine) Taylor expansion:

$$
\begin{align}
\mathbf{x}_{k+1} &\approx f(\mathbf{x}_{\mathrm{p}}, \mathbf{u}_{\mathrm{p}}) + A (\mathbf{x}_k - \mathbf{x}_{\mathrm{p}}) + B (\mathbf{u}_k - \mathbf{u}_{\mathrm{p}}) \\
&= A \mathbf{x}_k + B \mathbf{u}_k + \underbrace{f(\mathbf{x}_{\mathrm{p}}, \mathbf{u}_{\mathrm{p}}) - A \mathbf{x}_{\mathrm{p}} - B \mathbf{u}_{\mathrm{p}}}_{\mathbf{w}},
\end{align}
$$

with $A = \partial f / \partial \mathbf{x}$ and $B = \partial f / \partial \mathbf{u}$ evaluated at $(\mathbf{x}_{\mathrm{p}}, \mathbf{u}_{\mathrm{p}})$.
All of the operating point goes into the model offset $\mathbf{w}$.
Inputs, input and state limits, and references stay in absolute coordinates, so there is nothing else to shift.
For a continuous-time model $\dot{\mathbf{x}} = f_{\mathrm{c}}(\mathbf{x}, \mathbf{u})$, compute $\mathbf{w}_{\mathrm{c}} = f_{\mathrm{c}}(\mathbf{x}_{\mathrm{p}}, \mathbf{u}_{\mathrm{p}}) - A_{\mathrm{c}} \mathbf{x}_{\mathrm{p}} - B_{\mathrm{c}} \mathbf{u}_{\mathrm{p}}$ the same way and pass it to `setModelContinuous2Discrete()`.

To update the approximation about a new operating point (e.g. the current state and the last applied input), set the new model before each solve.
This changes $A$ and $B$, so the QP matrices are rebuilt and refactored at the next solve (see [Runtime Updates](runtime-updates.md)).
[Model Update Strategies](model-updates.md) compares updating the model about the measured state after the measurement with updating it about a predicted state before the measurement, which lets `prepare()` do the rebuild ahead of time.

B-splines are a partition of unity, so a constant trim input $\mathbf{u}_{\mathrm{p}}$ is represented exactly by setting every control point to $\mathbf{u}_{\mathrm{p}}$, e.g. with `setReferenceInput(u_p)`.

## Offset-Free Tracking

A constant model error or unmeasured disturbance causes steady-state tracking error.
The standard remedy is a disturbance model,

$$
\mathbf{x}_{k+1} = A \mathbf{x}_k + B \mathbf{u}_k + \mathbf{w}_{\mathrm{nom}} + B_{\mathrm{d}} \mathbf{d}_k,
$$

where an observer (your own, e.g. a Kalman filter on the state augmented with $\mathbf{d}$) estimates $\hat{\mathbf{d}}_k$ from measurements.
Before each solve, set the model offset to $\mathbf{w} = \mathbf{w}_{\mathrm{nom}} + B_{\mathrm{d}} \hat{\mathbf{d}}_k$ with `setModelOffsetDiscrete()` (or `setModelOffsetContinuous2Discrete()` for a continuous-time offset), which keeps $A$ and $B$ and so does not rebuild the QP matrices.
The MPC then predicts with the current disturbance estimate and removes the offset at steady state.
For an input disturbance, $B_{\mathrm{d}} = B$.

## Input Delay

If the plant applies each input $d$ steps after it is computed, augment the state with the inputs still in flight.
For a one-step delay, $\mathbf{x}_{k+1} = A \mathbf{x}_k + B \mathbf{u}_{k-1} + \mathbf{w}$, use the state $\mathbf{z}_k = (\mathbf{x}_k, \mathbf{u}_{k-1})$:

$$
\mathbf{z}_{k+1} =
\begin{bmatrix} A & B \\ 0 & 0 \end{bmatrix} \mathbf{z}_k
+ \begin{bmatrix} 0 \\ I \end{bmatrix} \mathbf{u}_k
+ \begin{bmatrix} \mathbf{w} \\ 0 \end{bmatrix}.
$$

Longer delays add one block of past inputs per step.
Pass the augmented state (the measured state and the inputs already sent) to `solve()`.
Give the input part of the state zero weight in `Q` (weights only need to be non-negative), and use the input limits as usual since $\mathbf{u}_k$ is still the decision.

## Deviation Form for Other Bases

The patterns above use absolute inputs, which works because every built-in parameterization can hold a constant input.
A basis that decays to zero (e.g. Laguerre functions through `Parameterization.fromBasis()`, see [Custom Bases](input-parameterization.md#custom-bases)) can not, and is better used in deviation form $\tilde{\mathbf{u}} = \mathbf{u} - \mathbf{u}_{\mathrm{ss}}$, where $\mathbf{u}_{\mathrm{ss}}$ is the input that holds the plant at the reference in steady state.
The input settles to $\mathbf{u}_{\mathrm{ss}}$ as the basis decays, so this choice lets the prediction end at the reference.
This is a change of variables you can apply yourself:

1. Use the model offset $\mathbf{w} + B \mathbf{u}_{\mathrm{ss}}$ (or $\mathbf{w}_{\mathrm{c}} + B_{\mathrm{c}} \mathbf{u}_{\mathrm{ss}}$ before discretizing).
1. Use the input limits $\mathbf{u}_{\min} - \mathbf{u}_{\mathrm{ss}}$ and $\mathbf{u}_{\max} - \mathbf{u}_{\mathrm{ss}}$, and the input reference $\mathbf{u}_{\mathrm{ref}} - \mathbf{u}_{\mathrm{ss}}$.
   With a custom basis, `setReferenceInput()` is not available; the default zero coefficient reference corresponds to $\mathbf{u}_{\mathrm{ref}} = \mathbf{u}_{\mathrm{ss}}$.
1. Keep the control point slew rate unchanged (B-splines only), and pass $\mathbf{u}_{-1} - \mathbf{u}_{\mathrm{ss}}$ to `setPreviousInput()` for the initial slew rate.
1. Keep the states, state limits, and `Q` unchanged.
1. Add $\mathbf{u}_{\mathrm{ss}}$ back to the results of `getNextInput()` and `getInputTrajectory()`.

Caveats:

- If $\mathbf{u}_{\mathrm{ss}}$ changes between solves (e.g. with the reference), redo steps 1 to 3 with the new value.
  For step 3, `solve()` stored the previous input relative to the old value, so call `setPreviousInput(u_applied - u_ss_new)`.
- The logger records $\tilde{\mathbf{u}}$, not $\mathbf{u}$.
- A reference that changes along the horizon would need a time-varying $\mathbf{u}_{\mathrm{ss},k}$ and so a time-varying model, which the library does not support yet.
