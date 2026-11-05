# Modeling Patterns

`affine_mpc` takes an affine model $\bm{x}_{k+1} = A \bm{x}_k + B \bm{u}_k + \bm{w}$ and absolute inputs, states, limits, and references.
Several common control problems fit this form without any special library support.
This page collects those patterns.

## Operating Points

To control a nonlinear plant $\bm{x}_{k+1} = f(\bm{x}_k, \bm{u}_k)$ near an operating point $(\bm{x}_{\mathrm{p}}, \bm{u}_{\mathrm{p}})$, approximate it there with its first-order (affine) Taylor expansion:

$$
\begin{align}
\bm{x}_{k+1} &\approx f(\bm{x}_{\mathrm{p}}, \bm{u}_{\mathrm{p}}) + A (\bm{x}_k - \bm{x}_{\mathrm{p}}) + B (\bm{u}_k - \bm{u}_{\mathrm{p}}) \\
&= A \bm{x}_k + B \bm{u}_k + \underbrace{f(\bm{x}_{\mathrm{p}}, \bm{u}_{\mathrm{p}}) - A \bm{x}_{\mathrm{p}} - B \bm{u}_{\mathrm{p}}}_{\bm{w}},
\end{align}
$$

with $A = \partial f / \partial \bm{x}$ and $B = \partial f / \partial \bm{u}$ evaluated at $(\bm{x}_{\mathrm{p}}, \bm{u}_{\mathrm{p}})$.
All of the operating point goes into the model offset $\bm{w}$.
Inputs, input and state limits, and references stay in absolute coordinates, so there is nothing else to shift.
For a continuous-time model $\dot{\bm{x}} = f_{\mathrm{c}}(\bm{x}, \bm{u})$, compute $\bm{w}_{\mathrm{c}} = f_{\mathrm{c}}(\bm{x}_{\mathrm{p}}, \bm{u}_{\mathrm{p}}) - A_{\mathrm{c}} \bm{x}_{\mathrm{p}} - B_{\mathrm{c}} \bm{u}_{\mathrm{p}}$ the same way and pass it to `setModelContinuous2Discrete()`.

To update the approximation about a new operating point (e.g. the current state and the last applied input), set the new model before each solve.
This changes $A$ and $B$, so the QP matrices are rebuilt and refactored at the next solve (see [Runtime Updates](runtime-updates.md)).
[Model Update Strategies](model-updates.md) compares updating the model about the measured state after the measurement with updating it about a predicted state before the measurement, which lets `prepare()` do the rebuild ahead of time.

B-splines are a partition of unity, so a constant trim input $\bm{u}_{\mathrm{p}}$ is represented exactly by setting every control point to $\bm{u}_{\mathrm{p}}$, e.g. with `setReferenceInput(u_p)` (which requires an `input_cost` other than `Off`).

## Offset-Free Tracking

A constant model error or unmeasured disturbance causes steady-state tracking error.
The standard remedy is a disturbance model,

$$
\bm{x}_{k+1} = A \bm{x}_k + B \bm{u}_k + \bm{w}_{\mathrm{nom}} + B_{\mathrm{d}} \bm{d}_k,
$$

where an observer (your own, e.g. a Kalman filter on the state augmented with $\bm{d}$) estimates $\hat{\bm{d}}_k$ from measurements.
Before each solve, set the model offset to $\bm{w} = \bm{w}_{\mathrm{nom}} + B_{\mathrm{d}} \hat{\bm{d}}_k$ with `setModelOffsetDiscrete()` (or `setModelOffsetContinuous2Discrete()` for a continuous-time offset), which keeps $A$ and $B$ and so does not rebuild the QP matrices.
The MPC then predicts with the current disturbance estimate and removes the offset at steady state.
For an input disturbance, $B_{\mathrm{d}} = B$.

## Input Delay

If the plant applies each input $d$ steps after it is computed, augment the state with the inputs still in flight.
For a one-step delay, $\bm{x}_{k+1} = A \bm{x}_k + B \bm{u}_{k-1} + \bm{w}$, use the state $\bm{z}_k = (\bm{x}_k, \bm{u}_{k-1})$:

$$
\bm{z}_{k+1} =
\begin{bmatrix} A & B \\ 0 & 0 \end{bmatrix} \bm{z}_k
+ \begin{bmatrix} 0 \\ I \end{bmatrix} \bm{u}_k
+ \begin{bmatrix} \bm{w} \\ 0 \end{bmatrix}.
$$

Longer delays add one block of past inputs per step.
Pass the augmented state (the measured state and the inputs already sent) to `solve()`.
Give the input part of the state zero weight in `Q` (weights only need to be non-negative), and use the input limits as usual since $\bm{u}_k$ is still the decision.

## Deviation Form for Other Bases

The patterns above use absolute inputs, which works because every built-in parameterization can hold a constant input.
A basis that decays to zero (e.g. Laguerre functions through `Parameterization.fromBasis()`, see [Custom Bases](input-parameterization.md#custom-bases)) can not, and is better used in deviation form $\tilde{\bm{u}} = \bm{u} - \bm{u}_{\mathrm{ss}}$, where $\bm{u}_{\mathrm{ss}}$ is the input that holds the plant at the reference in steady state.
The input settles to $\bm{u}_{\mathrm{ss}}$ as the basis decays, so this choice lets the prediction end at the reference.
This is a change of variables you can apply yourself:

1. Use the model offset $\bm{w} + B \bm{u}_{\mathrm{ss}}$ (or $\bm{w}_{\mathrm{c}} + B_{\mathrm{c}} \bm{u}_{\mathrm{ss}}$ before discretizing).
1. Use the input limits $\bm{u}_{\min} - \bm{u}_{\mathrm{ss}}$ and $\bm{u}_{\max} - \bm{u}_{\mathrm{ss}}$, and the input reference $\bm{u}_{\mathrm{ref}} - \bm{u}_{\mathrm{ss}}$.
   With general constraints, use the bounds $\bm{h}_{\min} - H_u \bm{u}_{\mathrm{ss}}$ and $\bm{h}_{\max} - H_u \bm{u}_{\mathrm{ss}}$.
   With a custom basis and `InputCostMode::ControlPoints`, `setReferenceInput()` is not available; the default zero coefficient reference corresponds to $\bm{u}_{\mathrm{ref}} = \bm{u}_{\mathrm{ss}}$.
1. Keep the slew rate unchanged, and pass $\bm{u}_{-1} - \bm{u}_{\mathrm{ss}}$ to `setPreviousInput()` for the initial slew rate and the input rate cost.
1. Keep the states, state limits, and `Q` unchanged.
1. Add $\bm{u}_{\mathrm{ss}}$ back to the results of `getNextInput()` and `getInputTrajectory()`.

Caveats:

- If $\bm{u}_{\mathrm{ss}}$ changes between solves (e.g. with the reference), redo steps 1 to 3 with the new value.
  For step 3, `solve()` stored the previous input relative to the old value, so call `setPreviousInput(u_applied - u_ss_new)`.
- The logger records $\tilde{\bm{u}}$, not $\bm{u}$.
- A reference that changes along the horizon would need a time-varying $\bm{u}_{\mathrm{ss},k}$ and so a time-varying model, which the library does not support yet.
