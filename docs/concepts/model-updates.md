# Model Update Strategies

`affine_mpc` solves with an affine model that you provide.
For a nonlinear plant $\dot{\mathbf{x}} = f(\mathbf{x}, \mathbf{u})$, you compute that model about an operating point $(\mathbf{x}_{\mathrm{p}}, \mathbf{u}_{\mathrm{p}})$ (see [Operating Points](modeling-patterns.md#operating-points)):

$$
\dot{\mathbf{x}} \approx A_{\mathrm{c}} \mathbf{x} + B_{\mathrm{c}} \mathbf{u} + \mathbf{w}_{\mathrm{c}},
\qquad
\mathbf{w}_{\mathrm{c}} = f(\mathbf{x}_{\mathrm{p}}, \mathbf{u}_{\mathrm{p}}) - A_{\mathrm{c}} \mathbf{x}_{\mathrm{p}} - B_{\mathrm{c}} \mathbf{u}_{\mathrm{p}},
$$

with $A_{\mathrm{c}} = \partial f / \partial \mathbf{x}$ and $B_{\mathrm{c}} = \partial f / \partial \mathbf{u}$ at the operating point.
This is often called linearization; because the result keeps the offset $\mathbf{w}_{\mathrm{c}}$, it is an affine approximation.

As the plant moves, you update the model about a new operating point.
This page shows several ways to schedule that update relative to the state measurement, and what work the library does at each point in the loop.
How accurate each approximation is depends on your plant, its model, and the sample period; the [quadrotor example](../examples/quadrotor-model-updates.md) shows the results for one system.

## What a Model Update Costs

- Changing $A$ or $B$ (`setModelDiscrete()` or `setModelContinuous2Discrete()`) makes the next `solve()` or `prepare()` rebuild the QP matrices and refactor OSQP.
  `setModelContinuous2Discrete()` also computes a matrix exponential inside the setter.
- Changing only the offset $\mathbf{w}$ (`setModelOffsetDiscrete()` or `setModelOffsetContinuous2Discrete()`) updates QP vectors only: no matrix exponential, no rebuild, and no refactorization.
- `prepare()` does any pending rebuild and refactorization before the measurement, so `solve()` does not have to.
- `solve(x_meas)` always uses the measured state as the initial state, wherever the model was computed.

See [Runtime Updates](runtime-updates.md) for the full list of what each setter changes.

## Measured: After the Measurement

Compute the model about the measured state and the last applied input, $(\mathbf{x}_k, \mathbf{u}_{k-1})$:

1. measure $\mathbf{x}_k$
1. compute $A_{\mathrm{c}}$, $B_{\mathrm{c}}$, $\mathbf{w}_{\mathrm{c}}$ at $(\mathbf{x}_k, \mathbf{u}_{k-1})$ and call `setModelContinuous2Discrete()`
1. `solve(x_k)` and apply the input

This is the simplest loop, and the model is computed about the state the prediction starts from.
But all of the work happens after the measurement: computing the model, the matrix exponential, and, inside `solve()`, the QP rebuild and refactorization.
All of it adds to the latency, and `prepare()` can not help, because the operating point is not known before the measurement.

## Predicted: Before the Measurement

Compute the model about a prediction of the next state, right after applying the input, and prepare the QP before the next measurement:

1. after applying $\mathbf{u}_k$, predict $\hat{\mathbf{x}}_{k+1}$ by integrating your model one step from $\mathbf{x}_k$ with $\mathbf{u}_k$ (or take the first predicted state of the last solution)
1. compute the model at $(\hat{\mathbf{x}}_{k+1}, \mathbf{u}_k)$, call `setModelContinuous2Discrete()`, then `prepare()`
1. measure $\mathbf{x}_{k+1}$, `solve(x_{k+1})`, and apply the input

This is the preparation and feedback split of the real-time iteration (RTI) scheme ([Diehl, Bock, and Schlöder, 2005](https://doi.org/10.1137/S0363012902400713)).
After the measurement, `solve()` only updates the QP vectors that depend on the initial state and runs the solver iterations.
The model is computed about the prediction rather than the measured state; how much that matters depends on how close the prediction is to the measurement.

If you take the prediction from the last solution, call `getPredictedStateTrajectory()` before `prepare()`: the condensed formulation computes the prediction from the current model.

## Hybrid: Predicted Model, Offset Recomputed After the Measurement

Do the predicted update, then recompute only the offset at the measured state:

1. as in the predicted update, compute the model at $(\hat{\mathbf{x}}_{k+1}, \mathbf{u}_k)$, call `setModelContinuous2Discrete()`, then `prepare()`
1. measure $\mathbf{x}_{k+1}$ and recompute the offset at the measured state with the prepared $A_{\mathrm{c}}$ and $B_{\mathrm{c}}$, $\mathbf{w}_{\mathrm{c}} = f(\mathbf{x}_{k+1}, \mathbf{u}_k) - A_{\mathrm{c}} \mathbf{x}_{k+1} - B_{\mathrm{c}} \mathbf{u}_k$
1. call `setModelOffsetContinuous2Discrete(w_c)`, `solve(x_{k+1})`, and apply the input

`setModelOffsetContinuous2Discrete()` discretizes the offset with the stored discretization ($\mathbf{w}_{\mathrm{d}} = G \mathbf{w}_{\mathrm{c}}$, one matrix-vector product), and the model offset only enters QP vectors, so `solve()` does not rebuild the QP matrices and OSQP does not refactor.
If you discretize the model yourself and set it with `setModelDiscrete()`, compute the discrete offset $\mathbf{w}_{\mathrm{d}} = f_{\mathrm{d}}(\mathbf{x}_{k+1}, \mathbf{u}_k) - A_{\mathrm{d}} \mathbf{x}_{k+1} - B_{\mathrm{d}} \mathbf{u}_k$ instead and pass it to `setModelOffsetDiscrete()`, where $f_{\mathrm{d}}$ is your one-step model.

With this offset, the model matches $f$ at the measured state, while $A_{\mathrm{c}}$ and $B_{\mathrm{c}}$ are still those computed at the prediction.
The cost is one evaluation of $f$ and an offset update after the measurement.

## Where the Work Happens

| Strategy | Work after the measurement | Work before the measurement | Model computed about |
| --- | --- | --- | --- |
| Measured | model, matrix exponential, QP rebuild and refactorization, solver | none | measured state |
| Predicted | solver (vector updates and iterations) | prediction, model, matrix exponential, `prepare()` | prediction |
| Hybrid | one evaluation of $f$, offset update, solver | as predicted | prediction ($A$, $B$); measured state ($\mathbf{w}$) |

The total work per step is about the same in all three.
Predicted and hybrid move the matrix exponential, rebuild, and refactorization out of the time between measurement and input; whether that matters depends on how that time compares with your sample period and whatever else must run within it.

## In the Quadrotor Example

[Quadrotor Model Updates](../examples/quadrotor-model-updates.md) flies a simulated quadrotor with any of these strategies; its `--compare` mode runs all three at 100 and 500 Hz.
In that example the three tracked equally well, and the measured strategy's time from measurement to input was about an order of magnitude longer than the predicted strategy's, almost all of it the QP rebuild and refactorization inside `solve()`; predicted and hybrid did the same work in `prepare()` before the measurement.
That ratio grows with the number of horizon steps, since the rebuild does, so it is specific to this horizon.
These results are for one vehicle and model; the example page lists what was run and the full results.

## Code

The loop for the predicted update, with the extra lines for hybrid marked.
`affine_model(x, u)` stands for your affine approximation, returning $A_{\mathrm{c}}$, $B_{\mathrm{c}}$, and $\mathbf{w}_{\mathrm{c}}$; `predict(x, u)` integrates your model over one sample period.

=== "Python"

    ```python
    from time import perf_counter

    Ac, Bc, wc = affine_model(x_meas, u_prev)
    mpc.setModelContinuous2Discrete(Ac, Bc, wc, dt)
    while running:
        mpc.setReferenceStateTrajectory(x_ref_horizon)  # anything known in advance
        mpc.prepare()  # QP rebuild and refactorization, before the measurement

        x_meas = measure()
        start = perf_counter()
        # hybrid only: offset at the measured state (vector update only)
        wc = f(x_meas, u_prev) - Ac @ x_meas - Bc @ u_prev
        mpc.setModelOffsetContinuous2Discrete(wc)
        status = mpc.solve(x_meas)
        u = mpc.getNextInput()
        apply(u)
        latency = perf_counter() - start  # your measurement-to-input time

        info = mpc.getSolveInfo()
        # info.mpc_solve_time: solve() only; info.prepare_time: prepare() above

        # after applying u, before the next measurement
        x_pred = predict(x_meas, u)
        Ac, Bc, wc = affine_model(x_pred, u)
        mpc.setModelContinuous2Discrete(Ac, Bc, wc, dt)  # matrix exponential here
        u_prev = u
    ```

=== "C++"

    ```cpp
    affine_model(x_meas, u_prev, Ac, Bc, wc);
    mpc.setModelContinuous2Discrete(Ac, Bc, wc, dt);
    while (running) {
      mpc.setReferenceStateTrajectory(x_ref_horizon); // anything known in advance
      (void)mpc.prepare(); // QP rebuild and refactorization, before measuring

      measure(x_meas);
      const auto start{std::chrono::steady_clock::now()};
      // hybrid only: offset at the measured state (vector update only)
      wc.noalias() = f(x_meas, u_prev) - Ac * x_meas - Bc * u_prev;
      mpc.setModelOffsetContinuous2Discrete(wc);
      const affine_mpc::SolveStatus status{mpc.solve(x_meas)};
      mpc.getNextInput(u);
      apply(u);
      const std::chrono::duration<double> latency{
          std::chrono::steady_clock::now() - start};

      const affine_mpc::SolveInfo info{mpc.getSolveInfo()};
      // info.mpc_solve_time: solve() only; info.prepare_time: prepare() above

      // after applying u, before the next measurement
      predict(x_meas, u, x_pred);
      affine_model(x_pred, u, Ac, Bc, wc);
      mpc.setModelContinuous2Discrete(Ac, Bc, wc, dt); // matrix exponential here
      u_prev = u;
    }
    ```

For the measured update, drop `prepare()`, the prediction, and the offset lines, and call `affine_model(x_meas, u_prev)` and `setModelContinuous2Discrete()` after the measurement, inside the timed section.

Which timings to read:

- `SolveInfo.mpc_solve_time` is the duration of `solve()`, including any QP rebuild and refactorization that was still pending; with predicted or hybrid it is only the vector updates and the solver.
- `SolveInfo.prepare_time` is the duration of the `prepare()` calls since the previous solve.
- `setModelContinuous2Discrete()` computes the matrix exponential inside the setter, so its cost is in neither of these; time it yourself, or time the whole loop (e.g. as the `loop_time` passed to `MPCLogger.logStep()`).
- `setModelOffsetDiscrete()` and `setModelOffsetContinuous2Discrete()` are the cheap path when only $\mathbf{w}$ changes: no matrix exponential, no QP rebuild, and no refactorization.

## Time-Varying Models

Computing a model about each step of a predicted trajectory gives a time-varying model $(A_k, B_k, \mathbf{w}_k)$.
The library supports only time-invariant models for now.
