# Quadrotor Model Updates

`examples/quadrotor_model_updates.py` controls a simulated quadrotor, a nonlinear plant, by recomputing its affine model as it flies.
A single run uses one of the strategies described in [Model Update Strategies](../concepts/model-updates.md) and logs the flight with `MPCLogger`; `--compare` runs them all and reports their tracking error and timing.

## What the Example Demonstrates

- Computing an affine model $(A_{\mathrm{c}}, B_{\mathrm{c}}, \mathbf{w}_{\mathrm{c}})$ of nonlinear dynamics from analytic Jacobians (checked against central differences at startup)
- Updating the model every step with `setModelContinuous2Discrete()`
- Moving the QP rebuild and refactorization before the measurement with `prepare()`
- Updating only the model offset with `setModelOffsetContinuous2Discrete()`
- Logging each run with `MPCLogger`, passing the measured latency as `loop_time` and the run's settings as metadata
- Initializing the solver with a model at a random state, so the sparsity pattern covers every later model (see [Model Entries](../concepts/runtime-updates.md#model-entries-initial-nonzeros-or-full))

## Run the Example

The example is Python-only and needs NumPy.
`--plot` opens a window with trajectory, position error, and timing tabs, which needs the plotting dependencies of the `[examples]` extra (matplotlib, abracatabra, and a Qt binding, see [Python Getting Started](../getting-started/python.md#visualization-dependencies)); with tqdm installed, a progress bar shows the time remaining.

```sh
# one run: hybrid at 100 Hz (choose with --strategy and --rate)
python examples/quadrotor_model_updates.py --plot

# every strategy at 100 and 500 Hz, and the comparison tables
# (choose a subset with --strategies and --rates)
python examples/quadrotor_model_updates.py --compare

# the tables and plot again from the saved logs, without simulating
python examples/quadrotor_model_updates.py --compare --load --plot
```

Each run writes its own log to `ampc_example/quadrotor/<rate>hz_<strategy>/` in the system temp directory (`log.npz` and `params.yaml`, see [Logging](../logging.md)), with a `_delay<scale>` suffix for delayed runs; `--log-dir` changes the location.
`--load` reads those logs instead of simulating; it stops if a requested run has no log, or if the logs were made with different durations or seeds.
`--quick` flies 3 s instead of 12 s; see `--help` for the other options.
If you installed the package with `pip`, `affine-mpc-example-quadrotor` runs the same script with the same options, e.g. `affine-mpc-example-quadrotor --compare --plot`.

## What It Simulates

- **Strategies:** each run updates the model one way:
    - **measured:** about the measured state, after the measurement
    - **predicted:** about a one-step prediction, before the measurement, followed by `prepare()`
    - **hybrid:** as predicted, plus the offset recomputed at the measured state after the measurement
    - **hover:** one model about hover that is never updated, as a baseline (not a strategy you would use)
- **Plant:** 12 states (position, velocity, ZYX Euler angles, body rates) and four rotor thrusts in a "+" configuration; 1 kg, inertia diag(0.0082, 0.0082, 0.0149) kg m², 0.2 m arms, linear drag, rotor thrusts limited to [0, 5] N (thrust-to-weight about 2).
  Euler angles keep the state a 12-vector with box weights and stay far from their singularity at 90° pitch (the task tilts up to about 30°).
  The plant is integrated with RK4 at 0.5 ms or finer, and measurements have small Gaussian noise (fixed seed).
- **Task:** 12 s, two laps of a figure-eight (±2 m by ±1 m, ±0.5 m in height, 6 s per lap) with yaw swinging ±45°, with a reference from differential flatness.
  A body-rate kick of (4, −3, 0) rad/s at 3 s models a collision that no prediction can anticipate; a constant wind force of (1.5, −1, 0) N starts at 6 s and is not in the model.
- **MPC:** `MPC` with the condensed formulation, a 0.8 s horizon (80 steps at 100 Hz, 400 at 500 Hz), and a degree-2 B-spline with 8 control points per input and geometric knots (`knot_ratio=1.5`), which puts short knot intervals at the start of the horizon.
  The horizon is fixed in seconds so the controller previews the same time and uses the same knot times at both rates; the number of steps, and with it the cost of rebuilding the QP after a model change, grows with the rate.
  The weights scale the errors so that 1 cm of position, 3 cm/s of velocity, 1° of attitude, and 0.3 N of thrust cost about the same, with a small weight on body rates (`Q = diag(40, 40, 40, 2, 2, 2, 1, 1, 4, 0.01, 0.01, 0.01)`, `Q_f = Q`, `R = 0.05 I` on the rotor thrusts around the flatness reference).

## What It Measures

- **Position error** in three parts of the run: before the kick (first quarter), after the kick (second quarter), and in wind (second half), computed from the logged states and the reference at each logged time.
- **Latency:** the wall time from the measurement to the input, timed with `time.perf_counter()` around the work after the measurement and logged as `loop_time`.
  For measured that is computing the model, `setModelContinuous2Discrete()`, and `solve()`; for predicted, `solve()`; for hybrid, computing the offset, `setModelOffsetContinuous2Discrete()`, and `solve()`.
- **`mpc_solve_time` and `prepare_time`:** logged from `getSolveInfo()`, the time inside `solve()` and inside `prepare()` before it.
- **Delayed runs** (`--delay-scale`): each new input is applied only after the given multiple of its measured latency (the previous input stays applied until then, at most one sample).
  A single run is then delayed itself; with `--compare --delay-scale 1`, every run is repeated with this delay and the change in tracking is reported, to check whether the latency itself affects tracking.

## Results

One run of `python examples/quadrotor_model_updates.py --compare --delay-scale 1` (12 s, 100 and 500 Hz, seed 0) with `affine_mpc` built from source at `23ba500` (after v0.3.0), NumPy 2.5.3, and Python 3.14.8 on Linux with an AMD Ryzen 9 5900.
The tracking errors are deterministic for a given seed.
The timings vary between machines and between runs (repeated runs on this machine moved the medians by up to about 10%) and include Python overhead, so compare ratios between strategies rather than absolute values.

Tracking error:

|   Rate | Strategy  | RMS before kick (mm) | Max after kick (mm) | RMS in wind (mm) |
| -----: | --------- | -------------------: | ------------------: | ---------------: |
| 100 Hz | measured  |                 0.57 |               11.23 |            85.17 |
| 100 Hz | predicted |                 0.57 |               11.20 |            85.17 |
| 100 Hz | hybrid    |                 0.57 |               11.20 |            85.17 |
| 100 Hz | hover     |                87.17 |              151.69 |           123.46 |
| 500 Hz | measured  |                 0.61 |                7.18 |            74.90 |
| 500 Hz | predicted |                 0.61 |                7.21 |            74.90 |
| 500 Hz | hybrid    |                 0.61 |                7.19 |            74.90 |
| 500 Hz | hover     |                78.25 |              127.13 |           109.09 |

Timing, medians per step from the runs without delay:

|   Rate | Strategy  | Latency (ms) | mpc_solve_time (ms) | prepare_time (ms) |
| -----: | --------- | -----------: | ------------------: | ----------------: |
| 100 Hz | measured  |        0.303 |               0.244 |             0.000 |
| 100 Hz | predicted |        0.034 |               0.032 |             0.183 |
| 100 Hz | hybrid    |        0.059 |               0.036 |             0.208 |
| 100 Hz | hover     |        0.036 |               0.033 |             0.000 |
| 500 Hz | measured  |        1.054 |               0.996 |             0.000 |
| 500 Hz | predicted |        0.057 |               0.054 |             0.830 |
| 500 Hz | hybrid    |        0.084 |               0.061 |             0.941 |
| 500 Hz | hover     |        0.063 |               0.060 |             0.000 |

## Observations

These hold for this run; they show where the library's work goes in each loop, not a general ranking of the strategies.

1. Measured, predicted, and hybrid tracked equally well, within 0.03 mm of each other in every column.
   At these rates the prediction was close to the measurement, so computing the model about the measured or the predicted state made no measurable difference, even right after the kick.
   The hover model, which is never updated, had more than 100 times their error before the kick.
   Most of the remaining error is from the wind, which is not in the model (see [Offset-Free Tracking](../concepts/modeling-patterns.md#offset-free-tracking) for one way to handle constant disturbances).
1. The measured strategy's latency was about 9 times the predicted strategy's at 100 Hz and 18 times at 500 Hz.
   Almost all of it is `mpc_solve_time`, i.e. the QP rebuild and refactorization inside `solve()`; computing the model and calling `setModelContinuous2Discrete()` took the remaining 0.06 ms or so.
   The rebuild grows with the number of horizon steps (80 at 100 Hz, 400 at 500 Hz), so at 500 Hz the measured strategy spent about half of the 2 ms sample period between measurement and input.
1. Predicted and hybrid spent a comparable time in `prepare()` before the measurement, so the total work per step was about the same as for measured; only its place in the loop changed.
1. Hybrid's `mpc_solve_time` stayed close to predicted's and the hover baseline's, far below measured's: the offset-only setter caused no rebuild and no refactorization.
   Its extra latency over predicted, computing the offset in Python, was 0.02 to 0.03 ms.
1. Applying each input only after its measured latency changed the errors by less than 0.01 mm, and 10 times that latency (`--compare --delay-scale 10`, which reaches the one-sample cap for measured at 500 Hz) by at most 0.01 mm.
   For this vehicle at these rates, the latency mattered for the time budget (what else must run within a sample period, or a slower computer where the measured strategy no longer fits), not for tracking accuracy.
1. No solve failed, and the QP matrices stayed within the initialized sparsity pattern.
