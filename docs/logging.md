# Logging

`MPCLogger` provides binary logging for simulation and debugging workflows.
It is designed to avoid text-format overhead in high-rate loops by writing raw binary payloads during execution and finalizing them as recoverable raw files, standalone `.npy` files, or `.npz` archives.

The logger is available from both the C++ and Python interfaces.

Each logger instance is associated with a single MPC object at construction time and reads trajectories and metadata from that stored MPC reference during `logStep()`.

## Purpose

The logger is useful for:

- simulation debugging
- post-run plotting in Python
- recording state, input, and solve-time histories
- exporting metadata alongside the data itself

## Basic Usage

=== "Python"

    ```python
    logger = ampc.MPCLogger(
        mpc,
        save_dir,
        ts,
        prediction_stride=1,
        mode=ampc.MPCLogger.Mode.NpzCompressed,
    )

    while t < tf:
        status = mpc.solve(xk)
        if status != ampc.SolveStatus.Success:
            # handle how you wish
            break

        logger.logStep(t, xk)
        t += ts
    ```

=== "C++"

    ```cpp
    affine_mpc::MPCLogger logger{&mpc, "/tmp/ampc_example", dt, 1, false,
                                 "log",
                                 affine_mpc::MPCLogger::Mode::NpzCompressed};

    affine_mpc::SolveStatus status;
    while (t < tf) {
      status = mpc.solve(xk);
      if (status != affine_mpc::SolveStatus::Success) {
        // handle how you wish
        break;
      }

      logger.logStep(t, xk);
      t += dt;
    }
    ```

Call `finalize()` explicitly when logging is done.
If `finalize()` is not called manually, the destructor attempts to finalize automatically, but a destructor can not throw, so any error (for example a full disk or an unwritable directory) is only printed to `stderr` and is easy to miss.
An explicit `finalize()` throws `std::runtime_error` (`RuntimeError` in Python) instead.

## Main Constructor Arguments

- `mpc`: the MPC object bound to the logger for its lifetime (a pointer in C++)
- `save_dir`: output directory
- `ts`: simulation time step used to align predicted trajectories
- `prediction_stride`: downsampling factor for predicted trajectories
- `log_control_points`: whether to log parameterized control points instead of dense evaluated inputs
- `save_name`: base file name for the output artifact(s)
- `mode`: one of `RawRecoverable`, `Npy`, `NpzUncompressed`, or `NpzCompressed`

In Python, the constructor can be called with keyword arguments, which is often clearer in scripts.

## Output Files

The logger always stages raw payloads under `<save_name>_raw/` while logging.
At construction it also writes `params.yaml`, `meta.npz`, and `staging_info.yaml` (the shape of one row of each payload) there, so the payloads of a run that never reaches `finalize()`, for example after a crash, can still be loaded (see [Loading Logs in Python](#loading-logs-in-python)).

Final outputs by mode:

- `RawRecoverable`: keep `<save_name>_raw/` with `*.bin`, `*.npyh`, `data_info.yaml`, `meta.npz`, and `params.yaml`
- `Npy`: write `<save_name>_npy/` with `*.npy` and `meta.npz`, plus `<save_name>_params.yaml`
- `NpzUncompressed`: write `<save_name>.npz` plus `<save_name>_params.yaml` using stored ZIP entries
- `NpzCompressed`: write `<save_name>.npz` plus `<save_name>_params.yaml` using deflate compression when zlib is available

The metadata is stored twice: in binary, exactly and with its types (the `meta_*` arrays of the `.npz`, or `meta.npz` in the `_npy` and `_raw` directories), and in the YAML file for reading.
Since the YAML file is named after `save_name`, several logs can share a `save_dir`.

`RawRecoverable` is the safest mode for crash recovery and C++-side loading.
Its `data_info.yaml` file is written last and acts as the completion marker for a finalized raw log; finalizing also removes `staging_info.yaml`, since the `.npyh` headers then hold the full shapes.

Large logs may exceed the current ZIP32 limits of the built-in NPZ writer.
In that case, the logger removes the partial archive file and writes a fallback directory `<save_name>_npy/` containing standalone NPY files, with `meta.npz`, plus the usual metadata YAML file.
`load_log()` reads the fallback directory like any other log.

## Lifetime Model

The logger is constructed with one MPC object and assumes that object remains valid for the lifetime of the logger.
In C++, this means the MPC instance must outlive the logger.
Python bindings enforce this relationship with `py::keep_alive`.

## NPZ Arrays

Common arrays in the `.npy` or `.npz` outputs include:

- `time`: `(N,)`
- `states`: `(N, K, n)` or `(N, n)` depending on stride
- `ref_states`: same shape as `states`
- `inputs`: `(N, K, m)`, `(N, nc, m)`, or `(N, m)` depending on mode
- `ref_inputs`: logged with an input cost; the reference control points, `(N, nc, m)`, for `InputCostMode::ControlPoints` with `log_control_points`, and otherwise the reference at the logged steps like `ref_states`, `(N, K, m)` or `(N, m)`: the reference control points evaluated there (`ControlPoints`) or the sampled reference (`Trajectory`, also with `log_control_points`, since a sampled reference has no control point form)
- `mpc_solve_time`: `(N,)` duration of each whole `solve()` call (`SolveInfo.mpc_solve_time`)
- `qp_solve_time`: `(N,)` time the QP solver spent solving (`SolveInfo.qp_solve_time`; for OSQP, zero if it is built without profiling)
- `prepare_time`: `(N,)` total time of the `prepare()` calls before each solve (`SolveInfo.prepare_time`, zero when `prepare()` is not used)
- `loop_time`: `(N,)` optional user-measured time passed to `logStep()`, e.g. of the whole control loop on hardware (`-1` when not provided)
- `meta_*`: the metadata, with ints, doubles, and strings as 0-d arrays (read them with `.item()`) and vectors as 1-d float64 arrays; `meta.npz` holds the same arrays for the `.npy` and raw outputs

The logged `states` start at the current state `x0` passed to `logStep()` (prediction step $k = 0$), followed by the predicted states.
This differs from `getPredictedStateTrajectory()`, which starts at the first predicted state $x_1$ and does not include `x0`.

The timing arrays nest: `qp_solve_time` is part of `mpc_solve_time`, which is the MPC's latency after the measurement; `prepare_time` is separate work done before the measurement, so `prepare_time + mpc_solve_time` is the MPC's work per step; and `loop_time`, if you pass it, covers your whole loop, including model computation and discretization.
`meta_t_pred_offsets` holds the prediction times of the logged samples relative to the time of each step.

Here:

- `N` is the number of logged time steps
- `K` is the number of strided prediction samples
- `n` is state dimension
- `m` is input dimension
- `nc` is the number of control points

## Metadata

The logger records configuration metadata such as:

- dimensions
- formulation (`formulation`: `condensed` or `sparse`)
- parameterization type (`bspline` or `custom`)
- B-spline degree and knot vector
- the sampled basis for every parameterization, flattened row by row (reshape `meta_basis` to `(horizon_steps, num_control_points)`); the input at step $k$ is `basis[k] @ control_points`, so logged control points can be evaluated without affine_mpc
- the `Options` fields (`opt_*`), with the input cost mode as `opt_input_cost` (`off`, `control_points`, or `trajectory`) and the bound modes as `opt_input_bounds` (`control_points`, `exact`, or `refined`, with `opt_refined_bound_interval`), `opt_state_bounds` (`off`, `hard`, or `soft`), and `opt_slew_bounds` (`off`, `control_points`, or `exact`)
- limits
- slew rates (`u0_slew`, and `slew_rate` per sample step)
- weights
- bounded states and soft-bound penalties (`bounded_states`, `x_soft_l1`, `x_soft_l2`)
- general and terminal constraints (`Hx`, `Hu`, `h_min`, `h_max`, `Hf`, `hf_min`, `hf_max`), with the matrices flattened row by row
- prediction stride
- logging mode

The YAML file (`<save_name>_params.yaml`, or `params.yaml` inside `<save_name>_raw/`) holds every entry, one `key: value` per line.
It is meant for reading, so doubles have 6 significant digits unless `addMetadata()` is given a precision, while the binary metadata keeps the exact values.
Strings are written as is, unless they could be read as a number or a vector, or YAML would change them; those are double-quoted with JSON escapes.

You can also append custom metadata, under any key the logger does not write itself (`addMetadata()` throws `std::invalid_argument`, `ValueError` in Python, for those, such as `ts` or `u_max`, whether or not the options log them):

```python
logger.addMetadata("example_name", "mass_spring_damper")
```

or in C++:

```cpp
logger.addMetadata("example_name", std::string{"mass_spring_damper"});
```

## Loading Logs in Python

`load_log()` reads a log written in any mode, by either interface, and returns an `MPCLog`.
It is the supported way to read logs: the data arrays keep their names and shapes, but the metadata layout can change between releases (recorded as `log_format_version`).

```python
import affine_mpc as ampc

log = ampc.load_log("/tmp/ampc_example")  # the logger's save_dir

# per-step data, as stored
log.time        # (N,)
log.states      # (N, K, n), or (N, n) with prediction_stride 0
log.inputs      # (N, K, m), (N, m), or (N, nc, m) with log_control_points

# the same data sampled at the logged prediction steps, with K = 1 for prediction_stride 0
log.t_pred      # (N, K) time of each logged prediction sample
log.x_pred      # (N, K, n) logged states, starting at x0
log.u_pred      # (N, K, m) inputs, evaluated from the control points with log_control_points
log.x, log.u    # (N, n) and (N, m) state and applied input at each step

# what the logger recorded about itself and the MPC
log.ts, log.state_dim, log.u_max
log.formulation            # ampc.Formulation.Condensed
log.options.input_cost     # ampc.InputCostMode.ControlPoints
log.R_diag                 # None without an input cost

# metadata added with addMetadata()
log.meta["example_name"]
```

Given `save_dir`, it reads `<save_name>.npz`, `<save_name>_npy/`, or `<save_name>_raw/`, whichever exists (pass `name=` for a `save_name` other than `log`); it raises `ValueError` if more than one does, and you can pass the artifact itself instead.
The array fields (`time`, `loop_time`, `mpc_solve_time`, `qp_solve_time`, `prepare_time`, `states`, `ref_states`, `inputs`, `ref_inputs`) have the names and shapes listed above, with `ref_inputs` set to `None` without an input cost.

The properties give the logged data in one form, whatever the stride and `log_control_points`:

- `t_pred` `(N, K)`, the time of each logged prediction sample, so `plot(log.t_pred[i], log.x_pred[i])` plots the prediction of step `i`
- `x_pred` and `x_ref_pred` `(N, K, n)`, the logged states and reference states; `x` and `x_ref` `(N, n)` are their first samples, the state passed to `logStep()` and its reference
- `u_pred` and `u_ref_pred` `(N, K, m)`, the inputs and reference inputs at the logged prediction steps, evaluated from the control points with `log_control_points` (the terminal step $T$ repeats the input of step $T - 1$, since the horizon has one input fewer than states); `u` and `u_ref` `(N, m)` are their first samples, the applied input and its reference
- `u_ref_pred` and `u_ref` are `None` without an input cost

Except for `u_pred` and `u_ref_pred` with `log_control_points`, which are computed, these are views of the stored arrays, not copies.

Every built-in metadata value is an attribute of the same name, read exactly from the binary metadata:

- the logger settings `ts`, `prediction_stride`, `log_control_points`, and `logging_mode` (an `MPCLogger.Mode`); `t_pred_offsets` is left out, since `t_pred` holds the same times
- the dimensions, `formulation` (a `Formulation`), and `parameterization_type` (a `Parameterization.Type`), with `spline_degree` and `knots` for a B-spline, and `basis`, reshaped to `(horizon_steps, num_control_points)`
- `options`, an `Options` rebuilt from the `opt_*` values and `bounded_states`
- the limits, weights, slew rates, soft-bound penalties, and constraints, with the constraint matrices reshaped to one row per constraint; those that depend on an option, such as `R_diag` or `Hx`, are `None` when the option leaves them out

`meta` holds only the metadata added with `addMetadata()`: keys have no `meta_` prefix, scalars are Python `int`, `float`, or `str`, and vectors are float64 arrays.
`load_log()` reads the `log_format_version` of this affine_mpc only and raises `ValueError` for others, including logs from earlier releases, which have none; read those with `numpy.load()`.
`mmap=True` memory-maps `.npy` and raw payloads instead of reading them, which helps with logs too large for memory.

A `<save_name>_raw/` directory without `data_info.yaml` is the staged payload of a logger that was never finalized.
`load_log()` reads it with `finalized` set to `False`, takes the number of steps from the payload file sizes, and drops a partly written last step.
Its metadata is what the logger knew at construction (its `meta.npz`), so entries added later with `addMetadata()` are missing.

The helper script `examples/plot_sim.py` and the quadrotor example load their logs this way.

## Striding Behavior

`prediction_stride` controls how much of each predicted horizon is saved.

- `0`: log only the current step
- `1`: log every prediction step
- `K`: log every `K`-th prediction step, always including the terminal state

This lets you trade off detail versus file size.

## Control-Point Logging Mode

When `log_control_points` is `true`, the logger stores parameterized control points instead of the dense evaluated input trajectory.
This can be useful for debugging the parameterization itself, and it keeps the whole input trajectory when `prediction_stride` is above 1.
The logged control points are `(N, nc, m)` whatever the stride, so with many control points (such as move blocking with one per step) and a coarse stride they are larger than the strided inputs.
`load_log()` evaluates them at the logged steps as `u_pred`, the same values the default mode stores.

## Performance Notes

- Logging writes binary data incrementally during the simulation loop
- Finalization repackages those payloads according to the configured logging mode
- This design helps keep the hot loop lighter than repeated text formatting or repeated small archive writes

## Practical Recommendations

- Use a dedicated output directory per run when comparing experiments
- Call `finalize()` explicitly so errors are raised instead of only printed by the destructor
- Use `prediction_stride > 1` for long simulations if file size becomes large
- Use `RawRecoverable` for the strongest crash-recovery guarantees
- Use `NpzUncompressed` when faster finalization matters more than archive size
- Use `load_log()` rather than reading the files directly; it handles the shape rules and metadata types, and keeps reading older logs when the layout changes
