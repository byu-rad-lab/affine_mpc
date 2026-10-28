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
    affine_mpc::MPCLogger logger{mpc, "/tmp/ampc_example", dt, 1, false,
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

- `mpc`: the MPC object bound to the logger for its lifetime
- `save_dir`: output directory
- `ts`: simulation time step used to align predicted trajectories
- `prediction_stride`: downsampling factor for predicted trajectories
- `log_control_points`: whether to log parameterized control points instead of dense evaluated inputs
- `save_name`: base file name for the output artifact(s)
- `mode`: one of `RawRecoverable`, `Npy`, `NpzUncompressed`, or `NpzCompressed`

In Python, the constructor can be called with keyword arguments, which is often clearer in scripts.

## Output Files

The logger always stages raw payloads under `<save_name>_raw/` while logging.

Final outputs by mode:

- `RawRecoverable`: keep `<save_name>_raw/` with `*.bin`, `*.npyh`, `data_info.yaml`, and `params.yaml`
- `Npy`: write `<save_name>_npy/*.npy` plus `params.yaml`
- `NpzUncompressed`: write `<save_name>.npz` plus `params.yaml` using stored ZIP entries
- `NpzCompressed`: write `<save_name>.npz` plus `params.yaml` using deflate compression when zlib is available

`RawRecoverable` is the safest mode for crash recovery and C++-side loading.
Its `data_info.yaml` file is written last and acts as the completion marker for a finalized raw log.

Large logs may exceed the current ZIP32 limits of the built-in NPZ writer.
In that case, the logger removes the partial archive file and writes a fallback directory `<save_name>_npy/` containing standalone NPY files, plus the usual metadata YAML file.
The fallback arrays can be loaded directly in Python:

```python
import numpy as np

states = np.load("my_log_npy/states.npy")
inputs = np.load("my_log_npy/inputs.npy")
t = np.load("my_log_npy/time.npy")
```

If needed, these arrays can be repackaged into a custom archive from Python afterward.

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
- `ref_inputs`: same shape as `inputs`, with an input cost: the reference control points (`InputCostMode::ControlPoints` with `log_control_points`), the reference control points evaluated at the logged steps (`ControlPoints`), or the sampled reference at the logged steps (`Trajectory`); not logged for `Trajectory` with `log_control_points`, since that reference has no control point form
- `mpc_solve_time`: `(N,)` duration of each whole `solve()` call (`SolveInfo.mpc_solve_time`)
- `qp_solve_time`: `(N,)` time OSQP spent solving (`SolveInfo.qp_solve_time`, zero if OSQP is built without profiling)
- `prepare_time`: `(N,)` total time of the `prepare()` calls before each solve (`SolveInfo.prepare_time`, zero when `prepare()` is not used)
- `loop_time`: `(N,)` optional user-measured time passed to `logStep()`, e.g. of the whole control loop on hardware (`-1` when not provided)
- `meta_*`: metadata arrays copied into the NPZ container

The logged `states` start at the current state `x0` passed to `logStep()` (prediction step $k = 0$), followed by the predicted states.

The timing arrays nest: `qp_solve_time` is part of `mpc_solve_time`, which is the MPC's latency after the measurement; `prepare_time` is separate work done before the measurement, so `prepare_time + mpc_solve_time` is the MPC's work per step; and `loop_time`, if you pass it, covers your whole loop, including model computation and discretization.
This differs from `getPredictedStateTrajectory()`, which starts at the first predicted state $x_1$ and does not include `x0`.
`meta_t_pred` holds the prediction times of the logged samples relative to the time of each step.

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
- B-spline degree and knot vector, or for a custom basis (`Parameterization.fromBasis()`) the basis, flattened row by row (reshape `meta_basis` to `(horizon_steps, num_control_points)`)
- enabled options (`opt_*`), with the input cost mode as `opt_input_cost` (`off`, `control_points`, or `trajectory`), the bound modes as `opt_input_bounds` (`control_points`, `exact`, or `refined`, with `opt_refined_bound_interval`) and `opt_state_bounds` (`off`, `hard`, or `soft`); `opt_saturate_input_trajectory` (1 for `exact`), `opt_saturate_states` (1 unless `off`), and `opt_use_input_cost` (1 unless `off`) are kept for readers of earlier logs
- limits
- weights
- prediction stride
- logging mode

You can also append custom metadata:

```python
logger.addMetadata("example_name", "mass_spring_damper")
```

or in C++:

```cpp
logger.addMetadata("example_name", std::string{"mass_spring_damper"});
```

## Python Loading Example

```python
import numpy as np

data = np.load("/tmp/ampc_example/log.npz")
time = data["time"]
states = data["states"]
```

The helper script `examples/plot_sim.py` loads the `.npz` file written by the default `NpzCompressed` mode and the top-level `params.yaml` file.

## Striding Behavior

`prediction_stride` controls how much of each predicted horizon is saved.

- `0`: log only the current step
- `1`: log every prediction step
- `K`: log every `K`-th prediction step, always including the terminal state

This lets you trade off detail versus file size.

## Control-Point Logging Mode

When `log_control_points` is `true`, the logger stores parameterized control points instead of the dense evaluated input trajectory. This can be useful for debugging the parameterization itself.

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
- Use the plotting scripts in `examples/` as reference consumers of the logged format
