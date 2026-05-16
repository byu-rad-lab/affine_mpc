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

        logger.logStep(t, xk, user_solve_time)
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

      logger.logStep(t, xk, user_solve_time);
      t += dt;
    }
    ```

If `finalize()` is not called manually, the destructor will attempt to finalize automatically.

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
- `ref_inputs`: same general shape as `inputs` when input reference data is applicable
- `solve_times`: `(N, 2)` for user-measured (1st) and OSQP-reported times (2nd)
- `meta_*`: metadata arrays copied into the NPZ container

Here:

- `N` is the number of logged time steps
- `K` is the number of strided prediction samples
- `n` is state dimension
- `m` is input dimension
- `nc` is the number of control points

## Metadata

The logger records configuration metadata such as:

- dimensions
- knot vector
- enabled options
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
- Call `finalize()` explicitly in long scripts so errors surface sooner
- Use `prediction_stride > 1` for long simulations if file size becomes large
- Use `RawRecoverable` for the strongest crash-recovery guarantees
- Use `NpzUncompressed` when faster finalization matters more than archive size
- Use the plotting scripts in `examples/` as reference consumers of the logged format
