# Logging

`MPCLogger` provides binary logging for simulation and debugging workflows.
It is designed to avoid text-format overhead in high-rate loops by writing raw binary data during execution and packing it into a compressed `.npz` file at finalization.

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
    logger = ampc.MPCLogger(mpc, save_dir, ts, prediction_stride=1)

    while t < tf:
        status = mpc.solve(xk)
        if status != ampc.SolveStatus.Success:
            break

        logger.logStep(t, xk, user_solve_time)
        t += ts
    ```

=== "C++"

    ```cpp
    affine_mpc::MPCLogger logger{mpc, "/tmp/ampc_example", dt, 1};

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
- `save_name`: base file name for the `.npz` output

## Output Files

The logger writes:

- `<save_name>.npz`
- `params.yaml`

During execution it also uses temporary binary files, which are packed and deleted during finalization.

## Lifetime Model

The logger is constructed with one MPC object and assumes that object remains valid for the lifetime of the logger.
In C++, this means the MPC instance must outlive the logger.
Python bindings enforce this relationship with `py::keep_alive`.

## NPZ Arrays

Common arrays in the `.npz` file include:

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

The helper script `examples/plot_sim.py` loads both the `.npz` file and `params.yaml`.

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
- Packing into `.npz` happens at finalization time
- This design helps keep the hot loop lighter than repeated text formatting or repeated small archive writes

## Practical Recommendations

- Use a dedicated output directory per run when comparing experiments
- Call `finalize()` explicitly in long scripts so errors surface sooner
- Use `prediction_stride > 1` for long simulations if file size becomes large
- Use the plotting scripts in `examples/` as reference consumers of the logged format
