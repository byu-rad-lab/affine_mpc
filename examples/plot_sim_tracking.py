import numpy as np
from numpy.typing import NDArray
import matplotlib.pyplot as plt
from matplotlib.axes import Axes
from pathlib import Path
import tempfile

ArrayF = NDArray[np.float64]


def main():
    save_dir = Path(tempfile.gettempdir()) / "ampc_example"

    npz_path = save_dir / "log.npz"
    if not npz_path.exists():
        print(f"Error: {npz_path} not found. Run 'example_sim' first.")
        return

    data = np.load(npz_path)

    use_input_cost = bool(data["meta_opt_use_input_cost"])

    time: ArrayF = data["time"]
    states: ArrayF = data["states"]  # (N, K, n) or (N, n)
    ref_states: ArrayF = data["ref_states"]  # same shape as states
    inputs: ArrayF = data["inputs"]  # (N, K, m) or (N, nc, m) or (N, m)
    solve_times: ArrayF = data["solve_times"] * 1000.0  # (N, 2) and convert to ms

    u_min = data["meta_u_min"]
    u_max = data["meta_u_max"]

    is_2d = states.ndim == 2
    x_hist = states if is_2d else states[:, 0]
    xr_hist = ref_states if is_2d else ref_states[:, 0]
    u_hist = inputs if (inputs.ndim == 2) else inputs[:, 0]

    pos, vel = x_hist.T
    pos_ref, vel_ref = xr_hist.T
    force = u_hist.squeeze()

    # Set up axes
    ref_style = "g-."
    lim_style = "r:"

    fig, axes = plt.subplots(3, 1, sharex=True, figsize=(10, 8))
    ax_pos: Axes = axes[0]
    ax_vel: Axes = axes[1]
    ax_F: Axes = axes[2]

    ax_pos.set_ylabel("position (m)")
    ax_pos.grid(True)

    ax_vel.set_ylabel("velocity (m/s)")
    ax_vel.grid(True)

    ax_F.set_ylabel("force (N)")
    ax_F.set_xlabel("time (s)")
    ax_F.grid(True)

    # Plot tracking
    ax_pos.plot(time, pos, label="actual")
    ax_pos.plot(time, pos_ref, ref_style, label="ref")

    ax_vel.plot(time, vel)
    ax_vel.plot(time, vel_ref, ref_style)

    ax_F.step(time, force, where="post")
    if use_input_cost:
        ref_inputs = data["ref_inputs"]  # same shape as inputs
        ur_hist = ref_inputs if (ref_inputs.ndim == 2) else ref_inputs[:, 0, :]
        force_ref = ur_hist.squeeze()
        ax_F.plot(time, force_ref, ref_style)

    # Plot limits
    ax_F.plot(time[[0, -1]], np.tile(u_min, 2), lim_style, label="limits")
    ax_F.plot(time[[0, -1]], np.tile(u_max, 2), lim_style)

    # Plot Solve Times
    fig2, ax_st = plt.subplots(1, 1, figsize=(10, 4))
    usr_mean, osqp_mean = np.mean(solve_times, axis=0)
    ax_st.plot(time, solve_times[:, 0], label=f"user (avg: {usr_mean:.3f}ms)")
    ax_st.plot(time, solve_times[:, 1], label=f"OSQP (avg: {osqp_mean:.3f}ms)")
    ax_st.set_ylabel("solve time (ms)")
    ax_st.set_xlabel("sim time (s)")
    ax_st.grid(True)

    # Finalize plots and show
    ax_pos.legend()
    ax_F.legend()
    ax_st.legend()

    fig.tight_layout()
    fig2.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
