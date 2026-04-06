import numpy as np
from numpy.typing import NDArray
from pathlib import Path
import tempfile

try:
    import abracatabra as plt
    from matplotlib.axes import Axes
except ImportError:
    print(
        "There are missing dependencies.",
        "You can install them bundled with affine_mpc:",
        '    pip install "affine_mpc[examples]"',
        "OR independently:",
        "    pip install matplotlib abracatabra pyside6",
        sep="\n",
    )
    exit(1)

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
    t_pred: ArrayF = data["meta_t_pred"]  # (K,)
    log_ctrl_pts = bool(data["meta_log_control_points"])

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
    pred_style = "b"

    window = plt.TabbedPlotWindow("AMPC Example")

    # Tracking tab
    fig_track = window.add_figure_tab("tracking")
    ax_t = fig_track.subplots(3, 1, sharex=True)
    ax_t_pos: Axes = ax_t[0]
    ax_t_vel: Axes = ax_t[1]
    ax_t_F: Axes = ax_t[2]

    ax_t_pos.set_ylabel("position (m)")
    ax_t_pos.grid(True)

    ax_t_vel.set_ylabel("velocity (m/s)")
    ax_t_vel.grid(True)

    ax_t_F.set_ylabel("force (N)")
    ax_t_F.set_xlabel("time (s)")
    ax_t_F.grid(True)

    # Plot tracking
    ax_t_pos.plot(time, pos, label="actual")
    ax_t_pos.plot(time, pos_ref, ref_style, label="ref")

    ax_t_vel.plot(time, vel)
    ax_t_vel.plot(time, vel_ref, ref_style)

    ax_t_F.step(time, force, where="post")
    if use_input_cost:
        ref_inputs = data["ref_inputs"]  # same shape as inputs
        ur_hist = ref_inputs if (ref_inputs.ndim == 2) else ref_inputs[:, 0, :]
        force_ref = ur_hist.squeeze()
        ax_t_F.plot(time, force_ref, ref_style)

    # Plot limits
    ax_t_F.plot(time[[0, -1]], np.tile(u_min, 2), lim_style, label="limits")
    ax_t_F.plot(time[[0, -1]], np.tile(u_max, 2), lim_style)

    ax_t_pos.legend()
    ax_t_F.legend()

    # Prediction tab
    if not is_2d:
        fig_pred = window.add_figure_tab("predictions")
        ax_p = fig_pred.subplots(3, 1, sharex=True)
        ax_p_pos: Axes = ax_p[0]
        ax_p_vel: Axes = ax_p[1]
        ax_p_F: Axes = ax_p[2]

        ax_p_pos.set_ylabel("position (m)")
        ax_p_pos.grid(True)

        ax_p_vel.set_ylabel("velocity (m/s)")
        ax_p_vel.grid(True)

        ax_p_F.set_ylabel("force (N)")
        ax_p_F.set_xlabel("time (s)")
        ax_p_F.grid(True)

        step = 2
        pos_pred, vel_pred = states.transpose(2, 0, 1)  # each now (N, K)
        force_pred = inputs.squeeze()  # now (N, K) or (N, nc)
        for i in range(0, len(time), step):
            t = time[i] + t_pred
            a = 0.15
            ax_p_pos.plot(
                t, pos_pred[i], pred_style, alpha=a, label="predicted" if i == 0 else ""
            )
            ax_p_vel.plot(t, vel_pred[i], pred_style, alpha=a)
            if not log_ctrl_pts:
                ax_p_F.step(t, force_pred[i], pred_style, where="post", alpha=a)

        # add tracking
        ax_p_pos.plot(time, pos, label="actual")
        ax_p_pos.plot(time, pos_ref, ref_style, label="ref")

        ax_p_vel.plot(time, vel)
        ax_p_vel.plot(time, vel_ref, ref_style)

        ax_p_F.step(time, force, where="post")
        if use_input_cost:
            ref_inputs = data["ref_inputs"]  # same shape as inputs
            ur_hist = ref_inputs if (ref_inputs.ndim == 2) else ref_inputs[:, 0, :]
            force_ref = ur_hist.squeeze()
            ax_p_F.plot(time, force_ref, ref_style)

        # add limits
        ax_p_F.plot(time[[0, -1]], np.tile(u_min, 2), lim_style, label="limits")
        ax_p_F.plot(time[[0, -1]], np.tile(u_max, 2), lim_style)

        ax_p_pos.legend()
        ax_p_F.legend()

    # Plot Solve Times
    fig2 = window.add_figure_tab("solve times")
    ax_st = fig2.add_subplot()
    usr_mean, osqp_mean = np.mean(solve_times, axis=0)
    ax_st.plot(time, solve_times[:, 0], label=f"user (avg: {usr_mean:.3f}ms)")
    ax_st.plot(time, solve_times[:, 1], label=f"OSQP (avg: {osqp_mean:.3f}ms)")
    ax_st.set_ylabel("solve time (ms)")
    ax_st.set_xlabel("sim time (s)")
    ax_st.grid(True)
    ax_st.legend()

    plt.show_all_windows(tight_layout=True)


if __name__ == "__main__":
    main()
