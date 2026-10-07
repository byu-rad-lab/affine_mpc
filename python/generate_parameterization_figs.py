import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path


def make_uniform_clamped_knots(
    horizon_steps: int, degree: int, num_control_points: int
) -> np.ndarray:
    if horizon_steps < 1:
        raise ValueError("horizon_steps must be positive")
    if degree < 0:
        raise ValueError("degree must be nonnegative")
    if degree >= horizon_steps:
        raise ValueError("degree must be less than horizon_steps")
    if num_control_points < degree + 1:
        raise ValueError("must have at least degree + 1 control points")
    if num_control_points > horizon_steps:
        raise ValueError("num_control_points cannot exceed horizon_steps")
    knots = np.empty(num_control_points + degree + 1, dtype=float)
    if degree > 0:
        knots[:degree] = 0.0
        knots[-degree:] = horizon_steps - 1.0
    num_active_knots = knots.size - 2 * degree
    active_knots = np.linspace(0.0, horizon_steps - 1.0, num_active_knots)
    knots[degree : knots.size - degree] = active_knots
    return knots


def make_clamped_knots_from_active(
    horizon_steps: int, degree: int, active_knots: np.ndarray
) -> np.ndarray:
    active_knots = np.asarray(active_knots, dtype=float)
    if active_knots.ndim != 1:
        raise ValueError("active_knots must be a 1D array")
    if active_knots.size < 2:
        raise ValueError("active_knots must contain at least [0, T-1]")
    if active_knots[0] != 0.0 or active_knots[-1] != horizon_steps - 1.0:
        raise ValueError("active_knots must start at 0 and end at T-1")
    if np.any(np.diff(active_knots) <= 0.0):
        raise ValueError("active_knots must be strictly increasing")
    return np.concatenate(
        [
            np.zeros(degree),
            active_knots,
            np.full(degree, horizon_steps - 1.0),
        ]
    )


def active_knots_from_full(knots: np.ndarray, degree: int) -> np.ndarray:
    if degree == 0:
        return knots.copy()
    return knots[degree : len(knots) - degree]


def greville_abscissae(knots: np.ndarray, degree: int) -> np.ndarray:
    num_control_points = len(knots) - degree - 1
    if degree == 0:
        return knots[:-1].copy()
    return np.array(
        [
            np.sum(knots[i + 1 : i + degree + 1]) / degree
            for i in range(num_control_points)
        ]
    )


def bspline_basis(i: int, degree: int, t: float, knots: np.ndarray) -> float:
    if degree == 0:
        left = knots[i]
        right = knots[i + 1]
        at_last_knot = (
            np.isclose(t, knots[-1]) and np.isclose(right, knots[-1]) and left < right
        )
        if (left <= t < right) or at_last_knot:
            return 1.0
        return 0.0
    value = 0.0
    denom1 = knots[i + degree] - knots[i]
    if denom1 > 0.0:
        value += ((t - knots[i]) / denom1) * bspline_basis(i, degree - 1, t, knots)
    denom2 = knots[i + degree + 1] - knots[i + 1]
    if denom2 > 0.0:
        value += ((knots[i + degree + 1] - t) / denom2) * bspline_basis(
            i + 1, degree - 1, t, knots
        )
    return value


def evaluate_spline(
    control_points: np.ndarray, degree: int, knots: np.ndarray, t: np.ndarray
) -> np.ndarray:
    control_points = np.asarray(control_points, dtype=float)
    t = np.asarray(t, dtype=float)
    values = np.zeros_like(t, dtype=float)
    for j, tj in enumerate(t):
        values[j] = sum(
            control_points[i] * bspline_basis(i, degree, tj, knots)
            for i in range(len(control_points))
        )
    return values


def style_axis(ax, horizon_steps: int, title: str):
    ax.set_title(title)
    ax.set_xlim(0, horizon_steps - 1)
    # ax.set_xlabel("Horizon index / spline parameter")
    # ax.set_ylabel("Input value")
    ax.set_xlabel("t / k")
    ax.set_ylabel("u")
    ax.grid(True, alpha=0.25)


def plot_parameterization(
    ax,
    horizon_steps: int,
    degree: int,
    knots: np.ndarray,
    control_points: np.ndarray,
    title: str,
):
    t_dense = np.linspace(0.0, horizon_steps - 1.0, 1000)
    u_dense = evaluate_spline(control_points, degree, knots, t_dense)
    k = np.arange(horizon_steps, dtype=float)
    u_samples = evaluate_spline(control_points, degree, knots, k)
    active_knots = active_knots_from_full(knots, degree)
    ctrl_x = greville_abscissae(knots, degree)
    for idx, tau in enumerate(active_knots):
        ax.axvline(
            tau,
            color="0.3",
            linestyle="--",
            linewidth=1.0,
            alpha=0.95,
            label="active knots" if idx == 0 else None,
        )
    ax.plot(t_dense, u_dense, color="#1f77b4", linewidth=2.0, label="continuous spline")
    ax.scatter(
        k, u_samples, color="#d62728", s=28, zorder=3, label="sampled inputs $u_k$"
    )
    ax.scatter(
        ctrl_x,
        control_points,
        marker="D",
        s=42,
        facecolors="white",
        edgecolors="black",
        linewidths=1.2,
        zorder=4,
        label="control points",
    )
    style_axis(ax, horizon_steps, title)


def make_factory_methods_figure(output_dir: Path):
    horizon_steps = 13
    fig, axes = plt.subplots(1, 3, figsize=(14, 5.2), constrained_layout=True)
    cp_move = np.array([0.2, 0.9, -0.4, 0.6])
    knots_move = make_uniform_clamped_knots(
        horizon_steps, degree=0, num_control_points=len(cp_move)
    )
    plot_parameterization(
        axes[0],
        horizon_steps,
        degree=0,
        knots=knots_move,
        control_points=cp_move,
        title="moveBlocking()",
    )
    cp_linear = np.array([0.2, 0.9, -0.4, 0.6])
    knots_linear = make_uniform_clamped_knots(
        horizon_steps, degree=1, num_control_points=len(cp_linear)
    )
    plot_parameterization(
        axes[1],
        horizon_steps,
        degree=1,
        knots=knots_linear,
        control_points=cp_linear,
        title="linearInterp()",
    )
    cp_bspline = np.array([0.2, 0.9, -0.5, 1.0, 0.4])
    knots_bspline = make_uniform_clamped_knots(
        horizon_steps, degree=3, num_control_points=len(cp_bspline)
    )
    plot_parameterization(
        axes[2],
        horizon_steps,
        degree=3,
        knots=knots_bspline,
        control_points=cp_bspline,
        title="bspline(), degree = 3",
    )
    handles, labels = axes[2].get_legend_handles_labels()
    fig.legend(
        handles,
        labels,
        loc="upper center",
        ncol=4,
        frameon=False,
        bbox_to_anchor=(0.5, 1.08),
    )
    output_path = output_dir / "factory-methods.svg"
    fig.savefig(output_path, format="svg", bbox_inches="tight")
    plt.close(fig)


def make_knot_placement_figure(output_dir: Path):
    horizon_steps = 13
    degree = 3
    control_points = np.array([0.0, 0.9, -0.8, 1.0, -0.2, 0.3])
    uniform_knots = make_uniform_clamped_knots(
        horizon_steps,
        degree=degree,
        num_control_points=len(control_points),
    )
    custom_active_knots = np.array([0.0, 1.5, 3.0, horizon_steps - 1.0])
    custom_knots = make_clamped_knots_from_active(
        horizon_steps,
        degree=degree,
        active_knots=custom_active_knots,
    )
    fig, axes = plt.subplots(1, 2, figsize=(10.5, 4.2), constrained_layout=True)
    plot_parameterization(
        axes[0],
        horizon_steps,
        degree=degree,
        knots=uniform_knots,
        control_points=control_points,
        title="Uniform Active Knots",
    )
    plot_parameterization(
        axes[1],
        horizon_steps,
        degree=degree,
        knots=custom_knots,
        control_points=control_points,
        title="Custom Active Knots",
    )
    handles, labels = axes[1].get_legend_handles_labels()
    fig.legend(
        handles,
        labels,
        loc="upper center",
        ncol=4,
        frameon=False,
        bbox_to_anchor=(0.5, 1.08),
    )
    output_path = output_dir / "knot-placement.svg"
    fig.savefig(output_path, format="svg", bbox_inches="tight")
    plt.close(fig)


def degree1_saturation_samples(active_knots: np.ndarray) -> np.ndarray:
    """Samples bounded by MPCBase for degree 1 when saturating the trajectory."""
    samples = []
    for tau in active_knots:
        tau_round = np.round(tau)
        if abs(tau - tau_round) <= 1e-9:
            samples.append(int(tau_round))
        else:
            samples += [int(np.floor(tau)), int(np.ceil(tau))]
    return np.unique(samples)


def plot_saturation_comparison(
    output_path: Path,
    horizon_steps: int,
    degree: int,
    knots: np.ndarray,
    cp_ctrls: np.ndarray,
    cp_traj: np.ndarray,
    samples: np.ndarray,
    samples_title: str,
    u_min: float = 0.0,
    u_max: float = 1.0,
):
    """Compare saturating control points (left) with saturating samples (right)."""
    fig, axes = plt.subplots(
        1, 2, figsize=(10.5, 4.2), sharey=True, constrained_layout=True
    )
    ctrl_x = greville_abscissae(knots, degree)
    sample_x = samples.astype(float)
    panels = [
        (axes[0], cp_ctrls, "Saturate Control Points", ctrl_x, cp_ctrls),
        (
            axes[1],
            cp_traj,
            samples_title,
            sample_x,
            evaluate_spline(cp_traj, degree, knots, sample_x),
        ),
    ]
    for ax, control_points, title, cons_x, cons_u in panels:
        ax.axhspan(u_min, u_max, color="0.85", alpha=0.5, zorder=0)
        for limit in (u_min, u_max):
            ax.axhline(limit, color="0.4", linewidth=1.0, zorder=1)
        plot_parameterization(ax, horizon_steps, degree, knots, control_points, title)
        ax.scatter(
            cons_x,
            cons_u,
            s=110,
            facecolors="none",
            edgecolors="#2ca02c",
            linewidths=1.8,
            zorder=5,
            label="constrained",
        )
    margin = 0.25 * (u_max - u_min)
    y_lo = min(u_min - margin, cp_traj.min() - 0.1 * margin)
    y_hi = max(u_max + margin, cp_traj.max() + 0.1 * margin)
    axes[0].set_ylim(y_lo, y_hi)

    handles, labels = axes[1].get_legend_handles_labels()
    fig.legend(
        handles,
        labels,
        loc="upper center",
        ncol=5,
        frameon=False,
        bbox_to_anchor=(0.5, 1.08),
    )
    fig.savefig(output_path, format="svg", bbox_inches="tight")
    plt.close(fig)


def make_degree1_saturation_figure(output_dir: Path):
    horizon_steps = 13
    degree = 1
    active_knots = np.array([0.0, 2.5, 6.5, horizon_steps - 1.0])
    knots = make_clamped_knots_from_active(horizon_steps, degree, active_knots)
    # same control points as the MPCBase degree 1 saturation test: u_3 = u_max
    # and u_7 = u_min while the control points at 2.5 and 6.5 leave the limits
    cp_traj = np.array([0.0, 0.0, -0.08, 0.8])
    cp_traj[1] = (4.0 - 0.5 * cp_traj[2]) / 3.5
    plot_saturation_comparison(
        output_dir / "degree1-saturation.svg",
        horizon_steps,
        degree,
        knots,
        cp_ctrls=np.clip(cp_traj, 0.0, 1.0),
        cp_traj=cp_traj,
        samples=degree1_saturation_samples(active_knots),
        samples_title="Saturate Knot-Adjacent Samples",
    )


def make_degree3_saturation_figure(output_dir: Path):
    horizon_steps = 13
    degree = 3
    shape = np.array([0.0, 1.6, -0.6, 1.3, -0.2, 0.6])
    knots = make_uniform_clamped_knots(horizon_steps, degree, len(shape))
    k = np.arange(horizon_steps, dtype=float)

    # Same shape in both panels: scaled so the control points span the limits
    # (left) or so the sampled inputs span the limits (right). B-spline basis
    # functions sum to 1, so an affine map of the control points maps the
    # spline the same way.
    def normalize(cp: np.ndarray, values: np.ndarray) -> np.ndarray:
        return (cp - values.min()) / (values.max() - values.min())

    plot_saturation_comparison(
        output_dir / "degree3-saturation.svg",
        horizon_steps,
        degree,
        knots,
        cp_ctrls=normalize(shape, shape),
        cp_traj=normalize(shape, evaluate_spline(shape, degree, knots, k)),
        samples=np.arange(horizon_steps),
        samples_title="Saturate Every Sample",
    )


def main():
    output_dir = Path("docs/assets/input-parameterization")
    output_dir.mkdir(parents=True, exist_ok=True)
    plt.rcParams.update(
        {
            "font.size": 10,
            "axes.titlesize": 17,
            "axes.labelsize": 12,
            "legend.fontsize": 12,
        }
    )
    make_factory_methods_figure(output_dir)
    make_knot_placement_figure(output_dir)
    make_degree1_saturation_figure(output_dir)
    make_degree3_saturation_figure(output_dir)
    print(f"Wrote figures to: {output_dir}")


if __name__ == "__main__":
    main()
