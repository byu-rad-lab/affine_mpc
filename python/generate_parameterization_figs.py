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
    print(f"Wrote figures to: {output_dir}")


if __name__ == "__main__":
    main()
