"""Model update strategies for a nonlinear quadrotor.

affine_mpc takes an affine model, so for a nonlinear plant the model is
recomputed about a new operating point as the plant moves. The strategy decides
when that update happens relative to the state measurement and which point it
uses (see the Model Update Strategies page of the docs):

- measured: at the measured state and the last applied input, after the
  measurement.
- predicted: at the one-step prediction, right after applying the input,
  followed by prepare() (real-time iteration).
- hybrid: as predicted, plus the model offset recomputed at the measured
  state with the offset-only setter after the measurement.
- hover: one fixed model about hover (a baseline, not a strategy).

The plant is a 12-state quadrotor (position, velocity, ZYX Euler angles, body
rates) driven by four rotor thrusts and integrated with RK4 at a fine step. It
flies a figure-eight with yaw motion, a body-rate kick, and a wind step.

By default the script runs one strategy at one rate (--strategy, --rate) and
logs it with MPCLogger, passing the computation after the measurement (the
latency) as loop_time. --compare runs every strategy at every rate, one log
each, and prints the comparison tables. --load reads the logs of an earlier run
instead of simulating, e.g. to print the tables or plot again.

Run with --quick for a short run and --plot to plot in tabs (needs matplotlib,
abracatabra, and a Qt binding, all in the [examples] extra). With tqdm
installed, a progress bar shows the time remaining.
Timings depend on the machine; compare ratios between strategies.
"""

# std lib
import argparse
import platform
import tempfile
from pathlib import Path
from time import perf_counter

# 3rd party
import numpy as np

try:
    from tqdm import tqdm  # optional: progress bar
except ImportError:
    tqdm = None

# local
import affine_mpc as ampc

# Quadrotor parameters (about a 1 kg, 450 mm-class airframe)
MASS = 1.0  # kg
GRAVITY = 9.81  # m/s^2
INERTIA = np.array([0.0082, 0.0082, 0.0149])  # kg m^2, principal axes
ARM = 0.2  # m, rotor distance from the center ("+" configuration)
YAW_COEF = 0.02  # m, rotor reaction torque per unit thrust
DRAG = 0.1  # N/(m/s), linear translational drag
F_MIN, F_MAX = 0.0, 5.0  # N per rotor (thrust-to-weight ratio of about 2)

# [collective thrust, roll, pitch, yaw torque] = MIXER @ rotor thrusts.
# Rotors 1-4 sit on the +x, +y, -x, -y arms; 1 and 3 spin opposite to 2 and 4.
MIXER = np.array(
    [
        [1.0, 1.0, 1.0, 1.0],
        [0.0, ARM, 0.0, -ARM],
        [-ARM, 0.0, ARM, 0.0],
        [YAW_COEF, -YAW_COEF, YAW_COEF, -YAW_COEF],
    ]
)
N_STATES, N_INPUTS = 12, 4
U_HOVER = np.full(N_INPUTS, MASS * GRAVITY / 4)

# Task: figure-eight (lateral acceleration up to about 5 m/s^2, so up to about
# 27 deg of tilt) with yaw swinging +-45 deg, a body-rate kick (e.g. a
# collision) that no prediction can anticipate, and a wind force step.
PERIOD = 6.0  # s, one figure-eight
AMP = np.array([2.0, 1.0, 0.5])  # m: x = a sin(wt), y = b sin(2wt), z = c sin(wt)
YAW_AMP = np.pi / 4
WIND = np.array([1.5, -1.0, 0.0])  # N, from half of the run on
KICK = np.array([4.0, -3.0, 0.0])  # rad/s body-rate jump at a quarter of the run

PLANT_STEP = 5e-4  # s, maximum RK4 step of the simulated plant
NOISE_STD = np.repeat([1e-3, 5e-3, 1e-3, 5e-3], 3)  # measurement noise
HORIZON = 0.8  # s
STRATEGIES = ["measured", "predicted", "hybrid", "hover"]


def dynamics(x, u, wind=None):
    """Continuous-time quadrotor dynamics dx/dt = f(x, u)."""
    v, (phi, theta, psi), (p, q, r) = x[3:6], x[6:9], x[9:12]
    thrust, tau_x, tau_y, tau_z = MIXER @ u
    sph, cph = np.sin(phi), np.cos(phi)
    sth, cth = np.sin(theta), np.cos(theta)
    sps, cps = np.sin(psi), np.cos(psi)
    jx, jy, jz = INERTIA

    dx = np.empty(N_STATES)
    dx[0:3] = v
    # third column of R = Rz(psi) Ry(theta) Rx(phi): body z axis in world frame
    z_body = np.array(
        [cps * sth * cph + sps * sph, sps * sth * cph - cps * sph, cth * cph]
    )
    dx[3:6] = (thrust * z_body - DRAG * v) / MASS
    dx[5] -= GRAVITY
    if wind is not None:
        dx[3:6] += wind / MASS
    dx[6] = p + (sph * q + cph * r) * sth / cth
    dx[7] = cph * q - sph * r
    dx[8] = (sph * q + cph * r) / cth
    dx[9] = ((jy - jz) * q * r + tau_x) / jx
    dx[10] = ((jz - jx) * p * r + tau_y) / jy
    dx[11] = ((jx - jy) * p * q + tau_z) / jz
    return dx


def jacobians(x, u):
    """Analytic Jacobians Ac = df/dx and Bc = df/du (Fortran order, as the
    bindings expect, so they are not copied)."""
    (phi, theta, psi), (p, q, r) = x[6:9], x[9:12]
    thrust = MIXER[0] @ u
    sph, cph = np.sin(phi), np.cos(phi)
    sth, cth = np.sin(theta), np.cos(theta)
    sps, cps = np.sin(psi), np.cos(psi)
    tth = sth / cth
    jx, jy, jz = INERTIA

    Ac = np.zeros((N_STATES, N_STATES), order="F")
    Bc = np.zeros((N_STATES, N_INPUTS), order="F")
    Ac[0:3, 3:6] = np.eye(3)
    Ac[3:6, 3:6] = -DRAG / MASS * np.eye(3)
    # d(z_body)/d(phi, theta, psi), scaled by thrust / mass
    Ac[3:6, 6:9] = (thrust / MASS) * np.array(
        [
            [
                -cps * sth * sph + sps * cph,
                cps * cth * cph,
                -sps * sth * cph + cps * sph,
            ],
            [
                -sps * sth * sph - cps * cph,
                sps * cth * cph,
                cps * sth * cph + sps * sph,
            ],
            [-cth * sph, -sth * cph, 0.0],
        ]
    )
    z_body = np.array(
        [cps * sth * cph + sps * sph, sps * sth * cph - cps * sph, cth * cph]
    )
    Bc[3:6, :] = np.outer(z_body, MIXER[0]) / MASS
    # Euler angle rates W(phi, theta) omega
    a, b = sph * q + cph * r, cph * q - sph * r
    Ac[6:9, 6:8] = [
        [b * tth, a / cth**2],
        [-a, 0.0],
        [b / cth, a * sth / cth**2],
    ]
    Ac[6:9, 9:12] = [
        [1.0, sph * tth, cph * tth],
        [0.0, cph, -sph],
        [0.0, sph / cth, cph / cth],
    ]
    # Euler's rotation equations
    Ac[9:12, 9:12] = [
        [0.0, (jy - jz) * r / jx, (jy - jz) * q / jx],
        [(jz - jx) * r / jy, 0.0, (jz - jx) * p / jy],
        [(jx - jy) * q / jz, (jx - jy) * p / jz, 0.0],
    ]
    Bc[9:12, :] = MIXER[1:] / INERTIA[:, None]
    return Ac, Bc


def affine_model(x, u):
    """First-order approximation dx/dt ~ Ac x + Bc u + wc about (x, u)."""
    Ac, Bc = jacobians(x, u)
    wc = dynamics(x, u) - Ac @ x - Bc @ u
    return Ac, Bc, wc


def check_jacobians(rng):
    """Compare the analytic Jacobians with central differences."""
    x = rng.uniform(-0.5, 0.5, N_STATES)
    u = rng.uniform(F_MIN, F_MAX, N_INPUTS)
    Ac, Bc = jacobians(x, u)
    h = 1e-6
    err = 0.0
    for i in range(N_STATES + N_INPUTS):
        dz = np.zeros(N_STATES + N_INPUTS)
        dz[i] = h
        fp = dynamics(x + dz[:N_STATES], u + dz[N_STATES:])
        fm = dynamics(x - dz[:N_STATES], u - dz[N_STATES:])
        col = Ac[:, i] if i < N_STATES else Bc[:, i - N_STATES]
        err = max(err, np.max(np.abs((fp - fm) / (2 * h) - col)))
    if err > 1e-6:
        raise RuntimeError(
            f"Analytic Jacobians disagree with finite differences ({err:.1e})"
        )


def rk4(x, u, duration, wind=None, max_step=PLANT_STEP):
    """Integrate the dynamics over duration with a zero-order-hold input."""
    n = max(1, int(np.ceil(duration / max_step - 1e-9)))
    h = duration / n
    for _ in range(n):
        k1 = dynamics(x, u, wind)
        k2 = dynamics(x + 0.5 * h * k1, u, wind)
        k3 = dynamics(x + 0.5 * h * k2, u, wind)
        k4 = dynamics(x + h * k3, u, wind)
        x = x + h / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
    return x


def reference(t):
    """Reference states (len(t), 12) and rotor thrusts (len(t), 4) from
    differential flatness of the figure-eight."""
    t = np.asarray(t, dtype=float)
    w = 2 * np.pi / PERIOD
    s1, c1, s2, c2 = np.sin(w * t), np.cos(w * t), np.sin(2 * w * t), np.cos(2 * w * t)
    pos = np.stack([AMP[0] * s1, AMP[1] * s2, AMP[2] * s1], axis=1)
    vel = np.stack([AMP[0] * w * c1, 2 * AMP[1] * w * c2, AMP[2] * w * c1], axis=1)
    acc = np.stack(
        [-AMP[0] * w**2 * s1, -4 * AMP[1] * w**2 * s2, -AMP[2] * w**2 * s1], axis=1
    )
    psi = YAW_AMP * np.sin(0.5 * w * t)
    psi_dot = 0.5 * w * YAW_AMP * np.cos(0.5 * w * t)

    # thrust vector, body axes, and ZYX angles
    force = MASS * acc + DRAG * vel
    force[:, 2] += MASS * GRAVITY
    thrust = np.linalg.norm(force, axis=1)
    z_b = force / thrust[:, None]
    x_c = np.stack([np.cos(psi), np.sin(psi), np.zeros_like(psi)], axis=1)
    y_b = np.cross(z_b, x_c)
    y_b /= np.linalg.norm(y_b, axis=1)[:, None]
    x_b = np.cross(y_b, z_b)
    phi = np.arctan2(y_b[:, 2], z_b[:, 2])
    theta = -np.arcsin(x_b[:, 2])

    # body rates from the Euler angle rates (roll and pitch rates by finite
    # differences; they get a small weight, so this accuracy is enough)
    phi_dot = np.gradient(phi, t) if len(t) > 1 else np.zeros_like(t)
    theta_dot = np.gradient(theta, t) if len(t) > 1 else np.zeros_like(t)
    sph, cph, sth, cth = np.sin(phi), np.cos(phi), np.sin(theta), np.cos(theta)
    p = phi_dot - sth * psi_dot
    q = cph * theta_dot + sph * cth * psi_dot
    r = -sph * theta_dot + cph * cth * psi_dot

    x_ref = np.column_stack([pos, vel, phi, theta, psi, p, q, r])
    u_ref = np.repeat(thrust[:, None] / 4, N_INPUTS, axis=1)
    return x_ref, u_ref


def build_mpc(dt, rng):
    horizon_steps = int(round(HORIZON / dt))
    # Degree-2 B-spline with 8 control points per input; geometric knots put
    # short intervals at the start of the horizon (where feedback acts) and
    # long ones at the end. The same knots in seconds at every rate.
    param = ampc.Parameterization.bspline(
        horizon_steps, degree=2, num_control_points=8, knot_ratio=1.5
    )
    mpc = ampc.MPC(N_STATES, N_INPUTS, param, ampc.Options(use_input_cost=True))
    # Initialize with a model at a random point so that the QP sparsity pattern
    # covers every entry that becomes nonzero away from hover.
    x_rand = rng.uniform(-0.5, 0.5, N_STATES)
    u_rand = rng.uniform(F_MIN, F_MAX, N_INPUTS)
    mpc.setModelContinuous2Discrete(*affine_model(x_rand, u_rand), dt)
    mpc.setInputLimits(np.full(N_INPUTS, F_MIN), np.full(N_INPUTS, F_MAX))
    # position 1 cm ~ velocity 3 cm/s ~ attitude 1 deg ~ thrust 0.3 N
    q_diag = np.array([40, 40, 40, 2, 2, 2, 1, 1, 4, 0.01, 0.01, 0.01], dtype=float)
    r_diag = np.full(N_INPUTS, 0.05)
    mpc.setWeights(q_diag, q_diag, r_diag)
    mpc.setReferenceState(np.zeros(N_STATES))
    mpc.setReferenceInput(U_HOVER)
    if not mpc.initializeSolver():
        raise RuntimeError("Failed to initialize the solver")
    return mpc, horizon_steps


def versions():
    return (
        f"affine_mpc {getattr(ampc, '__version__', 'unknown')}, "
        f"NumPy {np.__version__}, "
        f"Python {platform.python_version()}, {platform.platform()}"
    )


def run_dir(log_dir, rate, strategy, delay_scale):
    """Directory of one run's log: one MPCLogger log per run."""
    name = f"{rate}hz_{strategy}"
    if delay_scale > 0:
        name += f"_delay{delay_scale:g}"
    return log_dir / name


def simulate(strategy, rate, duration, delay_scale, seed, save_dir, progress=None):
    """Closed-loop run of one strategy at one rate, logged with MPCLogger to
    save_dir/log.npz and save_dir/params.yaml.

    With delay_scale > 0, each new input reaches the plant delay_scale times
    its measured latency after the measurement (at most one sample later), and
    the previous input stays applied until then.
    """
    dt = 1.0 / rate
    rng = np.random.default_rng(seed)
    mpc, horizon_steps = build_mpc(dt, rng)
    num_steps = int(round(duration / dt))
    x_ref, u_ref = reference(np.arange(num_steps + horizon_steps + 1) * dt)

    # Log only the current step of each solve (prediction_stride=0); the
    # latency from measurement to input is passed as loop_time.
    logger = ampc.MPCLogger(mpc, save_dir, dt, prediction_stride=0)
    logger.addMetadata("example_name", "quadrotor_model_updates")
    logger.addMetadata("versions", versions())
    logger.addMetadata("strategy", strategy)
    logger.addMetadata("strategy_index", STRATEGIES.index(strategy))
    logger.addMetadata("rate", rate)
    logger.addMetadata("duration", duration)
    logger.addMetadata("seed", seed)
    logger.addMetadata("delay_scale", delay_scale)

    x = x_ref[0].copy()
    u_prev = u_ref[0].copy()
    if strategy == "hover":
        mpc.setModelContinuous2Discrete(*affine_model(np.zeros(N_STATES), U_HOVER), dt)
    else:
        # the first operating point is the initial state for every strategy
        Ac, Bc, wc = affine_model(x, u_prev)
        mpc.setModelContinuous2Discrete(Ac, Bc, wc, dt)

    failures = 0
    kick_step = int(round(0.25 * duration / dt))
    for k in range(num_steps):
        t = k * dt
        wind = WIND if t >= 0.5 * duration else None
        if k == kick_step:
            x[9:12] += KICK
        # Work before the measurement: references (and for the predicted
        # strategies the model, see the end of the loop) are applied by prepare().
        mpc.setReferenceStateTrajectory(x_ref[k + 1 : k + 1 + horizon_steps].ravel())
        mpc.setReferenceInput(u_ref[k])
        if strategy in ("predicted", "hybrid"):
            mpc.prepare()

        y = x + NOISE_STD * rng.standard_normal(N_STATES)  # measurement

        # Work after the measurement (the latency)
        t0 = perf_counter()
        if strategy == "measured":
            Ac, Bc, wc = affine_model(y, u_prev)
            mpc.setModelContinuous2Discrete(Ac, Bc, wc, dt)
        elif strategy == "hybrid":
            wc = dynamics(y, u_prev) - Ac @ y - Bc @ u_prev
            mpc.setModelOffsetContinuous2Discrete(wc)
        status = mpc.solve(y)
        u = mpc.getNextInput()
        latency = perf_counter() - t0

        logger.logStep(t, x, loop_time=latency)  # the true state, not y
        if status != ampc.SolveStatus.Success:
            failures += 1
            if status not in (
                ampc.SolveStatus.SolvedInaccurate,
                ampc.SolveStatus.MaxIterReached,
            ):
                u = u_prev  # no usable iterate: hold the last input

        # Plant: the new input takes effect after the (scaled) latency
        delay = min(delay_scale * latency, dt)
        if delay > 0.0:
            x = rk4(x, u_prev, delay, wind)
        x = rk4(x, u, dt - delay, wind)

        # Work after applying the input, before the next measurement
        if strategy in ("predicted", "hybrid"):
            x_pred = rk4(y, u, dt, max_step=dt)  # nominal model, no wind
            Ac, Bc, wc = affine_model(x_pred, u)
            mpc.setModelContinuous2Discrete(Ac, Bc, wc, dt)
        u_prev = u
        if progress is not None:
            progress.update()  # outside the timed section

    logger.addMetadata("failed_solves", failures)
    logger.addMetadata("within_sparsity_pattern", int(mpc.isWithinSparsityPattern()))
    logger.finalize()


def load_run(save_dir):
    """Read one run's log; the position error is computed against the
    reference at each logged time."""
    path = save_dir / "log.npz"
    if not path.exists():
        raise FileNotFoundError(path)
    with np.load(path) as data:
        run = {
            key: data[f"meta_{key}"].item()
            for key in ("strategy_index", "rate", "duration", "seed", "delay_scale")
        }
        run["failed_solves"] = data["meta_failed_solves"].item()
        run["within_sparsity_pattern"] = bool(data["meta_within_sparsity_pattern"])
        run["time"] = data["time"]
        run["pos"] = data["states"][:, 0:3]
        run["latency"] = data["loop_time"]
        run["mpc_solve"] = data["mpc_solve_time"]
        run["prepare"] = data["prepare_time"]
    run["strategy"] = STRATEGIES[run["strategy_index"]]
    run["pos_ref"] = reference(run["time"])[0][:, 0:3]
    run["pos_err"] = np.linalg.norm(run["pos"] - run["pos_ref"], axis=1)
    return run


def summarize(run):
    ms = 1e3
    err = run["pos_err"] * 1e3  # mm
    n = len(err)
    nominal, kick, wind = err[: n // 4], err[n // 4 : n // 2], err[n // 2 :]
    return {
        "rms_nominal": np.sqrt(np.mean(nominal**2)),
        "max_kick": np.max(kick),
        "rms_wind": np.sqrt(np.mean(wind**2)),
        "rms": np.sqrt(np.mean(err**2)),
        "max": np.max(err),
        "latency": np.median(run["latency"]) * ms,
        "mpc_solve": np.median(run["mpc_solve"]) * ms,
        "prepare": np.median(run["prepare"]) * ms,
    }


def markdown_table(header, rows):
    """Markdown table with padded columns; the strategy column is
    left-aligned, all others right-aligned."""
    widths = [max(len(row[j]) for row in [header] + rows) for j in range(len(header))]

    def line(cells):
        padded = [
            cell.ljust(w) if j == 1 else cell.rjust(w)
            for j, (cell, w) in enumerate(zip(cells, widths))
        ]
        return "| " + " | ".join(padded) + " |"

    rule = ["-" * w if j == 1 else "-" * (w - 1) + ":" for j, w in enumerate(widths)]
    return "\n".join([line(header), line(rule)] + [line(row) for row in rows])


def print_tables(runs, delayed):
    """runs and delayed map (rate, strategy) to loaded runs; delayed may be
    empty."""
    stats = {key: summarize(run) for key, run in runs.items()}
    print("\nTracking: position error (mm)\n")
    header = ["Rate", "Strategy", "RMS before kick (mm)", "Max after kick (mm)"]
    header += ["RMS in wind (mm)"]
    rows = [
        [f"{rate} Hz", strategy]
        + [f"{s[key]:.2f}" for key in ("rms_nominal", "max_kick", "rms_wind")]
        for (rate, strategy), s in stats.items()
    ]
    print(markdown_table(header, rows))
    if delayed:
        scale = next(iter(delayed.values()))["delay_scale"]
        changes = [(summarize(run), stats[key]) for key, run in delayed.items()]
        rms = max(abs(d["rms"] - s["rms"]) for d, s in changes)
        peak = max(abs(d["max"] - s["max"]) for d, s in changes)
        print(
            f"\nWith each input applied {scale:g}x its measured latency after "
            f"the measurement, the RMS error changed by at most {rms:.2f} mm and "
            f"the max error by at most {peak:.2f} mm."
        )

    print(f"\nTiming: medians per step{', runs without delay' if delayed else ''}\n")
    header = ["Rate", "Strategy", "Latency (ms)", "mpc_solve_time (ms)"]
    header += ["prepare_time (ms)"]
    rows = [
        [f"{rate} Hz", strategy]
        + [f"{s[key]:.3f}" for key in ("latency", "mpc_solve", "prepare")]
        for (rate, strategy), s in stats.items()
    ]
    print(markdown_table(header, rows))

    every = list(runs.values()) + list(delayed.values())
    failed = [run for run in every if run["failed_solves"] > 0]
    for run in failed:
        print(
            f"\n{run['rate']} Hz {run['strategy']} (delay scale "
            f"{run['delay_scale']:g}): {run['failed_solves']} solves did not "
            "return Success"
        )
    for run in every:
        if not run["within_sparsity_pattern"]:
            print(
                f"\n{run['rate']} Hz {run['strategy']}: QP matrices left the "
                "initialized sparsity pattern"
            )


def plot(runs):
    """Tabbed plots (abracatabra): trajectory, position error, and timing.
    Each strategy keeps one color; with several rates, each rate gets a row."""
    try:
        import abracatabra as abra
    except ImportError:
        print(
            "Plotting needs abracatabra and a Qt binding, bundled with affine_mpc:",
            '    pip install "affine_mpc[examples]"',
            "OR independently:",
            "    pip install matplotlib abracatabra pyside6",
            sep="\n",
        )
        return
    color = {s: f"C{i}" for i, s in enumerate(STRATEGIES)}
    rates = sorted({rate for rate, _ in runs})
    first = next(iter(runs.values()))
    duration = first["duration"]
    # one lap only: the dashes of a second lap would fill the gaps of the first
    lap = np.linspace(0.0, min(PERIOD, duration), 1000)
    pos_ref = reference(lap)[0][:, 0:3]

    def label(rate, strategy):
        return strategy if len(rates) > 1 else f"{strategy}, {rate} Hz"

    def mark_events(ax):
        ax.axvline(0.25 * duration, color="gray", linestyle="--", label="kick")
        ax.axvline(0.5 * duration, color="gray", linestyle=":", label="wind step")

    window = abra.TabbedPlotWindow("Quadrotor model updates")

    # x-y path of every run, the reference drawn last (on top) but listed first
    ax = window.add_figure_tab("trajectory").subplots()
    for (rate, strategy), run in runs.items():
        style = "-" if rate == rates[0] else "--"
        ax.plot(
            run["pos"][:, 0],
            run["pos"][:, 1],
            style,
            color=color[strategy],
            label=f"{strategy}, {rate} Hz",
        )
    for i, run in enumerate(runs.values()):
        k = np.searchsorted(run["time"], 0.25 * duration)
        ax.plot(
            *run["pos"][k, 0:2],
            "kx",
            markersize=8,
            label="kick" if i == 0 else None,
            zorder=4,
        )
    ax.plot(pos_ref[:, 0], pos_ref[:, 1], "k:", label="reference", zorder=3)
    handles, labels = ax.get_legend_handles_labels()
    ax.legend(
        handles[-1:] + handles[:-1],
        labels[-1:] + labels[:-1],
        loc="center left",
        bbox_to_anchor=(1.02, 0.5),
    )
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.axis("equal")
    ax.grid(True)

    # position error and latency over time: one row per rate
    fig_err = window.add_figure_tab("position error")
    fig_time = window.add_figure_tab("timing")
    axes_err = np.atleast_1d(fig_err.subplots(len(rates), 1, sharex=True))
    axes_time = np.atleast_1d(fig_time.subplots(len(rates), 1, sharex=True))
    single = len(runs) == 1
    for row, rate in enumerate(rates):
        ax_err, ax_time = axes_err[row], axes_time[row]
        for (r, strategy), run in runs.items():
            if r != rate:
                continue
            ax_err.plot(
                run["time"],
                run["pos_err"] * 100,
                color=color[strategy],
                label=label(rate, strategy),
            )
            ms = 1e3
            if single:
                # one run: the parts of its timing, in colors no strategy uses
                for key, name, c in [
                    ("latency", "latency", "C4"),
                    ("mpc_solve", "mpc_solve_time", "C5"),
                    ("prepare", "prepare_time", "C9"),
                ]:
                    ax_time.plot(run["time"], run[key] * ms, color=c, label=name)
            else:
                ax_time.plot(
                    run["time"],
                    run["latency"] * ms,
                    color=color[strategy],
                    label=label(rate, strategy),
                )
        mark_events(ax_err)
        ax_err.set_ylabel("position error (cm)")
        ax_time.set_ylabel("time (ms)" if single else "latency (ms)")
        if not single:
            ax_time.set_yscale("log")
        for ax in (ax_err, ax_time):
            if len(rates) > 1:
                ax.set_title(f"{rate} Hz")
            ax.grid(True)
            ax.legend(loc="upper left")
    axes_err[-1].set_xlabel("time (s)")
    axes_time[-1].set_xlabel("time (s)")

    abra.show_all_windows(tight_layout=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument(
        "--strategy",
        choices=STRATEGIES,
        help="strategy of a single run (default: hybrid)",
    )
    parser.add_argument(
        "--rate", type=int, help="rate of a single run in Hz (default: 100)"
    )
    parser.add_argument(
        "--compare",
        action="store_true",
        help="run every strategy in --strategies at every rate in --rates and "
        "print the comparison tables",
    )
    parser.add_argument(
        "--strategies",
        nargs="+",
        choices=STRATEGIES,
        help="strategies for --compare (default: all)",
    )
    parser.add_argument(
        "--rates",
        type=int,
        nargs="+",
        help="rates in Hz for --compare (default: 100 500)",
    )
    parser.add_argument("--quick", action="store_true", help="short run (3 s)")
    parser.add_argument("--duration", type=float, default=12.0, help="seconds")
    parser.add_argument(
        "--delay-scale",
        type=float,
        default=0.0,
        help="apply each input this many times its measured latency after the "
        "measurement (0: immediately); with --compare, every case also runs "
        "without delay and the change is reported",
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=0,
        help="seed of the measurement noise and the initial random model "
        "(default: 0)",
    )
    parser.add_argument(
        "--log-dir",
        type=Path,
        default=Path(tempfile.gettempdir()) / "ampc_example" / "quadrotor",
        help="directory for the logs, one subdirectory per run",
    )
    parser.add_argument(
        "--load",
        action="store_true",
        help="read the logs of an earlier run instead of simulating",
    )
    parser.add_argument(
        "--plot",
        action="store_true",
        help="show trajectory, position error, and timing tabs (needs "
        "matplotlib, abracatabra, and a Qt binding)",
    )
    args = parser.parse_args()
    duration = 3.0 if args.quick else args.duration

    # One run (--strategy, --rate) or a comparison (--compare with --strategies,
    # --rates); options of the other mode are an error rather than ignored.
    if args.compare:
        if args.strategy is not None or args.rate is not None:
            parser.error(
                "--strategy and --rate select a single run; with --compare, "
                "use --strategies and --rates"
            )
        strategies = args.strategies or STRATEGIES
        rates = args.rates or [100, 500]
    else:
        if args.strategies is not None or args.rates is not None:
            parser.error(
                "--strategies and --rates select the runs of --compare; for a "
                "single run, use --strategy and --rate"
            )
        strategies = [args.strategy or "hybrid"]
        rates = [args.rate or 100]
    cases = [(rate, s) for rate in rates for s in strategies]
    # (rate, strategy, delay scale) of every log; with --compare, the delayed
    # runs repeat each case
    logs = [(rate, s, 0.0 if args.compare else args.delay_scale) for rate, s in cases]
    if args.compare and args.delay_scale > 0:
        logs += [(rate, s, args.delay_scale) for rate, s in cases]

    if not args.load:
        print(versions())
        print(
            f"duration {duration:g} s, seed {args.seed}, delay scale "
            f"{args.delay_scale:g}, logs in {args.log_dir}"
        )
        check_jacobians(np.random.default_rng(args.seed))
        total_steps = sum(int(round(duration * rate)) for rate, _, _ in logs)
        progress = None
        if tqdm is not None:
            progress = tqdm(total=total_steps, unit="step", leave=False)
        for rate, strategy, scale in logs:
            if progress is not None:
                delayed = f", delay {scale:g}x" if scale > 0 else ""
                progress.set_description(f"{rate} Hz {strategy}{delayed}")
            save_dir = run_dir(args.log_dir, rate, strategy, scale)
            simulate(strategy, rate, duration, scale, args.seed, save_dir, progress)
        if progress is not None:
            progress.close()

    missing = [
        run_dir(args.log_dir, *log)
        for log in logs
        if not (run_dir(args.log_dir, *log) / "log.npz").exists()
    ]
    if missing:
        raise SystemExit(
            "Missing logs (run without --load first):\n"
            + "\n".join(f"  {path}" for path in missing)
        )
    loaded = {log: load_run(run_dir(args.log_dir, *log)) for log in logs}
    for (rate, strategy, scale), run in loaded.items():
        if (run["rate"], run["strategy"], run["delay_scale"]) != (
            rate,
            strategy,
            scale,
        ):
            raise SystemExit(
                f"{run_dir(args.log_dir, rate, strategy, scale)} "
                "holds a different run"
            )
    settings = {(run["duration"], run["seed"]) for run in loaded.values()}
    if len(settings) > 1:
        raise SystemExit(
            "The logs were made with different durations or seeds; rerun them "
            "together (without --load)."
        )
    if args.load:
        duration, seed = settings.pop()
        print(f"logs in {args.log_dir}: duration {duration:g} s, seed {seed}")

    runs = {
        (r, s): run
        for (r, s, scale), run in loaded.items()
        if scale == 0.0 or not args.compare
    }
    delayed = {
        (r, s): run
        for (r, s, scale), run in loaded.items()
        if args.compare and scale > 0
    }
    print_tables(runs, delayed)
    if not args.load and not args.compare:
        print("\nLog written to", run_dir(args.log_dir, *logs[0]) / "log.npz")
    if args.plot:
        plot(runs)


if __name__ == "__main__":
    main()
