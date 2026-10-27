"""Mass-spring-damper example: simulate and log, optionally plot.

Run with --plot to also plot the log, or with --load --plot to plot an
existing log (e.g. one written by the C++ example_sim) without simulating.
"""

# std lib
import argparse
from pathlib import Path
import sys
import tempfile
from time import perf_counter

# 3rd party
import numpy as np

# local
import affine_mpc as ampc


def plot():
    if __name__ == "__main__":
        import plot_sim
    else:
        from . import plot_sim

    plot_sim.main()


def simulate():
    tmp = Path(tempfile.gettempdir())
    save_dir = tmp / "ampc_example"
    if (save_dir / "log.npz").exists():
        answer = input("Log file already exists. Overwrite? [Y/n]: ")
        if len(answer) == 0:
            answer = "y"
        if not answer.lower().startswith("y"):
            print("Not overwriting.")
            return

    msd_mpc = ampc.CondensedMPC(
        state_dim=2,
        input_dim=1,
        param=ampc.Parameterization.linearInterp(
            horizon_steps=10, num_control_points=3
        ),
        opts=ampc.Options(use_input_cost=True, slew_control_points=True),
    )

    A = np.array([[0, 1], [-0.6, -0.1]])
    B = np.array([0, 0.2])
    w = np.zeros(2)
    ts = 0.1
    msd_mpc.setModelContinuous2Discrete(A, B, w, ts)

    u_min = np.zeros(1)
    u_max = np.array([3.0])
    msd_mpc.setInputLimits(u_min, u_max)

    slew = np.ones(1)
    msd_mpc.setSlewRate(slew)

    Q_diag = np.array([1, 0.1])
    R_diag = np.array([1e-5])
    msd_mpc.setWeights(Q_diag, R_diag)

    x_ref = np.array([1.0, 0])
    u_ref = np.array([3.0])  # 3N force holds mass at 1m
    msd_mpc.setReferenceState(x_ref)
    msd_mpc.setReferenceInput(u_ref)

    if not msd_mpc.initializeSolver():
        raise RuntimeError("Failed to initialize solver")

    pred_stride = 1  # 1 logs every step of each solve horizon
    logger = ampc.MPCLogger(msd_mpc, save_dir, ts, pred_stride)
    logger.addMetadata("example_name", "mass_spring_damper")

    xk = np.zeros(2)

    t = 0.0
    tf = 10.0
    while t < tf:
        # The logger records the solve() timing from SolveInfo automatically.
        # loop_time is for timing your own loop, e.g. on hardware.
        start = perf_counter()
        status = msd_mpc.solve(xk)

        if status != ampc.SolveStatus.Success:
            print("Solver status:", status)

        uk = msd_mpc.getNextInput()
        logger.logStep(t, xk, loop_time=perf_counter() - start)

        # Simulate system
        xk = msd_mpc.propagateModel(xk, uk)
        t += ts

    print("Log file written to", save_dir / "log.npz")


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument(
        "--load",
        action="store_true",
        help="plot the existing log instead of simulating (use with --plot)",
    )
    parser.add_argument(
        "--plot",
        action="store_true",
        help="plot the log (needs matplotlib, abracatabra, and a Qt binding)",
    )
    args = parser.parse_args()
    if args.load and not args.plot:
        parser.error("--load only skips the simulation before plotting; add --plot")
    if not args.load:
        simulate()
    if args.plot:
        plot()


# Deprecated command names; each still works as before after a notice.


def _renamed(command):
    print(
        f"Note: {Path(sys.argv[0]).name} is deprecated and will be removed in a "
        f"future release; use '{command}' instead.",
        file=sys.stderr,
    )


def run_all():
    """affine-mpc-example: simulate and plot."""
    _renamed("affine-mpc-example-msd --plot")
    simulate()
    plot()


def run_example():
    """affine-mpc-example-sim: simulate only."""
    _renamed("affine-mpc-example-msd")
    simulate()


def plot_existing():
    """affine-mpc-example-plot: plot the existing log only."""
    _renamed("affine-mpc-example-msd --load --plot")
    plot()


if __name__ == "__main__":
    main()
