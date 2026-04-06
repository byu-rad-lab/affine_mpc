# std lib
from pathlib import Path
import tempfile
from time import perf_counter

# 3rd party
import numpy as np

# local
import affine_mpc as ampc


def plot_example():
    if __name__ == "__main__":
        import plot_sim
    else:
        from . import plot_sim

    plot_sim.main()


def run_example():
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
        start = perf_counter()
        status = msd_mpc.solve(xk)
        elapsed = perf_counter() - start

        if status != ampc.SolveStatus.Success:
            print("Solver status:", status)

        uk = msd_mpc.getNextInput()
        logger.logStep(t, xk, elapsed)

        # Simulate system
        xk = msd_mpc.propagateModel(xk, uk)
        t += ts

    print("Log file written to", save_dir / "log.npz")


def main():
    run_example()
    plot_example()


if __name__ == "__main__":
    main()
