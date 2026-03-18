import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path


def main():
    save_dir = Path("/tmp/ampc_example")
    time = np.loadtxt(save_dir / "time.txt")
    states = np.loadtxt(save_dir / "states.txt")
    ref_states = np.loadtxt(save_dir / "ref_states.txt")
    inputs = np.loadtxt(save_dir / "inputs.txt")
    solve_times = np.loadtxt(save_dir / "solve_times.txt")

    fig, ax = plt.subplots(2, sharex=True)
    ax[0].plot(time[:, 0], ref_states[:, 0], "r--", label="ref_state")
    ax[0].plot(time[:, 0], states[:, 0], label="state")
    ax[0].set_ylabel("position (m)")

    ax[1].plot(time[:, 0], ref_states[:, 1], "r--", label="ref_state")
    ax[1].plot(time[:, 0], states[:, 1], label="state")
    ax[1].set_ylabel("velocity (m/s)")
    ax[1].set_xlabel("time (s)")

    fig, ax = plt.subplots(1)
    ax.plot(time[:, 0], inputs[:, 0], drawstyle="steps-post")
    ax.set_ylabel("force (N)")

    fig, ax = plt.subplots(1)
    ax.plot(time[:, 0], solve_times[:, 1])
    ax.set_ylabel("solve time (s)")
    ax.set_xlabel("sim time (s)")

    plt.show()


if __name__ == "__main__":
    main()
