import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
import tempfile
import os

def main():
    # Matches the default directory in examples/sim.cpp
    save_dir = Path(tempfile.gettempdir()) / "ampc_example"
    
    npz_path = save_dir / "log.npz"
    if not npz_path.exists():
        print(f"Error: {npz_path} not found. Run 'example_sim' first.")
        return

    # Load new NPZ format
    data = np.load(npz_path)
    
    time = data['time']
    x_pred = data['x_pred']  # (N, T+1, n)
    u_pred = data['u_pred']  # (N, T, m)
    solve_times = data['solve_times'] # (N, 2)
    
    # x_pred[:, 0, :] contains the state at each step (x0)
    # x_pred[:, 1, :] contains the first predicted state (x1)
    states = x_pred[:, 0, :]

    fig, ax = plt.subplots(2, 1, sharex=True, figsize=(10, 8))
    
    # Plot state 0 (Position)
    ax[0].plot(time, states[:, 0], label="Actual State")
    ax[0].set_ylabel("position (m)")
    ax[0].grid(True)
    ax[0].legend()

    # Plot state 1 (Velocity)
    ax[1].plot(time, states[:, 1], label="Actual State")
    ax[1].set_ylabel("velocity (m/s)")
    ax[1].set_xlabel("time (s)")
    ax[1].grid(True)
    ax[1].legend()

    # Plot Inputs
    fig2, ax2 = plt.subplots(1, 1, figsize=(10, 4))
    # Plot the first input of each horizon
    ax2.step(time, u_pred[:, 0, 0], where='post', label="Input u_0")
    ax2.set_ylabel("force (N)")
    ax2.set_xlabel("time (s)")
    ax2.grid(True)
    ax2.legend()

    # Plot Solve Times
    fig3, ax3 = plt.subplots(1, 1, figsize=(10, 4))
    # solve_times[:, 1] is the OSQP solver time
    ax3.plot(time, solve_times[:, 1] * 1000.0, label="OSQP Solve Time")
    ax3.set_ylabel("solve time (ms)")
    ax3.set_xlabel("sim time (s)")
    ax3.grid(True)
    ax3.legend()

    plt.show()

if __name__ == "__main__":
    main()
