import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
import tempfile
import os

def main():
    save_dir = Path(tempfile.gettempdir()) / "ampc_example"
    
    npz_path = save_dir / "log.npz"
    if not npz_path.exists():
        print(f"Error: {npz_path} not found. Run 'example_sim' first.")
        return

    data = np.load(npz_path)
    
    time = data['time']
    x_curr = data['x_curr']  # (N, n)
    u_curr = data['u_curr']  # (N, m)
    solve_times = data['solve_times'] # (N, 2)
    
    # x_pred shape: (N, T, n)
    # u_pred shape: (N, T, m)

    fig, ax = plt.subplots(2, 1, sharex=True, figsize=(10, 8))
    
    ax[0].plot(time, x_curr[:, 0], label="Actual State (x_0)")
    ax[0].set_ylabel("position (m)")
    ax[0].grid(True)
    ax[0].legend()

    ax[1].plot(time, x_curr[:, 1], label="Actual State (x_1)")
    ax[1].set_ylabel("velocity (m/s)")
    ax[1].set_xlabel("time (s)")
    ax[1].grid(True)
    ax[1].legend()

    fig2, ax2 = plt.subplots(1, 1, figsize=(10, 4))
    ax2.step(time, u_curr[:, 0], where='post', label="Applied Input u_0")
    ax2.set_ylabel("force (N)")
    ax2.set_xlabel("time (s)")
    ax2.grid(True)
    ax2.legend()

    fig3, ax3 = plt.subplots(1, 1, figsize=(10, 4))
    ax3.plot(time, solve_times[:, 1] * 1000.0, label="OSQP Solve Time")
    ax3.set_ylabel("solve time (ms)")
    ax3.set_xlabel("sim time (s)")
    ax3.grid(True)
    ax3.legend()

    plt.show()

if __name__ == "__main__":
    main()
