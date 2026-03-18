import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
import tempfile

def main():
    save_dir = Path(tempfile.gettempdir()) / "ampc_example"
    
    npz_path = save_dir / "log.npz"
    if not npz_path.exists():
        print(f"Error: {npz_path} not found. Run 'example_sim' first.")
        return

    data = np.load(npz_path)
    
    time = data['time']
    states = data['states']  # (N, K, n) or (N, n)
    inputs = data['inputs']  # (N, K, m) or (N, nc, m) or (N, m)
    solve_times = data['solve_times'] # (N, 2)
    t_pred = data['meta_t_pred'] # (K,)
    log_ctrl_pts = data['meta_log_control_points']
    
    is_2d = (states.ndim == 2)
    x_curr = states if is_2d else states[:, 0, :]
    u_curr = inputs if (inputs.ndim == 2) else inputs[:, 0, :]

    fig, ax = plt.subplots(2, 1, sharex=True, figsize=(10, 8))
    
    # Plot state 0 (Position)
    ax[0].plot(time, x_curr[:, 0], linewidth=3, label="Actual State (x_0)")
    
    if not is_2d:
        for i in range(0, len(time), max(1, len(time)//5)):
            ax[0].plot(time[i] + t_pred, states[i, :, 0], 'r--', alpha=0.5, 
                       label="Predicted" if i == 0 else "")
        
    ax[0].set_ylabel("position (m)")
    ax[0].grid(True)
    ax[0].legend()

    # Plot state 1 (Velocity)
    ax[1].plot(time, x_curr[:, 1], linewidth=3, label="Actual State (x_1)")
    if not is_2d:
        for i in range(0, len(time), max(1, len(time)//5)):
            ax[1].plot(time[i] + t_pred, states[i, :, 1], 'r--', alpha=0.5)
        
    ax[1].set_ylabel("velocity (m/s)")
    ax[1].set_xlabel("time (s)")
    ax[1].grid(True)
    ax[1].legend()

    # Plot Inputs
    fig2, ax2 = plt.subplots(1, 1, figsize=(10, 4))
    ax2.step(time, u_curr[:, 0], where='post', linewidth=3, label="Applied Input u_0")
    
    if not is_2d and not log_ctrl_pts:
        for i in range(0, len(time), max(1, len(time)//5)):
            ax2.step(time[i] + t_pred, inputs[i, :, 0], 'g--', where='post', alpha=0.5,
                     label="Predicted Input" if i == 0 else "")

    ax2.set_ylabel("force (N)")
    ax2.set_xlabel("time (s)")
    ax2.grid(True)
    ax2.legend()

    # Plot Solve Times
    fig3, ax3 = plt.subplots(1, 1, figsize=(10, 4))
    ax3.plot(time, solve_times[:, 1] * 1000.0, label="OSQP Solve Time")
    ax3.set_ylabel("solve time (ms)")
    ax3.set_xlabel("sim time (s)")
    ax3.grid(True)
    ax3.legend()

    plt.show()

if __name__ == "__main__":
    main()
