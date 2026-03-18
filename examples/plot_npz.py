import numpy as np
import os
import yaml
import matplotlib.pyplot as plt

def load_mpc_log(log_dir, log_name="log"):
    """
    Loads an MPC log (NPZ + YAML) and returns the data and parameters.
    """
    npz_path = os.path.join(log_dir, f"{log_name}.npz")
    yaml_path = os.path.join(log_dir, "params.yaml")
    
    data = np.load(npz_path)
    
    with open(yaml_path, 'r') as f:
        params = yaml.safe_load(f)
        
    return data, params

def plot_log(data, params):
    """
    Basic plot of states and inputs from the log.
    """
    time = data['time']
    x_pred = data['x_pred'] # Shape (N, T+1, n)
    u_pred = data['u_pred'] # Shape (N, T, m)
    
    n = params['n']
    m = params['m']
    
    # Plot current states (first element of each prediction)
    x_curr = x_pred[:, 0, :]
    
    fig, ax = plt.subplots(n + m, 1, sharex=True, figsize=(10, 8))
    
    for i in range(n):
        ax[i].plot(time, x_curr[:, i], label=f'x_{i}')
        ax[i].set_ylabel(f'State {i}')
        ax[i].grid(True)
        ax[i].legend()
        
    for i in range(m):
        ax[n+i].step(time, u_pred[:, 0, i], where='post', label=f'u_{i}')
        ax[n+i].set_ylabel(f'Input {i}')
        ax[n+i].set_xlabel('Time [s]')
        ax[n+i].grid(True)
        ax[n+i].legend()
        
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    import sys
    if len(sys.argv) > 1:
        log_dir = sys.argv[1]
    else:
        # Default to the example directory if it exists
        import tempfile
        log_dir = os.path.join(tempfile.gettempdir(), "ampc_example")
        
    if os.path.exists(log_dir):
        print(f"Loading log from {log_dir}...")
        data, params = load_mpc_log(log_dir)
        plot_log(data, params)
    else:
        print(f"Log directory {log_dir} not found.")
        print("Run the 'example_sim' binary first to generate a log.")
