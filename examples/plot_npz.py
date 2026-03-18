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
    states = data['states'] # Shape (N, K, n)
    inputs = data['inputs'] # Shape (N, K, m) or (N, nc, m)
    t_pred = data['meta_t_pred']
    log_ctrl_pts = data['meta_log_control_points']
    
    n = params['state_dim']
    m = params['input_dim']
    
    # Extract current actual state
    is_2d = (states.ndim == 2)
    x_curr = states if is_2d else states[:, 0, :]
    u_curr = inputs if (inputs.ndim == 2) else inputs[:, 0, :]
    
    fig, ax = plt.subplots(n + m, 1, sharex=True, figsize=(10, 8))
    
    for i in range(n):
        ax[i].plot(time, x_curr[:, i], linewidth=3, label=f'Actual x_{i}')
        # Plot predictions
        if not is_2d:
            for j in range(0, len(time), max(1, len(time)//5)):
                ax[i].plot(time[j] + t_pred, states[j, :, i], 'r--', alpha=0.5)
        
        ax[i].set_ylabel(f'State {i}')
        ax[i].grid(True)
        ax[i].legend()
        
    for i in range(m):
        ax[n+i].step(time, u_curr[:, i], where='post', linewidth=3, label=f'Applied u_{i}')
        
        if not is_2d and not log_ctrl_pts:
            for j in range(0, len(time), max(1, len(time)//5)):
                ax[n+i].step(time[j] + t_pred, inputs[j, :, i], 'g--', where='post', alpha=0.5)
                
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
        import tempfile
        log_dir = os.path.join(tempfile.gettempdir(), "ampc_example")
        
    if os.path.exists(log_dir):
        print(f"Loading log from {log_dir}...")
        data, params = load_mpc_log(log_dir)
        plot_log(data, params)
    else:
        print(f"Log directory {log_dir} not found.")
        print("Run the 'example_sim' binary first to generate a log.")
