import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation
import os

# -------------------------------
# Model propagation functions
# -------------------------------

def propagate_simple(state, control, L, dt):
    """
    Propagate one time step using the simple kinematic bicycle model.
    Equations:
        x_dot   = v * cos(psi)
        y_dot   = v * sin(psi)
        psi_dot = v/L * tan(delta)
    """
    x, y, psi = state
    v, delta = control
    x_dot   = v * np.cos(psi)
    y_dot   = v * np.sin(psi)
    psi_dot = v / L * np.tan(delta)
    new_state = state + dt * np.array([x_dot, y_dot, psi_dot])
    return new_state

def propagate_improved(state, control, L, L_r, dt):
    """
    Propagate one time step using the improved kinematic bicycle model.
    In this model the vehicle’s center-of-mass is taken into account via a slip angle beta.
    
    Definitions:
        L = L_f + L_r, where L_f and L_r are the distances from the center-of-mass to the front and rear axles.
        beta = arctan((L_r / L) * tan(delta))
    
    Equations:
        x_dot   = v * cos(psi + beta)
        y_dot   = v * sin(psi + beta)
        psi_dot = (v*cos(beta)/L) * tan(delta)
    """
    x, y, psi = state
    v, delta = control
    beta = np.arctan((L_r / L) * np.tan(delta))
    x_dot   = v * np.cos(psi + beta)
    y_dot   = v * np.sin(psi + beta)
    psi_dot = (v * np.cos(beta) / L) * np.tan(delta)
    new_state = state + dt * np.array([x_dot, y_dot, psi_dot])
    return new_state

# -------------------------------
# Simulation using control inputs
# -------------------------------

def simulate_controls(u_controls, dt, L, L_r, initial_state):
    """
    Simulate the vehicle dynamics using the provided control inputs for both models.
    
    Parameters:
      - u_controls: (N,2) array of control inputs [v, delta]
      - dt: time step in seconds (10 Hz → dt = 0.1 s)
      - L: total wheelbase (L_f + L_r)
      - L_r: distance from center-of-mass to the rear axle (for the improved model)
      - initial_state: [x, y, psi]
    
    Returns:
      - simple_states: (N+1, 3) array of states from the simple model.
      - improved_states: (N+1, 3) array of states from the improved model.
    """
    N = u_controls.shape[0]
    simple_states = np.zeros((N+1, 3))
    improved_states = np.zeros((N+1, 3))
    simple_states[0] = initial_state
    improved_states[0] = initial_state
    for i in range(N):
        control = u_controls[i]
        simple_states[i+1] = propagate_simple(simple_states[i], control, L, dt)
        improved_states[i+1] = propagate_improved(improved_states[i], control, L, L_r, dt)
    return simple_states, improved_states

# -------------------------------
# Visualization functions
# -------------------------------

def plot_static_trajectories(simple_states, improved_states, x_refs=None):
    plt.figure(figsize=(10, 8))
    
    # Plot the simulated trajectories
    plt.plot(simple_states[:, 0], simple_states[:, 1], 'b-', label="Simple Model")
    plt.plot(improved_states[:, 0], improved_states[:, 1], 'r-', label="Improved Model")
    
    # Plot the reference trajectory if available
    if x_refs is not None and x_refs.shape[1] >= 2:
        plt.plot(x_refs[:, 0], x_refs[:, 1], 'g--', label="Reference Path")
    
    # Determine start and end markers.
    if x_refs is not None and x_refs.shape[0] > 0:
        start = x_refs[0]
        end = x_refs[-1]
    else:
        start = simple_states[0]
        end = simple_states[-1]
    
    # Draw star markers at the start and end.
    plt.plot(start[0], start[1], 'k*', markersize=12, label='Start')
    plt.plot(end[0], end[1], 'k*', markersize=12, label='End')
    
    plt.xlabel("X [m]")
    plt.ylabel("Y [m]")
    plt.title("Trajectories from Control Inputs and Reference")
    plt.legend()
    plt.grid(True)
    plt.axis('equal')
    plt.show()

def animate_simulation(simple_states, improved_states, dt, x_refs=None):
    """
    Create an animation that shows the evolution of the two simulated trajectories.
    Also plots the reference trajectory (if provided) as a static dashed line.
    For each time step, the current vehicle state is marked with an arrow indicating heading.
    """
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.set_title("Vehicle Trajectories")
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.grid(True)
    
    # Plot full trajectories for context.
    ax.plot(simple_states[:, 0], simple_states[:, 1], 'b--', label='Simple Model Path')
    ax.plot(improved_states[:, 0], improved_states[:, 1], 'r--', label='Improved Model Path')
    if x_refs is not None and x_refs.shape[1] >= 2:
        ax.plot(x_refs[:, 0], x_refs[:, 1], 'g--', label='Reference Path')
    
    # Draw static start and end markers.
    if x_refs is not None and x_refs.shape[0] > 0:
        start = x_refs[0]
        end = x_refs[-1]
    else:
        start = simple_states[0]
        end = simple_states[-1]
    ax.plot(start[0], start[1], 'k*', markersize=12, label='Start')
    ax.plot(end[0], end[1], 'k*', markersize=12, label='End')
    
    # Markers for the current positions (for animation)
    marker_simple, = ax.plot([], [], 'bo', markersize=8, label='Simple Current')
    marker_improved, = ax.plot([], [], 'ro', markersize=8, label='Improved Current')
    
    ax.legend()
    
    # Determine plot limits based on all trajectories
    all_x = np.hstack((simple_states[:, 0], improved_states[:, 0]))
    all_y = np.hstack((simple_states[:, 1], improved_states[:, 1]))
    if x_refs is not None:
        all_x = np.hstack((all_x, x_refs[:, 0]))
        all_y = np.hstack((all_y, x_refs[:, 1]))
    ax.set_xlim(all_x.min() - 1, all_x.max() + 1)
    ax.set_ylim(all_y.min() - 1, all_y.max() + 1)
    ax.set_aspect('equal', 'box')
    
    N = simple_states.shape[0]
    
    # For drawing arrows, keep track so that we can remove them each frame.
    arrow_artists = []
    
    def init():
        marker_simple.set_data([], [])
        marker_improved.set_data([], [])
        return marker_simple, marker_improved
    
    def update(frame):
        nonlocal arrow_artists
        # Remove previous arrows
        for artist in arrow_artists:
            artist.remove()
        arrow_artists = []
        
        # Get the current state for both models
        s_state = simple_states[frame]
        i_state = improved_states[frame]
        marker_simple.set_data(s_state[0], s_state[1])
        marker_improved.set_data(i_state[0], i_state[1])
        
        # Draw arrows to indicate heading.
        arrow_len = 0.5  # arrow length
        # For the simple model:
        s_dx = arrow_len * np.cos(s_state[2])
        s_dy = arrow_len * np.sin(s_state[2])
        arrow_s = ax.arrow(s_state[0], s_state[1], s_dx, s_dy, 
                           head_width=0.1, fc='blue', ec='blue')
        arrow_artists.append(arrow_s)
        # For the improved model:
        i_dx = arrow_len * np.cos(i_state[2])
        i_dy = arrow_len * np.sin(i_state[2])
        arrow_i = ax.arrow(i_state[0], i_state[1], i_dx, i_dy, 
                           head_width=0.1, fc='red', ec='red')
        arrow_artists.append(arrow_i)
        
        return marker_simple, marker_improved, arrow_s, arrow_i
    
    ani = animation.FuncAnimation(fig, update, frames=N, init_func=init,
                                  interval=dt*1000, blit=False, repeat=False)
    plt.show()

# -------------------------------
# Main simulation and visualization
# -------------------------------

def main():
    dt = 0.1  # 10 Hz simulation (0.1 s time step)
    
    # Get current directory (assumes files are in the same directory)
    current_dir = os.path.dirname(os.path.abspath(__file__))
    
    # Load control inputs from file (u_c.npy should be in the current directory)
    try:
        u_controls = np.load(os.path.join(current_dir, "u_c_simple.npy"))
    except FileNotFoundError:
        print("u_c.npy not found. Please ensure the file is in the current directory.")
        return

    # Load reference trajectory from x_refs.npy
    try:
        x_refs = np.load(os.path.join(current_dir, "x_refs.npy"))
    except FileNotFoundError:
        print("x_refs.npy not found. Reference trajectory will not be displayed.")
        x_refs = None

    # Choose vehicle geometry.
    # In your original code, the total wheelbase was L = 0.258.
    # For the improved model, we need to define L_f and L_r such that L = L_f + L_r.
    # For example, let L_r = 0.108 and L_f = 0.15.
    L = 0.258      # total wheelbase
    L_r = 0.129    # rear axle distance
    
    # Set the initial state.
    # If the reference trajectory (x_refs) is available, use its first entry.
    # Otherwise, default to [0, 0, 0].
    if x_refs is not None and x_refs.shape[0] > 0:
        initial_state = x_refs[0]
    else:
        initial_state = np.array([0.0, 0.0, 0.0])
    initial_state = np.array([1.55, 4.046, 3.07484587])
    
    # Run simulation for both models
    simple_states, improved_states = simulate_controls(u_controls, dt, L, L_r, initial_state)
    
    # Plot static trajectories (including reference path and start/end markers)
    plot_static_trajectories(simple_states, improved_states, x_refs)
    
    # Create an animation to visualize the evolving states (including reference path)
    animate_simulation(simple_states, improved_states, dt, x_refs)

if __name__ == "__main__":
    main()
