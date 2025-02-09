#!/usr/bin/env python3
"""
Example: MPC for a kinematic bicycle using acados
-----------------------------------------------
We use:
   • State:    x = [x, y, psi]
   • Control:  u = [v, delta]
with dynamics:
   ẋ = v cos(psi),   ẏ = v sin(psi),   psi̇ = (v/L)*tan(delta)
and a cost function that penalizes
   – deviation of [x,y,psi] from a reference (i.e. path and yaw error),
   – deviation of u from a reference [v_ref, delta_ref],
   – and the change in u (i.e. u - u_last).
This is implemented via a least–squares cost with
   h(x,u) = [x-x_ref; y-y_ref; psi-psi_ref; v-v_ref; delta-delta_ref; v-v_last; delta-delta_last]
whose weighted square norm is minimized.
"""

import casadi as ca
import numpy as np
from acados_template import AcadosOcp, AcadosModel, AcadosOcpSolver
import matplotlib.pyplot as plt

def create_kinematic_bicycle_model(L=2.5):
    # Define the state: (x, y, psi)
    x = ca.SX.sym('x')
    y = ca.SX.sym('y')
    psi = ca.SX.sym('psi')
    X = ca.vertcat(x, y, psi)
    
    # Define the control: (v, delta)
    v = ca.SX.sym('v')
    delta = ca.SX.sym('delta')
    U = ca.vertcat(v, delta)
    
    # Define the explicit dynamics:
    x_dot   = v * ca.cos(psi)
    y_dot   = v * ca.sin(psi)
    psi_dot = (v / L) * ca.tan(delta)
    Xdot    = ca.vertcat(x_dot, y_dot, psi_dot)
    
    # Create the acados model:
    model = AcadosModel()
    model.name = 'kinematic_bicycle'
    model.x = X
    model.u = U
    model.f_expl_expr = Xdot

    # Define a parameter vector (used only in the cost) of length 7:
    # p = [x_ref, y_ref, psi_ref, v_ref, delta_ref, v_last, delta_last]
    p = ca.SX.sym('p', 7)
    model.p = p

    return model

def create_ocp_solver(T, N, L=2.5):
    """
    Create and return an acados OCP solver for our kinematic bicycle.
      T: sampling time
      N: number of shooting intervals
      L: wheelbase
    """
    model = create_kinematic_bicycle_model(L)
    ocp = AcadosOcp()
    ocp.model = model

    # Set default parameter values for model.p (7-dimensional)
    ocp.parameter_values = np.zeros(7)

    ocp.dims.N = N
    ocp.solver_options.tf = T * N

    # Use nonlinear least-squares for both stage and terminal cost.
    ocp.cost.cost_type   = 'NONLINEAR_LS'
    ocp.cost.cost_type_e = 'NONLINEAR_LS'
    
    # Define the stage cost output h(x,u)
    # h_stage = [ x - x_ref;
    #             y - y_ref;
    #             psi - psi_ref;
    #             v - v_ref;
    #             delta - delta_ref;
    #             v - v_last;
    #             delta - delta_last ]
    h_expr = ca.vertcat( model.x - model.p[0:3],
                         model.u - model.p[3:5],
                         model.u - model.p[5:7] )
    
    # Assign the cost expressions in both ocp.cost and the model.
    ocp.cost.y_expr = h_expr
    ocp.model.cost_y_expr = h_expr  # Ensure the model has the stage cost expression
    
    # Terminal cost: here we penalize only the state error
    terminal_expr = model.x - model.p[0:3]
    ocp.cost.y_expr_e = terminal_expr
    ocp.model.cost_y_expr_e = terminal_expr  # Ensure the model has the terminal cost expression

    # *** NEW: Set the reference outputs for the cost ***
    # For stage cost (7-dimensional) and terminal cost (3-dimensional):
    ocp.cost.yref   = np.zeros(7)
    ocp.cost.yref_e = np.zeros(3)

    # Choose weighting matrices:
    # For stage cost, the ordering is: [x_err, y_err, psi_err, v_err, delta_err, v_rate, delta_rate]
    Q_pos   = 10.0   # position error weight
    Q_yaw   = 5.0    # yaw error weight
    Q_v     = 1.0    # speed error weight
    Q_delta = 1.0    # steering error weight
    R_v     = 0.1    # penalize change in speed
    R_delta = 0.1    # penalize change in steering
    W = np.diag([Q_pos, Q_pos, Q_yaw, Q_v, Q_delta, R_v, R_delta])
    ocp.cost.W = W

    # Terminal cost weight (for state error only, dimension 3)
    ocp.cost.W_e = np.diag([Q_pos, Q_pos, Q_yaw])

    # Impose bounds on the control:
    # (Here v is limited between 0 and 20 m/s and delta between -0.5 and 0.5 radians.)
    ocp.constraints.lbu   = np.array([0.0, -0.5])
    ocp.constraints.ubu   = np.array([20.0, 0.5])
    ocp.constraints.idxbu = np.array([0, 1])
    
    # The initial condition will be provided at runtime.
    ocp.constraints.x0 = np.zeros(3)
    
    # Set some solver options
    ocp.solver_options.integrator_type = 'ERK'
    ocp.solver_options.nlp_solver_type = 'SQP_RTI'
    ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
    
    # Create and return the acados solver
    acados_solver = AcadosOcpSolver(ocp, json_file='acados_ocp.json')
    return acados_solver


# ==============================
# Simulation and Plotting
# ==============================
if __name__ == '__main__':
    # Simulation parameters
    T = 0.1         # sampling time (seconds)
    N = 20          # prediction horizon (number of shooting intervals)
    sim_steps = 200  # total simulation steps
    L = 2.5         # wheelbase of the bicycle

    # Create the MPC solver instance
    solver = create_ocp_solver(T, N, L)

    # Initial state: [x, y, psi]
    x_current = np.array([0.0, 0.0, 0.0])
    # Initialize "last control" (for rate penalty)
    v_last = 2.0      # start with the reference speed
    delta_last = 0.0

    # Reference path parameters (sine wave)
    v_ref = 2.0          # constant forward speed (m/s)
    amplitude = 1.0      # amplitude of the sine wave (m)
    frequency = 0.2      # frequency factor (rad/m)

    # Lists to store simulation data for plotting
    x_traj = [x_current[0]]
    y_traj = [x_current[1]]
    psi_traj = [x_current[2]]
    t_traj = [0.0]

    # Closed-loop simulation loop
    for step in range(sim_steps):
        # Set the current state as the initial condition for the MPC problem.
        solver.set(0, "x", x_current)

        # For each stage in the horizon, update the reference parameters.
        # Here, we assume that the desired x position is given by
        #   x_ref = v_ref * t_total
        # and the desired y position is
        #   y_ref = amplitude * sin(frequency * x_ref)
        # with psi_ref as the tangent angle.
        for i in range(N):
            t_ref = step * T + i * T  # future time
            x_ref_val = v_ref * t_ref
            y_ref_val = amplitude * np.sin(frequency * x_ref_val)
            # Desired heading: derivative of y_ref with respect to x_ref:
            psi_ref_val = np.arctan(amplitude * frequency * np.cos(frequency * x_ref_val))
            # Build the parameter vector:
            # p = [x_ref, y_ref, psi_ref, v_ref, delta_ref, v_last, delta_last]
            p_stage = np.array([x_ref_val, y_ref_val, psi_ref_val, v_ref, 0.0, v_last, delta_last])
            solver.set(i, "p", p_stage)

        # Terminal stage (only state error is penalized)
        t_ref = step * T + N * T
        x_ref_val = v_ref * t_ref
        y_ref_val = amplitude * np.sin(frequency * x_ref_val)
        psi_ref_val = np.arctan(amplitude * frequency * np.cos(frequency * x_ref_val))
        p_term = np.array([x_ref_val, y_ref_val, psi_ref_val, 0.0, 0.0, 0.0, 0.0])
        solver.set(N, "p", p_term)

        # Solve the MPC problem
        status = solver.solve()
        if status != 0:
            print("Solver failed at step", step, "with status", status)
            break

        # Get the first control action from the solution
        sol = solver.get(0, "u")
        v = sol[0]
        delta = sol[1]

        # Simulate the system forward one time step using Euler integration.
        x, y, psi = x_current
        x_dot = v * np.cos(psi)
        y_dot = v * np.sin(psi)
        psi_dot = (v / L) * np.tan(delta)
        x_next = x + T * x_dot
        y_next = y + T * y_dot
        psi_next = psi + T * psi_dot
        x_current = np.array([x_next, y_next, psi_next])

        # Update "last control" values for the next iteration.
        v_last = v
        delta_last = delta

        # Save the state for plotting.
        x_traj.append(x_current[0])
        y_traj.append(x_current[1])
        psi_traj.append(x_current[2])
        t_traj.append((step + 1) * T)

    # Plot the simulation results.
    plt.figure(figsize=(8, 4))
    plt.plot(x_traj, y_traj, 'b-', label='Robot trajectory')
    # Generate a dense set of reference points for plotting the sine wave path.
    x_ref_path = np.linspace(0, v_ref * sim_steps * T, 1000)
    y_ref_path = amplitude * np.sin(frequency * x_ref_path)
    plt.plot(x_ref_path, y_ref_path, 'r--', label='Reference path')
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.title("MPC Tracking of Sine Wave Reference Path")
    plt.legend()
    plt.grid(True)
    plt.show()
