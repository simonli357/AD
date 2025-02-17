#!/usr/bin/env python3
# coding=UTF-8

from path2 import Path
import os
import sys

from acados_template import AcadosOcp, AcadosOcpSolver, AcadosSimSolver

import numpy as np
import scipy.linalg

from draw2 import Draw_MPC_tracking
import casadi as ca
from acados_template import AcadosModel
import time
import yaml
import argparse

class Optimizer(object):
    def __init__(self, x0 = None):
        self.solver, self.integrator, self.T, self.N, self.t_horizon = self.create_solver()

        name = 'run107'
        self.path = Path(v_ref = self.v_ref, N = self.N, T = self.T, name=name, x0=x0)
        self.waypoints_x_global = self.path.waypoints_x
        self.waypoints_y_global = self.path.waypoints_y
        self.global_waypoints = np.column_stack((self.path.waypoints_x, self.path.waypoints_y))
        
        self.num_waypoints = self.path.num_waypoints
        self.wp_normals = self.path.wp_normals
        self.kappa = self.path.kappa
        self.density = self.path.density
        self.rdb_circumference = 3.95
        self.state_refs_global = self.path.state_refs  
        self.input_refs = self.path.input_refs
        np.savetxt('state_refs_global.txt', self.state_refs_global, fmt='%.3f')
        self.state_refs = self.convert_state_refs_to_frenet(self.state_refs_global)
        # save as txt with 3 decimals
        np.savetxt('state_refs_frenet.txt', self.state_refs, fmt='%.3f')
        # exit()
        self.waypoints_x = self.state_refs[:, 0]
        self.waypoints_y = self.state_refs[:, 1]


        self.counter = 0
        self.target_waypoint_index = 0
        self.last_waypoint_index = 0
        self.count = 0
        density = 1/abs(self.v_ref)/self.T
        self.region_of_acceptance = 0.05/10*density * 2*1.5
        self.last_u = None
        self.t0 = 0
        
        self.init_state = x0 if x0 is not None else self.state_refs[0]
        self.update_current_state(*self.init_state)  # assume update_current_state(s, d, epsi) sets self.current_state
        
        self.init_state = self.current_state.copy()
        self.u0 = np.zeros((self.N, 2))
        self.next_trajectories = np.tile(self.init_state, self.N+1).reshape(self.N+1, -1) # set the initial state as the first trajectories for the robot
        self.next_controls = np.zeros((self.N, 2))
        self.next_states = np.zeros((self.N+1, 3))
        self.x_c = [] # contains for the history of the state
        self.u_c = []
        self.u_cc = []
        self.t_c = [self.t0] # for the time
        self.xx = []
        self.x_refs = []
        self.x_errors = []
        self.y_errors = []
        self.yaw_errors = []
        ## start MPC
        self.mpciter = 0
        self.start_time = time.time()
        self.index_t = []
        filepath = os.path.dirname(os.path.abspath(__file__))
        self.export_fig = os.path.join(filepath+'/gifs_acados',name + '_N'+str(self.N) + '_vref'+str(self.v_ref) 
                                       + '_T'+str(self.T))
        
    def global_to_frenet(self, x, y, yaw, path_points, s_cum):
        """
        Converts a single state in global coordinates (x, y, yaw) to Frenet coordinates (s, d, epsi)
        given the reference path (path_points) and its cumulative arc-length (s_cum).
        """
        # Compute distances from the point to each waypoint.
        distances = np.linalg.norm(path_points - np.array([x, y]), axis=1)
        idx = np.argmin(distances)

        # Determine the segment for projection:
        if idx >= len(path_points) - 1:
            idx = len(path_points) - 2
        p1 = path_points[idx]
        p2 = path_points[idx + 1]

        # Compute the unit vector along the segment.
        seg = p2 - p1
        seg_length = np.linalg.norm(seg)
        if seg_length < 1e-6:
            seg_unit = np.array([1.0, 0.0])
        else:
            seg_unit = seg / seg_length

        # Project the point onto the segment.
        vec = np.array([x, y]) - p1
        proj_len = np.dot(vec, seg_unit)
        proj_point = p1 + proj_len * seg_unit

        # The along-track coordinate s is the cumulative distance to the projection.
        s_val = s_cum[idx] + proj_len

        # Compute the lateral deviation d.
        # First, compute a normal vector (rotate seg_unit by 90 degrees).
        normal = np.array([-seg_unit[1], seg_unit[0]])
        d_val = np.dot(vec, normal)

        # Compute the reference heading (the tangent of the segment).
        theta_path = np.arctan2(seg_unit[1], seg_unit[0])
        # Heading error (epsi): difference between the vehicle yaw and the path tangent.
        epsi = yaw - theta_path
        # Normalize epsi to be between -pi and pi.
        epsi = np.arctan2(np.sin(epsi), np.cos(epsi))
        return s_val, d_val, epsi

    def convert_state_refs_to_frenet(self, state_refs_global):
        """
        Converts an array of global state references (each [x, y, yaw])
        into Frenet coordinates [s, d, epsi] using the reference path.
        """
        # Build a (N x 2) array for the reference path from the waypoints.
        # Here we assume the waypoints are given in order.
        path_points = np.column_stack((self.waypoints_x_global, self.waypoints_y_global))
        # Compute cumulative arc-length along the path.
        diffs = np.diff(path_points, axis=0)
        ds = np.linalg.norm(diffs, axis=1)
        s_cum = np.concatenate(([0], np.cumsum(ds)))

        # Convert each global reference state.
        state_refs_frenet = []
        for ref in state_refs_global:
            x, y, yaw = ref
            s_val, d_val, epsi = self.global_to_frenet(x, y, yaw, path_points, s_cum)
            state_refs_frenet.append([s_val, d_val, epsi])
        return np.array(state_refs_frenet)  
    
    def frenet_to_global(self, frenet_state):
        """
        Convert a single Frenet state [s, d, epsi] back to global [x, y, yaw].
        This function uses the original (global) reference path stored in self.global_waypoints.
        """
        s, d, epsi = frenet_state
        global_points = self.global_waypoints  # Nx2 array of global [x,y]
        # Compute cumulative arc-length along the global reference path.
        diffs = np.diff(global_points, axis=0)
        ds = np.linalg.norm(diffs, axis=1)
        s_cum = np.concatenate(([0], np.cumsum(ds)))
        # Find the segment in which s lies.
        if s >= s_cum[-1]:
            i = len(s_cum) - 2
            s_local = s - s_cum[i]
        else:
            i = np.searchsorted(s_cum, s) - 1
            if i < 0:
                i = 0
            s_local = s - s_cum[i]
        p1 = global_points[i]
        p2 = global_points[i + 1]
        seg = p2 - p1
        seg_length = np.linalg.norm(seg)
        seg_unit = seg / seg_length if seg_length >= 1e-6 else np.array([1.0, 0.0])
        pos_ref = p1 + s_local * seg_unit
        # Compute the normal vector to the segment.
        normal = np.array([-seg_unit[1], seg_unit[0]])
        # Global position is the reference plus lateral offset.
        x_global = pos_ref[0] + d * normal[0]
        y_global = pos_ref[1] + d * normal[1]
        # The global yaw is the reference segment heading plus the heading error.
        theta_ref = np.arctan2(seg_unit[1], seg_unit[0])
        yaw_global = theta_ref + epsi
        yaw_global = np.arctan2(np.sin(yaw_global), np.cos(yaw_global))
        return np.array([x_global, y_global, yaw_global])

    def convert_traj_frenet_to_global(self, traj_frenet):
        """
        Convert an array (trajectory) of Frenet states (each [s,d,epsi]) to global states.
        """
        return np.array([self.frenet_to_global(state) for state in traj_frenet])
    
    def create_solver(self, config_path='config/mpc_config25.yaml'):
        model = AcadosModel() #  ca.types.SimpleNamespace()
        # control inputs
        v = ca.SX.sym('v')
        delta = ca.SX.sym('delta')
        controls = ca.vertcat(v, delta)
        # model states
        s_f = ca.SX.sym('s') # distance along the path
        d_f = ca.SX.sym('d') # lateral deviation from the path
        epsi = ca.SX.sym('epsi') # heading error    
        states = ca.vertcat(s_f, d_f, epsi)
        
        self.L = 0.258
        # --- Define curvature as an extra parameter ---
        kappa = ca.SX.sym('kappa')
        
        # --- Define Frenet dynamics ---
        # ds/dt = v * cos(epsi) / (1 - kappa*d)
        ds = v * ca.cos(epsi) / (1 - kappa * d_f)
        # dd/dt = v * sin(epsi)
        dd = v * ca.sin(epsi)
        # depsi/dt = (v / L)*tan(delta) - kappa * ds/dt
        depsi = v / self.L * ca.tan(delta) - kappa * ds

        rhs = ca.vertcat(ds, dd, depsi)

        f = ca.Function('f', [states, controls, kappa], [rhs],
                        ['state', 'control_input', 'curvature'], ['rhs'])
        x_dot = ca.SX.sym('x_dot', rhs.size1())
        f_impl = x_dot - f(states, controls, kappa)

        model.f_expl_expr = f(states, controls, kappa)
        model.f_impl_expr = f_impl
        model.x = states
        model.xdot = x_dot
        model.u = controls
        model.p = ca.vertcat(kappa)

        current_path = os.path.dirname(os.path.realpath(__file__))
        path = os.path.join(current_path, config_path)
        with open(path, 'r') as f:
            config = yaml.safe_load(f)
        model.name = config['name']
        T = config['T']
        N = config['N']
        constraint_name = 'constraints'
        cost_name = 'costs'
        t_horizon = T * N

        # constraints
        self.v_max = config[constraint_name]['v_max']
        self.v_min = config[constraint_name]['v_min']
        self.delta_max = config[constraint_name]['delta_max']
        self.delta_min = config[constraint_name]['delta_min']
        
        self.x_min = config[constraint_name]['x_min']
        self.x_max = config[constraint_name]['x_max']
        self.y_min = config[constraint_name]['y_min']
        self.y_max = config[constraint_name]['y_max']
        self.s_min = -10.0
        self.s_max = 10.0
        self.d_min = -10.0
        self.d_max = 10.0
        
        self.v_ref = config[constraint_name]['v_ref']
        # self.x_cost = config[cost_name]['x_cost']
        # self.y_cost = config[cost_name]['y_cost']
        # self.yaw_cost = config[cost_name]['yaw_cost']
        self.s_cost = 2.0
        self.d_cost = 2.0
        self.epsi_cost = 1.0
        self.v_cost = config[cost_name]['v_cost']
        self.steer_cost = config[cost_name]['steer_cost']
        self.delta_v_cost = config[cost_name]['delta_v_cost']
        self.delta_steer_cost = config[cost_name]['delta_steer_cost']
        self.costs = np.array([self.s_cost, self.d_cost, self.epsi_cost, self.v_cost, self.steer_cost, self.delta_v_cost, self.delta_steer_cost])
        Q = np.array([[self.s_cost,    0.0,          0.0],
                      [0.0,         self.d_cost,    0.0],
                      [0.0,          0.0,       self.epsi_cost]])
        R = np.array([[self.v_cost, 0.0],
                      [0.0,        self.steer_cost]])

        nx = model.x.size()[0]
        nu = model.u.size()[0]
        ny = nx + nu
        n_params = model.p.size()[0]

        os.chdir(os.path.dirname(os.path.realpath(__file__)))
        acados_source_path = os.environ['ACADOS_SOURCE_DIR']
        sys.path.insert(0, acados_source_path)
        ocp = AcadosOcp()
        ocp.acados_include_path = acados_source_path + '/include'
        ocp.acados_lib_path = acados_source_path + '/lib'
        ocp.model = model
        ocp.dims.N = N
        ocp.solver_options.tf = t_horizon
        ocp.dims.np = n_params
        ocp.parameter_values = np.zeros(n_params)

        ocp.cost.cost_type = 'LINEAR_LS'
        ocp.cost.cost_type_e = 'LINEAR_LS'
        ocp.cost.W = scipy.linalg.block_diag(Q, R)
        ocp.cost.W_e = Q
        ocp.cost.Vx = np.zeros((ny, nx))
        ocp.cost.Vx[:nx, :nx] = np.eye(nx)
        ocp.cost.Vu = np.zeros((ny, nu))
        ocp.cost.Vu[-nu:, -nu:] = np.eye(nu)
        ocp.cost.Vx_e = np.eye(nx)

        ocp.constraints.lbu = np.array([self.v_min, self.delta_min])
        ocp.constraints.ubu = np.array([self.v_max, self.delta_max])
        ocp.constraints.idxbu = np.array([0, 1])
        # ocp.constraints.lbx = np.array([self.x_min, self.y_min])
        # ocp.constraints.ubx = np.array([self.x_max, self.y_max])
        ocp.constraints.lbx = np.array([self.s_min, self.d_min])
        ocp.constraints.ubx = np.array([self.s_max, self.d_max])
        ocp.constraints.idxbx = np.array([0, 1])

        x_ref = np.zeros(nx)
        u_ref = np.zeros(nu)
        ocp.constraints.x0 = x_ref
        ### 0--N-1
        ocp.cost.yref = np.concatenate((x_ref, u_ref))
        ### N
        ocp.cost.yref_e = x_ref # yref_e means the reference for the last stage

        ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
        ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
        ocp.solver_options.integrator_type = 'ERK'
        ocp.solver_options.print_level = 0
        ocp.solver_options.nlp_solver_type = 'SQP_RTI'

        json_file = os.path.join('./'+model.name+'_acados_ocp.json')
        solver = AcadosOcpSolver(ocp, json_file=json_file)
        integrator = AcadosSimSolver(ocp, json_file=json_file)
        return solver, integrator, T, N, t_horizon
    
    def update_and_solve(self):
        # self.target_waypoint_index = self.find_next_waypoint()
        self.target_waypoint_index +=1
        print("idx: {}, ref: {}, current: {}".format(
            self.target_waypoint_index,
            np.array2string(self.state_refs[self.target_waypoint_index], precision=3, separator=', ', suppress_small=True),
            np.array2string(self.current_state, precision=3, separator=', ', suppress_small=True)
        ))
        idx = self.target_waypoint_index
        self.next_trajectories = self.state_refs[idx:idx + self.N + 1]
        self.next_controls = self.input_refs[idx:idx + self.N]
        xs = self.state_refs[idx]
        self.solver.set(self.N, 'yref', xs)
        for j in range(self.N):
            if self.target_waypoint_index+j >= self.state_refs.shape[0]:
                self.solver.set(j, 'yref', np.concatenate((self.state_refs[-1], np.zeros(2))))
            else:
                self.solver.set(j, 'yref', np.concatenate((self.next_trajectories[j], self.next_controls[j])))# 设置当前循环x0 (stage 0)
        if self.last_u is None:
            self.last_u = np.zeros(2)
        self.last_u[0] = self.next_controls[0, 0]
        self.solver.set(0, 'yref', np.concatenate((self.next_trajectories[j], self.last_u)))
        self.solver.set(0, 'lbx', self.current_state)
        self.solver.set(0, 'ubx', self.current_state)
        status = self.solver.solve()
        if status != 0 :
            print('ERROR!!! acados acados_ocp_solver returned status {}. Exiting.'.format(status))
            return None
        next_u = self.solver.get(0, 'u')
        self.counter += 1
        self.last_u = next_u
        return next_u
    def integrate_next_states(self, u_res=None):
        self.integrator.set('x', self.current_state)
        self.integrator.set('u', u_res)
        status_s = self.integrator.solve()
        if status_s != 0:
            raise Exception('acados integrator returned status {}. Exiting.'.format(status_s))
        self.current_state = self.integrator.get('x')
        self.t0 += self.T
    
    # def update_current_state(self, x, y, yaw):
    #     if self.target_waypoint_index < len(self.state_refs):
    #         ref_yaw = self.state_refs[self.target_waypoint_index, 2]
    #         while yaw - ref_yaw > np.pi:
    #             yaw -= 2 * np.pi
    #         while yaw - ref_yaw < -np.pi:
    #             yaw += 2 * np.pi
    #     self.current_state = np.array([x, y, yaw])
    
    def update_current_state(self, s, d, epsi):
        """
        Example method to update the current state.
        In your implementation, ensure that self.current_state is in [s, d, epsi].
        """
        self.current_state = np.array([s, d, epsi])
        
    def find_closest_waypoint(self):
        """
        In Frenet coordinates, self.current_state = [s, d, epsi].
        self.waypoints_x and self.waypoints_y are the s and d coordinates of the reference path.
        We compute the Euclidean distance between [s, d] of the current state and each waypoint.
        """
        # Build an array of reference points in the Frenet plane.
        ref_points = np.column_stack((self.waypoints_x, self.waypoints_y))
        # Extract the [s, d] from the current state.
        current_sd = self.current_state[:2]
        # Compute distances in the (s,d) plane.
        distances = np.linalg.norm(ref_points - current_sd, axis=1)
        index = np.argmin(distances)
        return index, distances[index]
    def find_next_waypoint(self):
        """
        Find the next waypoint index based on the current state.
        This function mimics the provided C++ version:
        - It first finds the closest waypoint (using find_closest_waypoint).
        - If the distance to the closest waypoint is greater than 1.2 m,
            it prints a warning and re-searches using an updated minimum index.
        - It then uses a counter (reset every 8 calls) to either set the target
            to closest + lookahead or simply increments the target waypoint index.
        
        Returns:
        The next waypoint index (an integer, bounded by the number of waypoints).
        """
        # Make sure self.count is initialized (e.g., in __init__: self.count = 0)
        closest_idx, distance_to_current = self.find_closest_waypoint()

        if distance_to_current > 1.2:
            print("WARNING: find_next_waypoint(): distance to closest waypoint is too large:", distance_to_current)
            # Update min_index as in C++: max(closest_idx - distance_to_current * density * 1.2, 0)
            min_index = int(max(closest_idx - distance_to_current * self.density * 1.2, 0))
            closest_idx, distance_to_current = self.find_closest_waypoint(self.current_state, min_index, -1)

        # In C++ a static counter is used: if count >= 8, then target = closest_idx + lookahead,
        # else target = target_waypoint_index + 1, and then count is incremented.
        if self.count >= 8:
            # In C++ the lookahead is set to 1 if v_ref > 0.375, else default lookahead is 1.
            lookahead = 1 if self.v_ref > 0.375 else 1  # you can adjust this if needed
            target = closest_idx + lookahead
            self.count = 0
        else:
            target = self.target_waypoint_index + 1
            self.count += 1

        # Ensure we do not exceed the last waypoint index.
        output_target = min(target, len(self.state_refs) - 1)
        self.last_waypoint_index = output_target
        self.target_waypoint_index = output_target  # update the target waypoint index
        return output_target
    
    def draw_result(self, stats, xmin=None, xmax=None, ymin=None, ymax=None, objects=None, car_states=None):
        """
        Before calling the drawing function, convert the stored Frenet trajectories back to global coordinates.
        """
        # Convert initial state, simulation states, and reference states back to global.
        global_init_state = self.frenet_to_global(self.init_state)
        global_robot_states = self.convert_traj_frenet_to_global(self.xx)
        global_ref_states = self.convert_traj_frenet_to_global(self.x_refs)
        # Use the original global waypoints.
        global_waypoints = self.global_waypoints

        # Optionally, set drawing limits using global coordinates.
        if xmin is None:
            # For example, you might compute these from the waypoints:
            xmin = global_waypoints[:, 0].min() - 1
            xmax = global_waypoints[:, 0].max() + 1
            ymin = global_waypoints[:, 1].min() - 1
            ymax = global_waypoints[:, 1].max() + 1

        Draw_MPC_tracking(self.u_c,
                          init_state=global_init_state,
                          robot_states=global_robot_states,
                          ref_states=global_ref_states,
                          export_fig=self.export_fig,
                          waypoints_x=global_waypoints[:, 0],
                          waypoints_y=global_waypoints[:, 1],
                          stats=stats,
                          costs=self.costs,
                          xmin=xmin, xmax=xmax, ymin=ymin, ymax=ymax,
                          times=self.t_c, objects=objects, car_states=car_states)
    def compute_stats(self):
        ## after loop
        print("iter: ", self.mpciter)
        t_v = np.array(self.index_t)
        try:
            print("mean solve time: ",t_v.mean(), "max: ", t_v.max(), "min: ", t_v.min(), "std: ", t_v.std(), "median: ", np.median(t_v))
            print((time.time() - self.start_time)/(self.mpciter))
        except:
            print("error when computing time")
        u_c = np.array(self.u_c).reshape(-1, 2)

        print("average kappa: ", np.mean(self.kappa))
        average_speed = np.mean(u_c[:, 0])
        average_steer = np.mean(u_c[:, 1])
        
        delta_u_c = np.diff(u_c, axis=0)
        average_delta_speed = np.mean(np.abs(delta_u_c[:, 0]))
        average_delta_steer = np.mean(np.abs(delta_u_c[:, 1]))
        
        print(f"Average speed: {average_speed:.4f} m/s")
        print(f"Average steer angle: {average_steer:.4f} rad")
        print(f"Average change in speed: {average_delta_speed:.4f} m/s²")
        print(f"Average change in steer angle: {average_delta_steer:.4f} rad/s")

        average_x_error = np.mean(np.abs(self.x_errors))
        average_y_error = np.mean(np.abs(self.y_errors))
        self.yaw_errors = np.array(self.yaw_errors)
        self.yaw_errors = np.arctan2(np.sin(self.yaw_errors), np.cos(self.yaw_errors))
        average_yaw_error = np.mean(np.abs(self.yaw_errors))

        print(f"Average x error: {average_x_error:.4f} m")
        print(f"Average y error: {average_y_error:.4f} m")
        print(f"Average yaw error: {average_yaw_error:.4f} rad")

        stats = [average_speed, average_steer, average_delta_speed, average_delta_steer, average_x_error, average_y_error, average_yaw_error]
        return stats
   
if __name__ == '__main__':
    argparser = argparse.ArgumentParser(description='MPC controller')
    argparser.add_argument('--save_path', action='store_true', help='save path')
    args = argparser.parse_args()

    x0 = np.array([10, 13.29, np.pi])
    mpc = Optimizer()
    
    mpc.target_waypoint_index = 0
    while True:
        if mpc.target_waypoint_index >= mpc.num_waypoints-1 or mpc.mpciter > 1000:
            break
        t = time.time()
        mpc.x_errors.append(mpc.current_state[0] - mpc.next_trajectories[0, 0])
        mpc.y_errors.append(mpc.current_state[1] - mpc.next_trajectories[0, 1])
        mpc.x_refs.append(mpc.next_trajectories[0, :])
        mpc.yaw_errors.append(mpc.current_state[2] - mpc.next_trajectories[0, 2])
        t_ = time.time()
        u_res = mpc.update_and_solve()
        t2 = time.time()- t_
        if u_res is None:
            break
        mpc.index_t.append(t2)
        mpc.t_c.append(mpc.t0)
        mpc.u_c.append(u_res)
        mpc.integrate_next_states(u_res)
        mpc.xx.append(mpc.current_state)
        mpc.mpciter = mpc.mpciter + 1
    stats = mpc.compute_stats()
    mpc.draw_result(stats, -2, 22, -2, 16)
