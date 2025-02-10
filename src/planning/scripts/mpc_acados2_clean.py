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

        name = 'run3'
        self.path = Path(v_ref = self.v_ref, N = self.N, T = self.T, name=name, x0=x0)
        self.waypoints_x = self.path.waypoints_x
        self.waypoints_y = self.path.waypoints_y
        self.num_waypoints = self.path.num_waypoints
        self.wp_normals = self.path.wp_normals
        self.kappa = self.path.kappa
        self.density = self.path.density
        self.state_refs = self.path.state_refs
        self.input_refs = self.path.input_refs
        self.waypoints_x = self.state_refs[:,0]
        self.waypoints_y = self.state_refs[:,1]

        self.counter = 0
        self.target_waypoint_index = 0
        self.last_waypoint_index = 0
        density = 1/abs(self.v_ref)/self.T
        self.region_of_acceptance = 0.05/10*density * 2*1.5
        self.last_u = None
        self.t0 = 0
        self.init_state = x0 if x0 is not None else self.state_refs[0]
        self.update_current_state(self.init_state[0], self.init_state[1], self.init_state[2])
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
        
    def create_solver(self, config_path='config/mpc_config25.yaml'):
        model = AcadosModel() #  ca.types.SimpleNamespace()
        # control inputs
        v = ca.SX.sym('v')
        delta = ca.SX.sym('delta')
        controls = ca.vertcat(v, delta)
        # model states
        x = ca.SX.sym('x')
        y = ca.SX.sym('y')
        psi = ca.SX.sym('psi')
        states = ca.vertcat(x, y, psi)
        self.L = 0.258
        rhs = [v*ca.cos(psi), v*ca.sin(psi), v/self.L*ca.tan(delta)]

        f = ca.Function('f', [states, controls], [ca.vcat(rhs)], ['state', 'control_input'], ['rhs'])
        # acados model
        x_dot = ca.SX.sym('x_dot', len(rhs))
        f_impl = x_dot - f(states, controls)

        model.f_expl_expr = f(states, controls)
        model.f_impl_expr = f_impl
        model.x = states
        model.xdot = x_dot
        model.u = controls
        model.p = []

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
        self.v_ref = config[constraint_name]['v_ref']
        self.x_cost = config[cost_name]['x_cost']
        self.y_cost = config[cost_name]['y_cost']
        self.yaw_cost = config[cost_name]['yaw_cost']
        self.v_cost = config[cost_name]['v_cost']
        self.steer_cost = config[cost_name]['steer_cost']
        self.delta_v_cost = config[cost_name]['delta_v_cost']
        self.delta_steer_cost = config[cost_name]['delta_steer_cost']
        self.costs = np.array([self.x_cost, self.yaw_cost, self.v_cost, self.steer_cost, self.delta_v_cost, self.delta_steer_cost])
        Q = np.array([[self.x_cost, 0.0, 0.0],[0.0, self.y_cost, 0.0],[0.0, 0.0, self.yaw_cost]])*1
        R = np.array([[self.v_cost, 0.0], [0.0, self.steer_cost]])*1

        nx = model.x.size()[0]
        nu = model.u.size()[0]
        ny = nx + nu
        n_params = len(model.p)

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
        ocp.constraints.lbx = np.array([self.x_min, self.y_min])
        ocp.constraints.ubx = np.array([self.x_max, self.y_max])
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
        self.target_waypoint_index = self.find_next_waypoint()
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
    
    def update_current_state(self, x, y, yaw):
        if self.target_waypoint_index < len(self.state_refs):
            ref_yaw = self.state_refs[self.target_waypoint_index, 2]
            while yaw - ref_yaw > np.pi:
                yaw -= 2 * np.pi
            while yaw - ref_yaw < -np.pi:
                yaw += 2 * np.pi
        self.current_state = np.array([x, y, yaw])
        
    def find_closest_waypoint(self):
        distances = np.linalg.norm(np.vstack((self.waypoints_x, self.waypoints_y)).T - self.current_state[:2], axis=1)
        index = np.argmin(distances)
        return index, distances[index]
    def find_next_waypoint(self):
        closest_idx, dist_to_waypoint = self.find_closest_waypoint()
        if dist_to_waypoint < self.region_of_acceptance:
            if closest_idx - self.last_waypoint_index < 15:
                self.last_waypoint_index = max(self.last_waypoint_index, closest_idx+1)
            else:
                closest_idx = self.last_waypoint_index
        else:
            if closest_idx - self.last_waypoint_index > 15:
                closest_idx = self.last_waypoint_index + 1
            # If not within the region of acceptance, take smaller steps forward in the waypoint list
            self.last_waypoint_index += 1
        target_idx = max(self.last_waypoint_index, closest_idx)
        return min(target_idx, len(self.waypoints_x) - 1)
    def draw_result(self, stats, xmin=None, xmax=None, ymin=None, ymax=None, objects=None, car_states=None):
        if xmin is None:
            xmin = self.x_min
            xmax = self.x_max
            ymin = self.y_min
            ymax = self.y_max
        Draw_MPC_tracking(self.u_c, init_state=self.init_state, 
                        robot_states=self.xx, ref_states = self.x_refs, export_fig=self.export_fig, waypoints_x=self.waypoints_x, 
                        waypoints_y=self.waypoints_y, stats = stats, costs = self.costs, xmin=xmin, xmax=xmax, ymin=ymin, ymax=ymax,
                        times = self.t_c, objects=objects, car_states=car_states)
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
