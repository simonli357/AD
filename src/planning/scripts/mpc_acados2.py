#!/usr/bin/env python3
# coding=UTF-8

from path2 import Path
# from turning_wpts import Path
import os
import sys
import shutil
import errno
import timeit

from acados_template import AcadosOcp, AcadosOcpSolver, AcadosSimSolver

# import casadi as ca
import numpy as np
import scipy.linalg

from draw2 import Draw_MPC_tracking
import casadi as ca
from acados_template import AcadosModel
import time
import yaml
import argparse

def transform_point(pt, frame1, frame2):
    # Unpack the frames and the point
    x1, y1, theta1 = frame1
    x2, y2, theta2 = frame2
    x, y, psi = pt
    # Step 1: Translate to the origin of frame1
    x -= x1
    y -= y1
    # Step 2: Rotate to align frame1 with frame2
    # First, compute the rotation needed
    rotation_angle = theta2 - theta1
    # Create the rotation matrix
    rotation_matrix = np.array([
        [np.cos(rotation_angle), -np.sin(rotation_angle)],
        [np.sin(rotation_angle), np.cos(rotation_angle)]
    ])
    # Apply the rotation
    rotated_xy = np.dot(np.array([x, y]), rotation_matrix.T)
    # Update psi (yaw) and normalize
    rotated_psi = (psi + rotation_angle) % (2 * np.pi)
    # rotated_psi = np.arctan2(np.sin(rotated_psi), np.cos(rotated_psi))
    # Step 3: Translate to the origin of frame2
    transformed_xy = rotated_xy + np.array([x2, y2])
    # Recombine into the complete array and return
    return np.array([transformed_xy[0], transformed_xy[1], rotated_psi])

class Optimizer(object):
    def __init__(self, gazebo = False, x0 = None):
        self.gazebo = gazebo
        self.solver, self.integrator, self.T, self.N, self.t_horizon = self.create_solver()

        name = 'run1'
        # name = 'speedrun'
        # name = 'path2'
        self.path = Path(v_ref = self.v_ref, N = self.N, T = self.T, name=name, x0=x0)
        self.waypoints_x = self.path.waypoints_x
        self.waypoints_y = self.path.waypoints_y
        self.num_waypoints = self.path.num_waypoints
        self.wp_normals = self.path.wp_normals
        self.kappa = self.path.kappa
        self.density = self.path.density
        print("density: ", self.density)
        self.state_refs = self.path.state_refs
        self.input_refs = self.path.input_refs
        self.waypoints_x = self.state_refs[:,0]
        self.waypoints_y = self.state_refs[:,1]

        self.counter = 0
        self.target_waypoint_index = 0
        self.last_waypoint_index = 0
        density = 1/abs(self.v_ref)/self.T
        # self.region_of_acceptance = 0.05
        self.region_of_acceptance = 0.05/10*density * 2*1.5
        print("region_of_acceptance: ", self.region_of_acceptance)
        self.rdb_circumference = 4.15
        import math
        self.limit = math.floor(self.rdb_circumference/(self.v_ref * self.T))

        self.t0 = 0
        self.init_state = x0 if x0 is not None else self.state_refs[0]
        self.update_current_state(self.init_state[0], self.init_state[1], self.init_state[2])
        self.init_state = self.current_state.copy()
        # self.init_state = np.array([0.2, 0.2, 0])
        # self.current_state = self.init_state.copy()
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
        gaz_bool = "" if not gazebo else "_gazebo_"
        filepath = os.path.dirname(os.path.abspath(__file__))
        self.export_fig = os.path.join(filepath+'/gifs_acados',name + gaz_bool + '_N'+str(self.N) + '_vref'+str(self.v_ref) 
                                       + '_T'+str(self.T))
        self.obstacle = []
        
    def create_solver(self, config_path='config/mpc_config.yaml'):
        # extract the number from config_path (50 in this case)
        try:
            self.v_ref_int = str(int(''.join(filter(str.isdigit, config_path))))
        except:
            self.v_ref_int = ''
        print("number: ", self.v_ref_int)

        print("creating solver...")
        t1 = timeit.default_timer()
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
        self.L = 0.27
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
        if "park" in config_path:
            self.park_thresh = config['park_thresh']
            self.exit_thresh = config['exit_thresh']
        if "park" not in config_path:
            T = config['T']
            N = config['N']
            constraint_name = 'constraints'
            cost_name = 'costs'
        else:
            T = config['T_park']
            N = config['N_park']
            constraint_name = 'constraints_park'
            cost_name = 'costs_park'
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
        print("v_ref: ", self.v_ref)
        # costs
        self.x_cost = config[cost_name]['x_cost']
        self.y_cost = config[cost_name]['y_cost']
        self.yaw_cost = config[cost_name]['yaw_cost']
        self.v_cost = config[cost_name]['v_cost']
        self.steer_cost = config[cost_name]['steer_cost']
        self.delta_v_cost = config[cost_name]['delta_v_cost']
        self.delta_steer_cost = config[cost_name]['delta_steer_cost']
        self.costs = np.array([self.x_cost, self.yaw_cost, self.v_cost, self.steer_cost, self.delta_v_cost, self.delta_steer_cost])
        # 代价函数
        Q = np.array([[self.x_cost, 0.0, 0.0],[0.0, self.y_cost, 0.0],[0.0, 0.0, self.yaw_cost]])*1
        R = np.array([[self.v_cost, 0.0], [0.0, self.steer_cost]])*1

        # x状态数
        nx = model.x.size()[0]
        # u状态数
        nu = model.u.size()[0]
        # ny数为x与u之和，原因详见系统CasADi例子以及ACADOS构建优化问题PDF介绍
        ny = nx + nu
        # 额外参数，本例中没有
        n_params = len(model.p)

        # 设置环境变量
        os.chdir(os.path.dirname(os.path.realpath(__file__)))
        ## 获得系统中ACADOS的安装路径，请参照安装要求设置好
        acados_source_path = os.environ['ACADOS_SOURCE_DIR']
        sys.path.insert(0, acados_source_path)
        # 构建OCP
        ocp = AcadosOcp()
        ## 设置ACADOS系统引用以及库的路径（因为ACADOS最后将以C的形式运行，所以必须设置正确）
        ocp.acados_include_path = acados_source_path + '/include'
        ocp.acados_lib_path = acados_source_path + '/lib'
        ## 设置模型
        ocp.model = model
        ocp.dims.N = N
        ocp.solver_options.tf = t_horizon
        ocp.dims.np = n_params
        ocp.parameter_values = np.zeros(n_params)

        ## cost类型为线性
        ocp.cost.cost_type = 'LINEAR_LS'
        ocp.cost.cost_type_e = 'LINEAR_LS'
        ocp.cost.W = scipy.linalg.block_diag(Q, R)
        ocp.cost.W_e = Q
        ## 这里V类矩阵的定义需要参考acados构建里面的解释，实际上就是定义一些映射关系
        ocp.cost.Vx = np.zeros((ny, nx))
        ocp.cost.Vx[:nx, :nx] = np.eye(nx)
        ocp.cost.Vu = np.zeros((ny, nu))
        ocp.cost.Vu[-nu:, -nu:] = np.eye(nu)
        ocp.cost.Vx_e = np.eye(nx)

        # 约束条件设置
        ocp.constraints.lbu = np.array([self.v_min, self.delta_min])
        ocp.constraints.ubu = np.array([self.v_max, self.delta_max])
        ## 这里是为了定义之前约束条件影响的index，它不需要像CasADi那样定义np.inf这种没有实际意义的约束。
        ocp.constraints.idxbu = np.array([0, 1])
        ocp.constraints.lbx = np.array([self.x_min, self.y_min])
        ocp.constraints.ubx = np.array([self.x_max, self.y_max])
        ocp.constraints.idxbx = np.array([0, 1])

        # 一些状态的值，在实际仿真中可以重新给定，所里这里就定义一些空值
        x_ref = np.zeros(nx)
        u_ref = np.zeros(nu)
        ## 将给定值设定，注意到这里不需要像之前那样给所有N-1设定ref值，ACADOS会默认进行设置
        ocp.constraints.x0 = x_ref
        ### 0--N-1
        ocp.cost.yref = np.concatenate((x_ref, u_ref))
        ### N
        ocp.cost.yref_e = x_ref # yref_e means the reference for the last stage

        ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
        # ocp.solver_options.qp_solver = 'FULL_CONDENSING_HPIPM'
        # ocp.solver_options.qp_solver = 'FULL_CONDENSING_QPOASES'
        ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
        ocp.solver_options.integrator_type = 'ERK'
        ocp.solver_options.print_level = 0
        # ocp.solver_options.nlp_solver_type = 'SQP'
        ocp.solver_options.nlp_solver_type = 'SQP_RTI'

        json_file = os.path.join('./'+model.name+'_acados_ocp.json')
        initialize_solver_time = timeit.default_timer()
        solver = AcadosOcpSolver(ocp, json_file=json_file)
        print("initialize_solver_time: ", timeit.default_timer() - initialize_solver_time)
        integrator = AcadosSimSolver(ocp, json_file=json_file)
        print("done creating solver, time: ", timeit.default_timer() - t1)
        return solver, integrator, T, N, t_horizon
    
    def update_and_solve(self):
        # cur_path = os.path.dirname(os.path.realpath(__file__))
        # path = os.path.join(cur_path, 'paths')
        # os.makedirs(path, exist_ok=True)
        # # np.savetxt(os.path.join(path,'straight_states25.txt'), self.state_refs, fmt='%.8f')
        # # np.savetxt(os.path.join(path,'straight_inputs25.txt'), self.input_refs, fmt='%.8f')

        # np.savetxt(os.path.join(path,'state_refs_'+self.path.name+(self.v_ref_int)+'.txt'), self.state_refs, fmt='%.8f')
        # print("stateref shape: ", self.state_refs.shape)
        # np.savetxt(os.path.join(path,'input_refs_'+self.path.name+(self.v_ref_int)+'.txt'), self.input_refs, fmt='%.8f')
        # np.savetxt(os.path.join(path,'wp_normals_'+self.path.name+(self.v_ref_int)+'.txt'), self.wp_normals, fmt='%.8f')
        # np.savetxt(os.path.join(path,'wp_attributes_'+self.path.name+(self.v_ref_int)+'.txt'), self.path.attributes, fmt='%.8f')

        # # np.savetxt(os.path.join(path,'kappa2.txt'), self.kappa, fmt='%.8f')
        # exit()

        self.target_waypoint_index = self.find_next_waypoint()
        idx = self.target_waypoint_index
        self.next_trajectories = self.state_refs[idx:idx + self.N + 1]
        self.next_controls = self.input_refs[idx:idx + self.N]
        # xs = self.state_refs[idx+ self.N]
        xs = self.state_refs[idx]
        self.solver.set(self.N, 'yref', xs)
        for j in range(self.N):
            if self.target_waypoint_index+j >= self.state_refs.shape[0]:
                self.solver.set(j, 'yref', np.concatenate((self.state_refs[-1], np.zeros(2))))
            else:
                self.solver.set(j, 'yref', np.concatenate((self.next_trajectories[j], self.next_controls[j])))# 设置当前循环x0 (stage 0)
        # 设置当前循环x0 (stage 0)
        self.solver.set(0, 'lbx', self.current_state)
        self.solver.set(0, 'ubx', self.current_state)
        # 求解
        status = self.solver.solve()
        if status != 0 :
            print('ERROR!!! acados acados_ocp_solver returned status {}. Exiting.'.format(status))
            return None
        # 得到下个时刻最优控制
        next_u = self.solver.get(0, 'u')
        # std::cout << "x_cur: " << x_current[0] << ", " << x_current[1] << ", " << x_current[2] << ", ref: " << state_refs(idx, 0) << ", " << state_refs(idx, 1) << ", " << state_refs(idx, 2) << ", u: " << u_current[0] << ", " << u_current[1] << ", error: " << error << std::endl;
        # print x_cur, ref, u_cur, error
        self.counter += 1
        # print(self.counter, ") cur: ", np.around(self.current_state, 2), ", ref: ", np.around(self.next_trajectories[0, :], 2), ", ctrl: ", np.around(next_u, 2), ", idx: ", self.target_waypoint_index)
        return next_u
    def integrate_next_states(self, u_res=None):
        # 以下纯粹为了仿真
        # 仿真器获得当前位置和控制指令
        self.integrator.set('x', self.current_state)
        self.integrator.set('u', u_res)
        # 仿真器计算结果
        status_s = self.integrator.solve()
        if status_s != 0:
            raise Exception('acados integrator returned status {}. Exiting.'.format(status_s))
        # 将仿真器计算的小车位置作为下一个时刻的初始值
        self.current_state = self.integrator.get('x')
        self.t0 += self.T
    
    def move_to(self, xs, cur_frame, ref_frame, thresh, utils=None, odom=False, u_ref = np.zeros(2)):
        if utils is not None:
            utils.set_rate(1/self.T)
            print("rate: ", utils.rate)
        errors = []
        current_state_transformed = transform_point(self.current_state, cur_frame, ref_frame)
        xs_between = np.concatenate((xs, u_ref))
        self.solver.set(self.N, 'yref', xs)
        d = -1
        for i in range(self.N):
            self.solver.set(i, 'yref', xs_between)
        while True:
            current_state_transformed = transform_point(self.current_state, cur_frame, ref_frame)
            d+=1
            error = (current_state_transformed - xs)
            # error = np.linalg.norm(np.array([error[1], error[2]]))
            error = np.linalg.norm(error)
            errors.append(error)
            if d>150 or error < thresh:
                break
            t_ = time.time()
            self.solver.set(0, 'lbx', current_state_transformed)
            self.solver.set(0, 'ubx', current_state_transformed)
            status = self.solver.solve()
            u_res = self.solver.get(0, 'u')
            # print(d, ") trans: ", np.around(current_state_transformed, 2), ", cur: ", np.around(self.current_state, 2), ", error: ", round(error, 2), ", u: ", np.around(u_res, 2))
            if status != 0 :
                raise Exception('acados acados_ocp_solver returned status {}. Exiting.'.format(status))
            t2 = time.time()- t_
            if u_res is None:
                break
            self.t_c.append(self.t0)
            self.u_c.append(u_res)
            self.index_t.append(t2)
            if utils is None:
                self.integrate_next_states(u_res)
            else:
                utils.steer_command = -float(u_res[1]*180/np.pi)
                utils.velocity_command = float(u_res[0])
                utils.publish_cmd_vel(steering_angle=utils.steer_command, velocity=utils.velocity_command)
                with self.lock:
                    if odom:
                        self.update_current_state(x=utils.odomX, y=utils.odomY, yaw=utils.yaw)
                        self.update_real_state(x=utils.gps_x, y=utils.gps_y, yaw=utils.yaw)
                    else:
                        self.update_current_state(x=utils.gps_x, y=utils.gps_y, yaw=utils.yaw)
                utils.rate.sleep()
            self.xx.append(self.current_state)
            self.mpciter = self.mpciter + 1
        if utils is not None:
            for i in range(10):
                utils.steer_command = -float(0.0)
                utils.velocity_command = float(0.0)
                utils.publish_cmd_vel(steering_angle=utils.steer_command, velocity=utils.velocity_command)
                utils.rate.sleep()
        print("min error: ", np.min(errors)," at index: ", np.argmin(errors))
        return 
    def go_straight(self, offset, utils=None, odom=False, cur_frame = None):
        if cur_frame is None:
            cur_frame = self.current_state.copy()
        self.park_yaw = cur_frame[2]
        ref_frame = np.array([offset, 0, np.pi])
        xs = np.array([0, 0, np.pi])
        thresh = 0.05
        self.move_to(xs, cur_frame, ref_frame, thresh = thresh, utils=utils, odom=odom)
    def park(self, utils=None, odom=False, cur_frame = None):
        del self.solver, self.integrator
        parking_config_path = 'config/mpc_config_park.yaml'
        self.solver, self.integrator, self.T, self.N, self.t_horizon = self.create_solver(parking_config_path)
        if cur_frame is None:
            cur_frame = self.current_state.copy()
        self.park_yaw = cur_frame[2]
        # ref_frame = np.array([offset, 0, np.pi])
        # xs = np.array([0, 0, np.pi])
        thresh = self.park_thresh if self.park_thresh is not None else 0.05
        # self.move_to(xs, cur_frame, ref_frame, thresh = thresh, utils=utils, odom=odom)
        # self.solver.reset()
        # cur_frame = self.current_state.copy()
        ref_frame = np.array([0, 0, np.pi])
        xs = np.array([0.63, 0.32, np.pi])
        self.move_to(xs, cur_frame, ref_frame, thresh = thresh, utils=utils, odom=odom, u_ref = np.array([0.0, -0.0]))
    def exit_park(self, utils=None, odom=False, cur_frame = None):
        self.solver.reset()
        if cur_frame is None:
            cur_frame = self.current_state.copy()
        cur_frame[2] = self.park_yaw
        ref_frame = np.array([0.63, 0.32, np.pi])
        xs = np.array([0.0, 0.0, np.pi])
        thresh = self.exit_thresh if self.exit_thresh is not None else 0.08
        self.move_to(xs, cur_frame, ref_frame, thresh = thresh, utils=utils, odom=odom, u_ref = np.array([0.0, 0.0]))
        del self.solver, self.integrator
        self.solver, self.integrator, self.T, self.N, self.t_horizon = self.create_solver()

    def update_current_state(self, x, y, yaw):
        # Ensure target_waypoint_index is within bounds
        if self.target_waypoint_index < len(self.state_refs):
            # Extract the reference yaw at the current target waypoint
            ref_yaw = self.state_refs[self.target_waypoint_index, 2]
            # Adjust yaw to minimize its difference with ref yaw
            while yaw - ref_yaw > np.pi:
                yaw -= 2 * np.pi
            while yaw - ref_yaw < -np.pi:
                yaw += 2 * np.pi
        self.current_state = np.array([x, y, yaw])
    def update_real_state(self, x, y, yaw):
        if self.target_waypoint_index < len(self.state_refs):
            ref_yaw = self.state_refs[self.target_waypoint_index, 2]
            while yaw - ref_yaw > np.pi:
                yaw -= 2 * np.pi
            while yaw - ref_yaw < -np.pi:
                yaw += 2 * np.pi
        self.real_state = np.array([x, y, yaw])
    def find_closest_waypoint(self):
        # min_index = max(0, self.last_waypoint_index - self.limit)
        # max_index = min(self.num_waypoints - 1, self.last_waypoint_index + self.limit)
        # distances = np.linalg.norm(np.vstack((self.waypoints_x[min_index:max_index], self.waypoints_y[min_index:max_index])).T - self.current_state[:2], axis=1)
        distances = np.linalg.norm(np.vstack((self.waypoints_x, self.waypoints_y)).T - self.current_state[:2], axis=1)
        index = np.argmin(distances)
        return index, distances[index]
    def find_next_waypoint(self):
        # start = timeit.default_timer()
        closest_idx, dist_to_waypoint = self.find_closest_waypoint()

        # self.last_waypoint_index = self.target_waypoint_index
        # if dist_to_waypoint < self.region_of_acceptance:
        #     self.target_waypoint_index += 1
        # if closest_idx > self.target_waypoint_index:
        #     self.target_waypoint_index = closest_idx + 1
        # elif closest_idx + self.limit/3 < self.target_waypoint_index:
        #     self.target_waypoint_index = closest_idx + 1
        # return min(self.target_waypoint_index, len(self.waypoints_x) - 1)

        if dist_to_waypoint < self.region_of_acceptance:
            if closest_idx - self.last_waypoint_index < 15:
                self.last_waypoint_index = max(self.last_waypoint_index, closest_idx+1)
            else:
                # print("here: ", closest_idx, self.last_waypoint_index, closest_idx - self.last_waypoint_index)
                closest_idx = self.last_waypoint_index
                # self.last_waypoint_index += 1
        else:
            if closest_idx - self.last_waypoint_index > 15:
                # print("here2: ", closest_idx, self.last_waypoint_index, closest_idx - self.last_waypoint_index)
                closest_idx = self.last_waypoint_index + 1
                # closest_idx = self.last_waypoint_index 
            # If not within the region of acceptance, take smaller steps forward in the waypoint list
            self.last_waypoint_index += 1
        target_idx = max(self.last_waypoint_index, closest_idx)
        # print this: std::cout << "cur:" << current_state[0] << "," << current_state[1] << "," << current_state[2] << ", closest_idx:" << closest_idx << ", closest:" << state_refs(closest_idx, 0) << "," << state_refs(closest_idx, 1) << "," << state_refs(closest_idx, 2) << ", dist: " << dist<<  ", last_waypoint_index:" << last_waypoint_index << ", target_idx:" << target_waypoint_index << std::endl;
        # print("cur:", self.current_state, ", closest_idx: ", closest_idx, ", closest: ", self.state_refs[closest_idx], "dist_to_waypoint: ", dist_to_waypoint,  ", last_waypoint_index: ", self.last_waypoint_index, ", target_idx: ", target_idx)
        # print("find_next_waypoint time: ", timeit.default_timer() - start)
        return min(target_idx, len(self.waypoints_x) - 1)
    def draw_result(self, stats, xmin=None, xmax=None, ymin=None, ymax=None, objects=None, car_states=None):
        if xmin is None:
            xmin = self.x_min
            xmax = self.x_max
            ymin = self.y_min
            ymax = self.y_max
        print("saving as ", self.export_fig)
        # objects = [
        #     {'type': 'stop_sign', 'pose': [3, 3]},
        #     {'type': 'traffic_light', 'pose': [7, 8]},
        #     {'type': 'parking_spot', 'pose': [10, 2]},
        #     {'type': 'crosswalk', 'pose': [12, 4]},
        #     {'type': 'roundabout', 'pose': [5, 10]}
        # ]
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
        # print the control input, keep 2 digits after the decimal point
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

    running = True

    x0 = np.array([10, 13.29, np.pi])
    # mpc = Optimizer(x0=x0)
    mpc = Optimizer()
    
    if running:
        mpc.target_waypoint_index = 0
        while True:
            if mpc.target_waypoint_index >= mpc.num_waypoints-1 or mpc.mpciter > 1000:
            # if mpc.target_waypoint_index >= 375:
                break
            t = time.time()
            mpc.x_errors.append(mpc.current_state[0] - mpc.next_trajectories[0, 0])
            mpc.y_errors.append(mpc.current_state[1] - mpc.next_trajectories[0, 1])
            mpc.x_refs.append(mpc.next_trajectories[0, :])
            mpc.yaw_errors.append(mpc.current_state[2] - mpc.next_trajectories[0, 2])
            # print("cur: ", np.around(mpc.current_state, decimals=2), ", ref: ", np.around(mpc.next_trajectories[0, :], decimals=2), ", ctrl: ", np.around(mpc.next_controls[0, :], decimals=2), ", idx: ", mpc.target_waypoint_index)
            t_ = time.time()
            u_res = mpc.update_and_solve()
            t2 = time.time()- t_
            if u_res is None:
                break
            mpc.index_t.append(t2)
            mpc.t_c.append(mpc.t0)
            mpc.u_c.append(u_res)
            mpc.integrate_next_states(u_res)
            # u_res = u_res[0]
            # print("time: ", t2, "u_res: ", u_res)
            mpc.xx.append(mpc.current_state)
            mpc.mpciter = mpc.mpciter + 1
        stats = mpc.compute_stats()
        # save current states and controls as txt
        # np.savetxt('x.txt', mpc.xx, fmt='%.8f')
        # np.savetxt('u.txt', mpc.u_c, fmt='%.8f')
        # park_offset = 0.#95
        # mpc.current_state = np.array([park_offset, 0, np.pi])
        # mpc.go_straight(park_offset)
        # mpc.park()
        # mpc.exit_park()
        print("done")
        mpc.draw_result(stats, -35, 35, -35, 35)
        # mpc.draw_result(stats, -1, 5, -1, 2)
