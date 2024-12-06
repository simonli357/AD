#!/usr/bin/env python
# coding=UTF-8

# from path import Path
from pathPark import Path
import os
import sys
import shutil
import errno
import timeit

from acados_template import AcadosOcp, AcadosOcpSolver, AcadosSimSolver

# import casadi as ca
import numpy as np
import scipy.linalg

from draw import Draw_MPC_tracking
import casadi as ca
from acados_template import AcadosModel
import time
import yaml

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

class MobileRobotOptimizer(object):
    def __init__(self, gazebo = False):
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
        model.name = 'scandy'

        self.model = model

        cur_path = os.path.dirname(os.path.realpath(__file__))
        config_path = cur_path + '/config/mpc_config_park.yaml'
        self.parking_solver, self.integrator = self.create_solver(self.model, config_path)
        
        self.init_state = np.array([0 , 0, np.pi])

        self.t0 = 0
        self.current_state = self.init_state.copy()
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
        name = "mpc"
        filepath = os.path.dirname(os.path.abspath(__file__))
        self.export_fig = os.path.join(filepath+'/gifs_acados',name + gaz_bool + '_N'+str(self.N) + '_vref'+str(self.v_ref) 
                                       + '_T'+str(self.T))
    def create_solver(self, model, config_path):
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)
        
        self.T = config['T']
        self.N = config['N']
        self.t_horizon = self.T * self.N

        # constraints
        self.v_max = config['constraints']['v_max']
        self.v_min = config['constraints']['v_min']
        self.delta_max = config['constraints']['delta_max']
        self.delta_min = config['constraints']['delta_min']
        self.x_min = config['constraints']['x_min']
        self.x_max = config['constraints']['x_max']
        self.y_min = config['constraints']['y_min']
        self.y_max = config['constraints']['y_max']
        self.v_ref = config['constraints']['v_ref']
        # costs
        self.x_cost = config['costs']['x_cost']
        self.y_cost = config['costs']['y_cost']
        self.yaw_cost = config['costs']['yaw_cost']
        self.v_cost = config['costs']['v_cost']
        self.steer_cost = config['costs']['steer_cost']
        self.delta_v_cost = config['costs']['delta_v_cost']
        self.delta_steer_cost = config['costs']['delta_steer_cost']
        self.costs = np.array([self.x_cost, self.yaw_cost, self.v_cost, self.steer_cost, self.delta_v_cost, self.delta_steer_cost])
        # 代价函数
        Q = np.array([[self.x_cost, 0.0, 0.0],[0.0, self.y_cost, 0.0],[0.0, 0.0, self.yaw_cost]])*1
        R = np.array([[self.v_cost, 0.0], [0.0, self.steer_cost]])*1

        # x状态数
        nx = model.x.size()[0]
        self.nx = nx
        # u状态数
        nu = model.u.size()[0]
        self.nu = nu
        # ny数为x与u之和
        ny = nx + nu
        # 额外参数，没有
        n_params = len(model.p)

        # 设置环境变量
        os.chdir(os.path.dirname(os.path.realpath(__file__)))
        ## 获得系统中ACADOS的安装路径，请参照安装要求设置好
        acados_source_path = os.environ['ACADOS_SOURCE_DIR']
        sys.path.insert(0, acados_source_path)
        # 构建OCP
        ocp = AcadosOcp()
        ## 设置ACADOS系统引用以及库的路径（因为ACADOS最后将以C的形式运行）
        ocp.acados_include_path = acados_source_path + '/include'
        ocp.acados_lib_path = acados_source_path + '/lib'
        ## 设置模型
        ocp.model = self.model
        ocp.dims.N = self.N
        ocp.solver_options.tf = self.t_horizon
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
        ## 这里是为了定义之前约束条件影响的index，不需要像CasADi那样定义np.inf这种没有实际意义的约束。
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
        ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
        ocp.solver_options.integrator_type = 'ERK'
        ocp.solver_options.print_level = 0
        ocp.solver_options.nlp_solver_type = 'SQP_RTI'

        json_file = os.path.join('./'+model.name+'_acados_ocp.json')
        initialize_solver_time = timeit.default_timer()
        solver = AcadosOcpSolver(ocp, json_file=json_file)
        print("initialize_solver_time: ", timeit.default_timer() - initialize_solver_time)
        integrator = AcadosSimSolver(ocp, json_file=json_file)
        return solver, integrator
    
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

    def draw_result(self, stats, xmin=None, xmax=None, ymin=None, ymax=None):
        if xmin is None:
            xmin = self.x_min
            xmax = self.x_max
            ymin = self.y_min
            ymax = self.y_max
        print("saving as ", self.export_fig)
        self.obstacle = []
        self.waypoints_x = []
        self.waypoints_y = []
        Draw_MPC_tracking(self.u_c, obstacle = self.obstacle, rob_diam=0.3, init_state=self.init_state, 
                        robot_states=self.xx, ref_states = self.x_refs, export_fig=self.export_fig, waypoints_x=self.waypoints_x, 
                        waypoints_y=self.waypoints_y, stats = stats, costs = self.costs, xmin=xmin, xmax=xmax, ymin=ymin, ymax=ymax, times = self.t_c)
    def compute_stats(self):
        ## after loop
        print("iter: ", self.mpciter)
        t_v = np.array(self.index_t)
        print("mean solve time: ",t_v.mean(), "max: ", t_v.max(), "min: ", t_v.min(), "std: ", t_v.std(), "median: ", np.median(t_v))
        print((time.time() - self.start_time)/(self.mpciter))
        # print the control input, keep 2 digits after the decimal point
        self.u_c = np.array(self.u_c)
        self.x_c = np.array(self.x_c)

        # print("average kappa: ", np.mean(self.kappa))
        average_speed = np.mean(self.u_c[:, 0])
        average_steer = np.mean(self.u_c[:, 1])
        
        delta_u_c = np.diff(self.u_c, axis=0)
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
    def park(self, xs, cur_frame, ref_frame, thresh):
        errors = []
        current_state_transformed = transform_point(self.current_state, cur_frame, ref_frame)

        xs_between = np.concatenate((xs, np.zeros(2)))
        self.parking_solver.set(self.N, 'yref', xs)
        d = -1
        for i in range(self.N):
            self.parking_solver.set(i, 'yref', xs_between)
        while True:
            current_state_transformed = transform_point(self.current_state, cur_frame, ref_frame)
            d+=1
            error = np.linalg.norm(current_state_transformed - xs)
            print(d, ") transformed: ", np.around(current_state_transformed, 2), ", current: ", np.around(self.current_state, 2), ", error: ", error)
            errors.append(error)
            if d>120 or error < thresh:
                break
            t = time.time()
            self.x_errors.append(self.current_state[0] - self.next_trajectories[0, 0])
            self.y_errors.append(self.current_state[1] - self.next_trajectories[0, 1])
            self.x_refs.append(self.next_trajectories[0, :])
            self.yaw_errors.append(self.current_state[2] - self.next_trajectories[0, 2])
            t_ = time.time()
            self.parking_solver.set(0, 'lbx', current_state_transformed)
            self.parking_solver.set(0, 'ubx', current_state_transformed)
            status = self.parking_solver.solve()
            u_res = self.parking_solver.get(0, 'u')
            if status != 0 :
                raise Exception('acados acados_ocp_solver returned status {}. Exiting.'.format(status))
            t2 = time.time()- t_
            if u_res is None:
                break
            self.t_c.append(self.t0)
            self.u_c.append(u_res)
            self.index_t.append(t2)
            self.integrate_next_states(u_res)
            self.xx.append(self.current_state)
            self.mpciter = self.mpciter + 1
        print("min error: ", np.min(errors)," at index: ", np.argmin(errors))
        return 

if __name__ == '__main__':
    mpc = MobileRobotOptimizer()
    mpc.current_state = np.array([-1, 2, np.pi/3])
    cur_frame = mpc.current_state.copy()
    ref_frame = np.array([1, 0, np.pi])
    xs = np.array([0, 0, np.pi])
    mpc.park(xs, cur_frame, ref_frame, thresh = 0.01)

    cur_frame = mpc.current_state.copy()
    ref_frame = np.array([0, 0, np.pi])
    mpc.parking_solver.reset()
    xs = np.array([0.63, 0.32, np.pi])
    mpc.park(xs, cur_frame, ref_frame, thresh = 0.08)

    cur_frame = mpc.current_state.copy()
    ref_frame = np.array([0.63, 0.32, np.pi])
    mpc.parking_solver.reset()
    xs = np.array([0.13, 0, np.pi])
    mpc.park(xs, cur_frame, ref_frame, thresh = 0.08)

    print("done")
    mpc.draw_result(mpc.compute_stats(), -3, 3, -3, 3)
