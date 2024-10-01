#!/usr/bin/env python3

import time
import numpy as np
import os
from scipy.interpolate import UnivariateSpline, splprep, splev
from global_planner import GlobalPlanner
import yaml
import matplotlib.pyplot as plt

def transformation_matrix(state1, state2):
    # Extract individual components from state arrays
    x1, y1, theta1 = state1
    x2, y2, theta2 = state2
    # Rotation matrix to align state1 with x-axis
    R1 = np.array([
        [np.cos(-theta1), -np.sin(-theta1), 0],
        [np.sin(-theta1), np.cos(-theta1), 0],
        [0, 0, 1]
    ])
    # Translation matrix to move origin of state1 to global origin
    T1 = np.array([
        [1, 0, -x1],
        [0, 1, -y1],
        [0, 0, 1]
    ])
    # Translation matrix to move to origin of state2
    T2 = np.array([
        [1, 0, x2],
        [0, 1, y2],
        [0, 0, 1]
    ])
    # Rotation matrix to align with state2
    R2 = np.array([
        [np.cos(theta2), -np.sin(theta2), 0],
        [np.sin(theta2), np.cos(theta2), 0],
        [0, 0, 1]
    ])
    # Combine transformations
    M = np.dot(T2, np.dot(R2, np.dot(T1, R1)))
    return M

def transform_pose(pose, M, theta_diff):
    # Apply the transformation to get new x and y
    transformed_pose_homogeneous = np.dot(M, pose)
    # Compute new theta by adding the orientation difference between frame1 and frame2
    transformed_theta = (pose[2] + (theta_diff)) % (2 * np.pi)
    # Update theta in the transformed pose
    transformed_pose_homogeneous[2] = transformed_theta
    return transformed_pose_homogeneous
def plot_waypoints(waypoints, color='blue', label_suffix=''):
    # Extract x, y, and psi from waypoints
    x = waypoints[:, 0]
    y = waypoints[:, 1]
    psi = waypoints[:, 2]
    
    # Calculate boundaries with a margin
    margin = 2  # you can adjust this value to provide more space around the waypoints
    x_min, x_max = min(x) - margin, max(x) + margin
    y_min, y_max = min(y) - margin, max(y) + margin

    # Calculate dynamic sizes based on data range
    marker_size = max(2, (x_max - x_min + y_max - y_min) / 100)  # example logic
    arrow_scale = marker_size / 4/6
    head_width = arrow_scale / 2/3
    head_length = arrow_scale * 1.5/3

    plt.figure(figsize=(10, 8))
    plt.title('Waypoints with Yaw')
    plt.xlabel('x')
    plt.ylabel('y')
    plt.grid(True)
    plt.axis('equal')
    plt.xlim(x_min, x_max)
    plt.ylim(y_min, y_max)

    # Plot the waypoints with adjusted marker size
    plt.plot(x, y, '-o', color=color, label=f'Waypoints {label_suffix}', markersize=marker_size)
    # Plot arrows indicating yaw with adjusted sizes
    # for xi, yi, psii in zip(x, y, psi):
    #     plt.arrow(xi, yi, arrow_scale * np.cos(psii), arrow_scale * np.sin(psii), head_width=head_width, head_length=head_length, fc=color, ec=color)
    
    plt.legend()
    plt.show()
def smooth_yaw_angles(yaw_angles):
    # Calculate the differences between adjacent angles
    diffs = np.diff(yaw_angles)
    # Find where the difference is greater than pi and adjust
    diffs[diffs > np.pi] -= 2 * np.pi
    diffs[diffs < -np.pi] += 2 * np.pi
    # Compute the smoothed yaw angles
    smooth_yaw = np.concatenate(([yaw_angles[0]], yaw_angles[0] + np.cumsum(diffs)))
    return smooth_yaw

def compute_smooth_curvature(waypoints_x, waypoints_y, smooth_factor=0.1):
    # Step 1: Set up a spline interpolation
    t = np.linspace(0, 1, len(waypoints_x))
    spline_x = UnivariateSpline(t, waypoints_x, k=3, s=smooth_factor)
    spline_y = UnivariateSpline(t, waypoints_y, k=3, s=smooth_factor)
    # Step 2: Get smoothed derivatives of the path
    t_smooth = np.linspace(0, 1, len(waypoints_x))
    dx_dt = spline_x.derivative()(t_smooth)
    dy_dt = spline_y.derivative()(t_smooth)
    ddx_dt = spline_x.derivative(n=2)(t_smooth)
    ddy_dt = spline_y.derivative(n=2)(t_smooth)
    # Step 3: Compute curvature
    # κ = |dx/dt * d²y/dt² - dy/dt * d²x/dt²| / (dx/dt² + dy/dt²)^(3/2)
    curvature = np.abs(dx_dt * ddy_dt - dy_dt * ddx_dt) / (dx_dt**2 + dy_dt**2)**(3/2)
    # Step 4: Compute tangent 
    tangent_angles = np.arctan2(dy_dt, dx_dt)
    # Step 5: Compute normal
    normal_angles = tangent_angles + np.pi / 2
    # Convert normal angles to vectors (dx, dy)
    dx = np.cos(normal_angles)
    dy = np.sin(normal_angles)
    normals = np.vstack((dx, dy)).T
    return curvature, tangent_angles, normals

def filter_waypoints(waypoints, threshold):
    filtered_waypoints = [waypoints[0]]
    for i in range(1, len(waypoints)):
        if np.linalg.norm(np.array(waypoints[i]) - np.array(waypoints[i-1])) >= threshold:
            filtered_waypoints.append(waypoints[i])
        else: 
            # print("filtered out waypoint: ",i, waypoints[i])
            continue
    return np.array(filtered_waypoints)

def interpolate_waypoints(waypoints, num_points):
    x = waypoints[:, 0]
    y = waypoints[:, 1]
    tck, u = splprep([x, y], s=0) 
    # Generate set of equally spaced waypoints
    u_new = np.linspace(0, 1, num_points)
    x_new, y_new = splev(u_new, tck) 
    # Stack the x and y coordinates to get new waypoints
    new_waypoints = np.vstack((x_new, y_new)).T
    return new_waypoints

class Path:
    def __init__(self, v_ref, N, T, name, x0=None):
        self.v_ref = v_ref
        self.N = N
        self.global_planner = GlobalPlanner()

        current_path = os.path.dirname(os.path.abspath(__file__))
        with open(os.path.join(current_path, 'config/paths.yaml'), 'r') as stream:
            data = yaml.safe_load(stream)
            # destinations = data[name]

        # Plan runs between sequential destinations
        runs = []
        run1 = np.array([[0, 0], [0.4, 0.16], [0.75, 0.32]]).T
        run1 = np.array([[2.37, 13.3],[2.5, 13.32],[2.8, 13.36], [3.05,13.644], [3.214,13.644]]).T - np.array([2.37, 13.3]).reshape(2,1)
        filepath = os.path.dirname(os.path.abspath(__file__))
        run1 = np.load(os.path.join(filepath, 'waypoints/parallel_park.npy')).T
        run1[0,:]*=-1
        #shape is (2,60). create a less dense version of (2,20)
        wpts = np.zeros((2,5))
        for i in range(5):
            wpts[:,i] = run1[:,i*12]
        run1 = wpts
        print("wpts: ", run1)
        runs.append(run1)
        print("run1: ", run1.shape)

        # Handle x0 for the first run
        if x0 is not None:
            print("x0 given: ", x0)
            runs[0] = np.hstack((x0[0:2].reshape(2,1), runs[0]))
            exit()

        # Compute path lengths 
        path_lengths = [np.sum(np.linalg.norm(run[:, 1:] - run[:, :-1], axis=0)) for run in runs]
        self.density = 1/abs(self.v_ref)/T # wp/m
        # self.density = 15
        self.region_of_acceptance = 0.05/10*self.density
        print("density: ", self.density, ", region_of_acceptance: ", self.region_of_acceptance)
        for i, length in enumerate(path_lengths):
            print(i, ") path length: ", length)
            runs[i] = interpolate_waypoints(runs[i].T, int(np.ceil(length*self.density)))
        # Combine all runs into a single set of waypoints
        self.waypoints = np.vstack(runs)
        print("waypoints: ", self.waypoints.shape)
        print("waypoints: ", np.around(self.waypoints, 2))
        self.waypoints = filter_waypoints(self.waypoints, 0.01).T
        # Calculate the total path length of the waypoints
        total_path_length = np.sum(np.linalg.norm(self.waypoints[:, 1:] - self.waypoints[:, :-1], axis=0))
        print("total path length: ", total_path_length)
        # self.waypoints = np.vstack((self.waypoints[1], self.waypoints[0])) #flip x and y
        # self.waypoints[0] = 15-self.waypoints[0] # flip x
        # self.waypoints[1] = 15-self.waypoints[1] # flip y
        self.waypoints_x = self.waypoints[0, :]
        self.waypoints_y = self.waypoints[1, :]
        
        self.x_max = 2
        self.x_min = -2
        self.y_max = 2
        self.y_min = -2

        self.num_waypoints = len(self.waypoints_x)
        print("num_waypoints: ", self.num_waypoints)
        self.kappa, self.wp_theta, self.wp_normals = compute_smooth_curvature(self.waypoints_x, self.waypoints_y)
        # linear speed profile
        self.v_refs  = self.v_ref / (1 + np.abs(self.kappa))*(1 + np.abs(self.kappa))
        #nonlinear speed profile
        # K=1
        # epsilon=1
        # self.v_refs = self.v_ref * (K/(abs(self.kappa) + epsilon))**0.5
        num_ramp_waypoints = int(1 * self.density) # 1 meters
        num_ramp_waypoints = min(num_ramp_waypoints, len(self.v_refs))
        # Linearly increase the reference speed from 0 to v_ref over the first 2 meters
        self.v_refs[-2:] = 0 # stop at the end
        # print("v_ref: ", self.v_ref)
        # print("vrefs: ", self.v_refs[0:100])
        k_steer = 0 #0.4/np.amax(np.abs(self.kappa))
        self.steer_ref = k_steer * self.kappa
        # Extend waypoints and reference values by N
        self.waypoints_x = np.pad(self.waypoints_x, (0,self.N+4), 'edge')
        self.waypoints_y = np.pad(self.waypoints_y, (0,self.N+4), 'edge')
        self.kappa = np.pad(self.kappa, (0,self.N+5), 'edge')
        self.wp_theta = np.pad(self.wp_theta, (0,self.N+5), 'edge') + np.pi
        self.wp_normals = np.pad(self.wp_normals, ((0,self.N+5),(0,0)), 'edge')
        self.v_refs = np.pad(self.v_refs, (0,self.N+5), 'edge')
        self.steer_ref = np.pad(self.steer_ref, (0,self.N+5), 'edge')
        self.state_refs = np.vstack((self.waypoints_x, self.waypoints_y, self.wp_theta[1:])).T
        self.input_refs = np.vstack((self.v_refs, self.steer_ref)).T
        self.waypoints = np.vstack((self.waypoints_x, self.waypoints_y)).T
        self.state_refs[:,2] = smooth_yaw_angles(self.state_refs[:,2])
        # self.state_refs[-4:,:] = np.array([0.844, 0.344, np.pi])
        plot_waypoints(self.state_refs, color='blue', label_suffix='')
        print("state_refs: ", self.state_refs.shape, ", input_refs: ", self.input_refs.shape)
        print("input refs: ", np.around(self.input_refs, 2))
        print("state refs: ", np.around(self.state_refs, 2))

    def change_lane(self, start_index, end_index, normals, shift_distance=0.36-0.1):
        self.state_refs[start_index:end_index,0] += normals[start_index:end_index, 0] * shift_distance
        self.state_refs[start_index:end_index,1] += normals[start_index:end_index, 1] * shift_distance
        return self.state_refs

    def desired_command_and_trajectory(self, index):
        return self.state_refs[index:index + self.N + 1], self.input_refs[index:index + self.N]

if __name__ == '__main__':
    path = Path(0.8, 100, 0.1, "parallel_park")
    print("shape: ", path.state_refs.shape)
    # shape:  (114, 3)
    state1 = path.state_refs[0]
    print("state1: ", state1)
    state2 = np.array([4, 4, np.pi])
    M = transformation_matrix(state1, state2)
    theta_diff = state2[2] - state1[2]
    transformed_state_refs = np.zeros_like(path.state_refs)
    for i in range(len(path.state_refs)):
        transformed_state_refs[i] = transform_pose(path.state_refs[i], M, theta_diff)
    plot_waypoints(transformed_state_refs, color='blue', label_suffix='')
