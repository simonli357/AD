#!/usr/bin/env python3

import time
import numpy as np
import os
from scipy.interpolate import UnivariateSpline, splprep, splev
from global_planner import GlobalPlanner
import yaml

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
    # Step 1: set up a spline interpolation
    t = np.linspace(0, 1, len(waypoints_x))
    spline_x = UnivariateSpline(t, waypoints_x, k=3, s=smooth_factor)
    spline_y = UnivariateSpline(t, waypoints_y, k=3, s=smooth_factor)
    # Step 2: get smoothed derivatives of the path
    t_smooth = np.linspace(0, 1, len(waypoints_x))
    dx_dt = spline_x.derivative()(t_smooth)
    dy_dt = spline_y.derivative()(t_smooth)
    ddx_dt = spline_x.derivative(n=2)(t_smooth)
    ddy_dt = spline_y.derivative(n=2)(t_smooth)
    # Step 3: compute curvature
    # κ = |dx/dt * d²y/dt² - dy/dt * d²x/dt²| / (dx/dt² + dy/dt²)^(3/2)
    curvature = np.abs(dx_dt * ddy_dt - dy_dt * ddx_dt) / (dx_dt**2 + dy_dt**2)**(3/2)
    # Step 4: compute tangent 
    tangent_angles = np.arctan2(dy_dt, dx_dt)
    # Step 5: compute normal
    normal_angles = tangent_angles + np.pi / 2
    # convert normal angles to vectors (dx, dy)
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
    def __init__(self, v_ref, N, T, x0=None, name="speedrun"):
        self.v_ref = v_ref
        self.N = N
        self.global_planner = GlobalPlanner()

        current_path = os.path.dirname(os.path.abspath(__file__))
        with open(os.path.join(current_path, 'config/paths.yaml'), 'r') as stream:
            data = yaml.safe_load(stream)
            destinations = data[name]

        # Plan runs between sequential destinations
        runs = []
        for i in range(len(destinations) - 1):
            start = self.global_planner.place_names[destinations[i]]
            end = self.global_planner.place_names[destinations[i+1]]
            run, _ = self.global_planner.plan_path(start, end)
            print("run: ", run.shape)
            runs.append(run)
        runs1 = np.hstack(runs)
        print("runs1: ", runs1.shape, "x0: ", x0)
        # runs1:  (2, 168) x0:  [3 2 0]
        #find closest index to x0
        if x0 is not None:
            def calculate_distances(run, x0):
                return np.sqrt(np.sum((run[:2, :] - x0[:2, None])**2, axis=0))
            # Find the closest point across all runs
            min_distance = np.inf
            min_distance_run_index = -1
            min_distance_point_index = -1

            for i, run in enumerate(runs):
                distances = calculate_distances(run, x0)
                min_index = np.argmin(distances)
                min_dist = distances[min_index]
                
                if min_dist < min_distance:
                    min_distance = min_dist
                    min_distance_run_index = i
                    min_distance_point_index = min_index

            # Now modify the list of runs as per the instructions
            if min_distance_run_index != -1:
                closest_run = runs[min_distance_run_index]
                # Append x0 to the closest run before the closest waypoint
                modified_run = np.hstack((closest_run[:, :min_distance_point_index], x0[:2, None], closest_run[:, min_distance_point_index:]))
                # Eliminate the waypoints before x0
                modified_run = modified_run[:, min_distance_point_index:]
                # Update the list of runs
                runs = [modified_run] + runs[min_distance_run_index+1:]

            # Show the results
            for i, run in enumerate(runs):
                print(f"run{i+1}: shape is {run.shape}")
                
        print("runs: ", len(runs))
        # Compute path lengths 
        path_lengths = [np.sum(np.linalg.norm(run[:, 1:] - run[:, :-1], axis=0)) for run in runs]
        self.density = 1/abs(self.v_ref)/T # wp/m
        self.region_of_acceptance = 0.05/10*self.density
        print("density: ", self.density, ", region_of_acceptance: ", self.region_of_acceptance)
        runs_hw = []
        runs_cw = []
        for i, length in enumerate(path_lengths):
            print(i, ") path length: ", length)
            runs_hw.append(interpolate_waypoints(runs[i].T, int(np.ceil(length*self.density/1.5))))
            runs_cw.append(interpolate_waypoints(runs[i].T, int(np.ceil(length*self.density*1.5))))
            runs[i] = interpolate_waypoints(runs[i].T, int(np.ceil(length*self.density)))
        # for run,i in zip(runs, range(len(runs))):
        #     np.savetxt(f"run{i+1}.txt", run, fmt="%.8f")
        # exit()
        # Combine all runs into a single set of waypoints
        self.waypoints = np.vstack(runs)
        self.waypoints_hw = np.vstack(runs_hw)
        self.waypoints_cw = np.vstack(runs_cw)
        print("waypoints: ", self.waypoints.shape)
        self.waypoints = filter_waypoints(self.waypoints, 0.01).T
        self.waypoints_hw = filter_waypoints(self.waypoints_hw, 0.01).T
        self.waypoints_cw = filter_waypoints(self.waypoints_cw, 0.01).T
        # Calculate the total path length of the waypoints
        total_path_length = np.sum(np.linalg.norm(self.waypoints[:, 1:] - self.waypoints[:, :-1], axis=0))
        print("total path length: ", total_path_length)
        
        # self.waypoints = np.vstack((self.waypoints[1], self.waypoints[0])) #flip x and y
        # self.waypoints[0] = 15-self.waypoints[0] # flip x
        # self.waypoints[1] = 15-self.waypoints[1] # flip y
        self.waypoints_x = self.waypoints[0, :]
        self.waypoints_y = self.waypoints[1, :]
        self.waypoints_x_hw = self.waypoints_hw[0, :]
        self.waypoints_y_hw = self.waypoints_hw[1, :]
        self.waypoints_x_cw = self.waypoints_cw[0, :]
        self.waypoints_y_cw = self.waypoints_cw[1, :]
        # filepath = os.path.dirname(os.path.abspath(__file__))
        # wpts = np.load(os.path.join(filepath, 'waypoints/parallel_park.npy'))
        # self.waypoints_x = wpts[:, 0]+5
        # self.waypoints_y = wpts[:, 1]+5

        self.num_waypoints = len(self.waypoints_x)
        print("num_waypoints: ", self.num_waypoints)
        self.kappa, self.wp_theta, self.wp_normals = compute_smooth_curvature(self.waypoints_x, self.waypoints_y)
        self.kappa_hw, self.wp_theta_hw, self.wp_normals_hw = compute_smooth_curvature(self.waypoints_x_hw, self.waypoints_y_hw)
        self.kappa_cw, self.wp_theta_cw, self.wp_normals_cw = compute_smooth_curvature(self.waypoints_x_cw, self.waypoints_y_cw)
        # linear speed profile
        self.v_refs  = self.v_ref / (1 + np.abs(self.kappa))*(1 + np.abs(self.kappa))
        #nonlinear speed profile
        # K=1
        # epsilon=1
        # self.v_refs = self.v_ref * (K/(abs(self.kappa) + epsilon))**0.5
        num_ramp_waypoints = int(1 * self.density) # 1 meters
        num_ramp_waypoints = min(num_ramp_waypoints, len(self.v_refs))
        # Linearly increase the reference speed from 0 to v_ref over the first 2 meters
        self.v_refs[:num_ramp_waypoints] = np.linspace(0, self.v_ref, num_ramp_waypoints)
        self.v_refs[-2:] = 0 # stop at the end
        # print("v_ref: ", self.v_ref)
        # print("vrefs: ", self.v_refs[0:100])
        k_steer = 0 #0.4/np.amax(np.abs(self.kappa))
        self.steer_ref = k_steer * self.kappa
        # Extend waypoints and reference values by N
        self.waypoints_x = np.pad(self.waypoints_x, (0,self.N+4), 'edge')
        self.waypoints_y = np.pad(self.waypoints_y, (0,self.N+4), 'edge')
        self.kappa = np.pad(self.kappa, (0,self.N+5), 'edge')
        self.wp_theta = np.pad(self.wp_theta, (0,self.N+5), 'edge')
        self.wp_normals = np.pad(self.wp_normals, ((0,self.N+5),(0,0)), 'edge')
        self.v_refs = np.pad(self.v_refs, (0,self.N+5), 'edge')
        self.steer_ref = np.pad(self.steer_ref, (0,self.N+5), 'edge')
        self.state_refs = np.vstack((self.waypoints_x, self.waypoints_y, self.wp_theta[1:])).T
        self.input_refs = np.vstack((self.v_refs, self.steer_ref)).T
        self.waypoints = np.vstack((self.waypoints_x, self.waypoints_y)).T
        self.state_refs[:,2] = smooth_yaw_angles(self.state_refs[:,2])

        self.waypoints_hw_x = np.pad(self.waypoints_x_hw, (0,self.N+4), 'edge')
        self.waypoints_hw_y = np.pad(self.waypoints_y_hw, (0,self.N+4), 'edge')
        self.kappa_hw = np.pad(self.kappa_hw, (0,self.N+5), 'edge')
        self.wp_theta_hw = np.pad(self.wp_theta_hw, (0,self.N+5), 'edge')
        self.wp_normals_hw = np.pad(self.wp_normals_hw, ((0,self.N+5),(0,0)), 'edge')
        self.waypoints_hw = np.vstack((self.waypoints_hw_x, self.waypoints_hw_y)).T
        self.state_refs_hw = np.vstack((self.waypoints_hw_x, self.waypoints_hw_y, self.wp_theta_hw[1:])).T
        self.state_refs_hw[:,2] = smooth_yaw_angles(self.state_refs_hw[:,2])

        self.waypoints_cw_x = np.pad(self.waypoints_x_cw, (0,self.N+4), 'edge')
        self.waypoints_cw_y = np.pad(self.waypoints_y_cw, (0,self.N+4), 'edge')
        self.kappa_cw = np.pad(self.kappa_cw, (0,self.N+5), 'edge')
        self.wp_theta_cw = np.pad(self.wp_theta_cw, (0,self.N+5), 'edge')
        self.wp_normals_cw = np.pad(self.wp_normals_cw, ((0,self.N+5),(0,0)), 'edge')
        self.waypoints_cw = np.vstack((self.waypoints_cw_x, self.waypoints_cw_y)).T
        self.state_refs_cw = np.vstack((self.waypoints_cw_x, self.waypoints_cw_y, self.wp_theta_cw[1:])).T
        self.state_refs_cw[:,2] = smooth_yaw_angles(self.state_refs_cw[:,2])

        # for i in range(len(self.state_refs)):
        #     print(i, self.state_refs[i,2])
        # exit()
        print("state_refs: ", self.state_refs.shape, ", input_refs: ", self.input_refs.shape)

    def change_lane(self, start_index, end_index, normals, shift_distance=0.36-0.1):
        self.state_refs[start_index:end_index,0] += normals[start_index:end_index, 0] * shift_distance
        self.state_refs[start_index:end_index,1] += normals[start_index:end_index, 1] * shift_distance
        return self.state_refs
