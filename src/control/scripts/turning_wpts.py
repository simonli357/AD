#!/usr/bin/env python3

import time
import numpy as np
import os
from scipy.interpolate import UnivariateSpline, splprep, splev
from global_planner2 import GlobalPlanner
import yaml
import math

def smooth_yaw_angles(yaw_angles):
    # Calculate the differences between adjacent angles
    diffs = np.diff(yaw_angles)

    # Print indices and values of diffs where abs(diffs) > pi/2
    # for i, diff in enumerate(diffs):
    #     if abs(diff) > np.pi*0.75 and abs(diff) < np.pi*1.5:
    #         # print(f"diffs[{i}]: {diff:.3f}, yaw_angles[{i}]: {yaw_angles[i]:.3f}, yaw_angles[{i+1}]: {yaw_angles[i+1]:.3f}")
    #         yaw_angles[i+1] = (yaw_angles[i])
    #         diffs = np.diff(yaw_angles)

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
def filter_waypoints_and_attributes(waypoints, attributes, threshold):
    filtered_waypoints = [waypoints[0]]
    filtered_attributes = [attributes[0]]

    for i in range(1, len(waypoints)):
        if np.linalg.norm(np.array(waypoints[i]) - np.array(waypoints[i-1])) >= threshold:
            filtered_waypoints.append(waypoints[i])
            filtered_attributes.append(attributes[i])
        else:
            # Optionally print or log the filtered out waypoints and attributes
            # print(f"Filtered out waypoint: {i}, {waypoints[i]} with attribute {attributes[i]}")
            continue

    return np.array(filtered_waypoints), np.array(filtered_attributes)

def interpolate_waypoints(waypoints, num_points):
    # print("num_points: ", num_points)
    x = waypoints[:, 0]
    y = waypoints[:, 1]
    tck, u = splprep([x, y], s=0) 
    # Generate set of equally spaced waypoints
    u_new = np.linspace(0, 1, num_points)
    x_new, y_new = splev(u_new, tck) 
    # Stack the x and y coordinates to get new waypoints
    new_waypoints = np.vstack((x_new, y_new)).T
    return new_waypoints
def interpolate_attributes(waypoints, attributes, new_waypoints):
    num_new = len(new_waypoints)
    new_attributes = np.zeros(num_new, dtype=int)

    for i in range(num_new):
        # Calculate the distance from the new waypoint to all original waypoints
        distances = np.linalg.norm(waypoints - new_waypoints[i], axis=1)
        # Find the index of the nearest original waypoint
        nearest_index = np.argmin(distances)
        # Assign its attribute to the new waypoint
        new_attributes[i] = attributes[nearest_index]

    return new_attributes
def replace_segments(waypoints, waypoints_cw, attributes, attributes_cw, density_factor=1.5, values = [1]):
    # Find segments where attribute is 1 in 'attributes'
    segments = find_segments(attributes, values=values)

    new_waypoints = []
    new_attributes = []
    last_end = 0
    for seg in segments:
        start, end = seg
        # Add non-replaced segments to waypoints and attributes
        new_waypoints.extend(waypoints[last_end:start])
        new_attributes.extend(attributes[last_end:start])

        # Calculate corresponding segment in 'waypoints_cw' and 'attributes_cw'
        start_cw = int(math.floor(start * density_factor))
        end_cw = int(math.ceil(end * density_factor))

        # Replace the segment in 'waypoints' and 'attributes' with the corresponding segment in 'waypoints_cw' and 'attributes_cw'
        new_waypoints.extend(waypoints_cw[start_cw:end_cw])
        new_attributes.extend(attributes_cw[start_cw:end_cw])

        last_end = end

    # Add the remaining part of the run after the last segment to waypoints and attributes
    new_waypoints.extend(waypoints[last_end:])
    new_attributes.extend(attributes[last_end:])

    return np.array(new_waypoints), np.array(new_attributes)

def find_segments(array, values):
    """ Find start and end indices of segments in an array where the value matches. """
    segments = []
    start = None
    for i, val in enumerate(array):
        if val in values and start is None:
            start = i
        elif val not in values and start is not None:
            segments.append((start, i))
            start = None
    if start is not None:
        segments.append((start, len(array)))
    return segments

def interpolate_waypoints2(waypoints, num_points):
    # Initialize an array to hold the new interpolated waypoints
    new_waypoints = np.zeros((num_points, 2))

    # Calculate the total number of segments
    num_segments = len(waypoints) - 1

    # Calculate the number of points per segment
    points_per_segment = num_points // num_segments

    # Interpolate points for each segment
    for i in range(num_segments):
        start = waypoints[i]
        end = waypoints[i + 1]
        # For each segment, interpolate linearly between the start and end
        for j in range(points_per_segment):
            t = j / points_per_segment
            new_waypoints[i * points_per_segment + j] = (1 - t) * start + t * end

    # Handle any remaining points (due to integer division)
    for i in range(num_points - points_per_segment * num_segments):
        new_waypoints[-i - 1] = waypoints[-1]

    return new_waypoints

class Path:
    def __init__(self, v_ref, N, T, x0=None, name="path3"):
        self.v_ref = v_ref
        self.N = N
        self.global_planner = GlobalPlanner()

        current_path = os.path.dirname(os.path.abspath(__file__))
        with open(os.path.join(current_path, 'config/paths.yaml'), 'r') as stream:
            data = yaml.safe_load(stream)
            destinations = data[name]

        # Plan runs between sequential destinations
        runs = []
        attributes = []
        # for i in range(len(destinations) - 1):
        for i in range(1):
            start = 14
            # end = 131 #left
            end = 97 #right
            start = 31 # straight
            end = 126
            run, _, attribute = self.global_planner.plan_path(start, end)
            # print("run: ", run.shape)
            runs.append(run)
            # print(i, ") attribute:\n", attribute)
            attributes.append(attribute)
        runs1 = np.hstack(runs)
        # print("runs1: ", runs1.shape, "x0: ", x0)
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
            # for i, run in enumerate(runs):
            #     print(f"run{i+1}: shape is {run.shape}")
                
        # print("runs: ", len(runs))
        # Compute path lengths 
        path_lengths = [np.sum(np.linalg.norm(run[:, 1:] - run[:, :-1], axis=0)) for run in runs]
        self.density = 1/abs(self.v_ref)/T # wp/m
        self.region_of_acceptance = 0.05*10/self.density
        # print("density: ", self.density, ", region_of_acceptance: ", self.region_of_acceptance)

        runs_hw = []
        runs_cw = []
        attributes_hw = []
        attributes_cw = []
        for i, length in enumerate(path_lengths):
            # print(i, ") path length: ", length)
            runs_hw.append(interpolate_waypoints(runs[i].T, int(np.ceil(length*self.density/1.33))))
            runs_cw.append(interpolate_waypoints(runs[i].T, int(np.ceil(length*self.density*1.5))))
            old_run = runs[i].copy()
            runs[i] = interpolate_waypoints(runs[i].T, int(np.ceil(length*self.density)))
            # print("old shape: ", old_run.shape, ", new shape: ", runs[i].shape)
            attributes_cw.append(interpolate_attributes(old_run.T, attributes[i], runs_cw[i]))
            attributes_hw.append(interpolate_attributes(old_run.T, attributes[i], runs_hw[i]))
            attributes[i] = interpolate_attributes(old_run.T, attributes[i], runs[i])
        
        # print("run1: \n", runs[0].shape)
        # print("attr1: \n", attributes[0].shape)
        # print("run_hw1: \n", runs_hw[0].shape)
        # print("attr_hw1: \n", attributes_hw[0].shape)
        # print("attr1: \n", attributes[0])

        # for run,i in zip(runs, range(len(runs))):
        #     np.savetxt(f"run{i+1}.txt", run, fmt="%.8f")
        # exit()
        # Combine all runs into a single set of waypoints
        self.waypoints = np.vstack(runs)
        self.waypoints_hw = np.vstack(runs_hw)
        self.waypoints_cw = np.vstack(runs_cw)
        self.attributes = np.hstack(attributes)
        self.attributes_hw = np.hstack(attributes_hw)
        self.attributes_cw = np.hstack(attributes_cw)

        self.waypoints, self.attributes = replace_segments(
            self.waypoints, 
            self.waypoints_cw, 
            self.attributes, 
            self.attributes_cw
        )
        self.waypoints, self.attributes = replace_segments(
            self.waypoints, 
            self.waypoints_hw, 
            self.attributes, 
            self.attributes_hw,
            density_factor= 1/1.33,
            values = [4,5]
        )
        # self.waypoints = filter_waypoints(self.waypoints, 0.01).T
        self.waypoints, self.attributes = filter_waypoints_and_attributes(self.waypoints, self.attributes, 0.01)
        self.waypoints = self.waypoints.T

        # Calculate the total path length of the waypoints
        total_path_length = np.sum(np.linalg.norm(self.waypoints[:, 1:] - self.waypoints[:, :-1], axis=0))
        # print("total path length: ", total_path_length)
        
        self.waypoints_x = self.waypoints[0, :]
        self.waypoints_y = self.waypoints[1, :]

        self.num_waypoints = len(self.waypoints_x)
        # print("num_waypoints: ", self.num_waypoints)
        self.kappa, self.wp_theta, self.wp_normals = compute_smooth_curvature(self.waypoints_x, self.waypoints_y)
        # linear speed profile
        self.v_refs  = self.v_ref / (1 + np.abs(self.kappa))*(1 + np.abs(self.kappa))
        mask_cw = self.attributes == 1
        mask_hw1 = (self.attributes == 4)
        mask_hw2 = (self.attributes == 5)
        self.v_refs[mask_cw] *= 1/1.5
        self.v_refs[mask_hw1] *= 1.33
        self.v_refs[mask_hw2] *= 1.33
        # print("v_refs: \n", self.v_refs, ", wpts: ", self.waypoints.shape, ", attributes: ", self.attributes.shape)
        
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
        
        self.attributes = np.pad(self.attributes, (0,self.N+5), 'edge')

        # print(self.attributes.shape, self.attributes)

        # for i in range(len(self.state_refs)):
        #     print(i, self.state_refs[i])
        # exit()
        # print("state_refs: ", self.state_refs.shape, ", input_refs: ", self.input_refs.shape)

    def change_lane(self, start_index, end_index, normals, shift_distance=0.36-0.1):
        self.state_refs[start_index:end_index,0] += normals[start_index:end_index, 0] * shift_distance
        self.state_refs[start_index:end_index,1] += normals[start_index:end_index, 1] * shift_distance
        return self.state_refs

    def illustrate_path(self, state_refs):
        import matplotlib.pyplot as plt
        print("shape: ", state_refs.shape)
        plt.plot(state_refs[0,:], state_refs[1,:], 'b-')
        plt.show()

if __name__ == "__main__":
    path = Path(v_ref = 1.3, N=40, T=0.125)
