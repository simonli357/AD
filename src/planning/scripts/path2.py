#!/usr/bin/env python3

import numpy as np
import os
from scipy.interpolate import UnivariateSpline, splprep, splev
from global_planner import GlobalPlanner
import yaml
import math
import rospy
from std_msgs.msg import Float32MultiArray
from utils.srv import waypoints, waypointsResponse, go_to, go_to_multiple, go_toResponse, go_to_multipleResponse


def smooth_yaw_angles(yaw_angles):
    diffs = np.diff(yaw_angles)

    diffs[diffs > np.pi * 0.8] -= 2 * np.pi
    diffs[diffs < -np.pi * 0.8] += 2 * np.pi
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
    curvature = np.abs(dx_dt * ddy_dt - dy_dt * ddx_dt) / (dx_dt**2 + dy_dt**2)**(3 / 2)
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
        if np.linalg.norm(np.array(waypoints[i]) - np.array(waypoints[i - 1])) >= threshold:
            filtered_waypoints.append(waypoints[i])
        else:
            # print("filtered out waypoint: ",i, waypoints[i])
            continue
    return np.array(filtered_waypoints)


def filter_waypoints_and_attributes(waypoints, attributes, threshold):
    filtered_waypoints = [waypoints[0]]
    filtered_attributes = [attributes[0]]

    for i in range(1, len(waypoints)):
        if np.linalg.norm(np.array(waypoints[i]) - np.array(waypoints[i - 1])) >= threshold:
            filtered_waypoints.append(waypoints[i])
            filtered_attributes.append(attributes[i])
        else:
            # Optionally print or log the filtered out waypoints and attributes
            # print(f"Filtered out waypoint: {i}, {waypoints[i]} with attribute {attributes[i]}")
            continue

    return np.array(filtered_waypoints), np.array(filtered_attributes)


def interpolate_waypoints(waypoints, num_points):
    # Extract x and y coordinates
    x = waypoints[:, 0]
    y = waypoints[:, 1]

    # Calculate cumulative distances (arc-length)
    distances = np.sqrt(np.diff(x)**2 + np.diff(y)**2)
    cumulative_length = np.hstack(([0], np.cumsum(distances)))

    # Normalize to a range [0, 1]
    normalized_length = cumulative_length / cumulative_length[-1]

    # Fit the spline using the normalized arc-length as parameter
    try:
        tck, u = splprep([x, y], u=normalized_length, s=0)
    except Exception as e:
        points = np.vstack((x, y)).T
        print("error at: ", points)
        print("Error in splprep: ", e)
        return waypoints

    # Generate equally spaced parameter values in normalized arc-length
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


def replace_segments(waypoints, waypoints_cw, attributes, attributes_cw, density_factor=1.5, values=[1]):
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
    def __init__(self, v_ref, N, T, x0=None, name="speedrun", dest=None):
        self.hw_density_factor = rospy.get_param('hw', default=1.33)

        self.v_ref = v_ref
        print("v_ref: ", v_ref, ", N: ", N, ", T: ", T, ", x0: ", x0, ", name: ", name)
        self.N = N
        self.global_planner = GlobalPlanner()
        if name is not None:
            self.name = name
            current_path = os.path.dirname(os.path.abspath(__file__))
            with open(os.path.join(current_path, 'config/runs_mod2_custom.yaml'), 'r') as stream:
                data = yaml.safe_load(stream)
                destinations = data[name]
        else:
            destinations = []
            # check if dest is a list with x and y coordinates or a list containing a list with x and y coordinates
            if dest is not None and isinstance(dest[0], list):
                for d in dest:
                    dest_x = d[0]
                    dest_y = d[1]
                    destination = self.global_planner.find_closest_node(dest_x, dest_y)
                    destinations.append(destination)
            else:
                dest_x = dest[0]
                dest_y = dest[1]
                destination = self.global_planner.find_closest_node(dest_x, dest_y)
                print("destination: ", destination, type(destination))
                destinations = [destination]

        # Plan runs between sequential destinations
        runs = []
        attributes = []
        if x0 is not None:
            print("x0: ", x0)
            start = self.global_planner.find_closest_node(x0[0], x0[1])
            end = self.global_planner.get_node_number(destinations[0])
            run, _, attribute = self.global_planner.plan_path(start, end)
            runs.append(run)
            attributes.append(attribute)
        for i in range(len(destinations) - 1):
            start = self.global_planner.get_node_number(destinations[i])
            end = self.global_planner.get_node_number(destinations[i + 1])
            run, _, attribute = self.global_planner.plan_path(start, end)
            # if x0 is not None and i == 0:
            if False:
                runs[0] = np.hstack((runs[0], run))
                attributes[0].extend(attribute)
            else:
                # print("run: ", run.shape)
                runs.append(run)
                # print(i, ") attribute:\n", attribute)
                attributes.append(attribute)

        runs1 = np.hstack(runs)

        # Compute path lengths
        path_lengths = [np.sum(np.linalg.norm(run[:, 1:] - run[:, :-1], axis=0)) for run in runs]
        self.density = 1 / abs(self.v_ref) / T  # wp/m
        self.region_of_acceptance = 0.05 * 10 / self.density
        # print("density: ", self.density, ", region_of_acceptance: ", self.region_of_acceptance)

        runs_hw = []
        runs_cw = []
        attributes_hw = []
        attributes_cw = []
        n = len(runs)
        for i in range(n):
            if i >= n:
                break
            run = runs[i]
            if run.shape[1] < 4:
                if i == len(runs) - 1:
                    # if last, append to the previous run
                    # remove the first element of run
                    run = run[:, 1:]
                    runs[i - 1] = np.hstack((runs[i - 1], run))
                    attributes[i - 1] = attributes[i - 1] + attributes[i]
                    path_lengths[i - 1] += path_lengths[i]
                    runs.pop(i)
                    attributes.pop(i)
                    path_lengths.pop(i)
                    break
                else:
                    # if has less than 4 waypoints, append to the next run
                    # remove the last element of run
                    run = run[:, :-1]
                    runs[i + 1] = np.hstack((run, runs[i + 1]))
                    attributes[i + 1] = attributes[i] + attributes[i + 1]
                    path_lengths[i + 1] += path_lengths[i]
                    runs.pop(i)
                    attributes.pop(i)
                    path_lengths.pop(i)
                    i -= 1
                    n -= 1
                i += 1
        for i, length in enumerate(path_lengths):
            # print(i, ") path length: ", length)
            runs_hw.append(interpolate_waypoints(runs[i].T, int(np.ceil(length * self.density / self.hw_density_factor))))
            runs_cw.append(interpolate_waypoints(runs[i].T, int(np.ceil(length * self.density * 1.5))))
            old_run = runs[i].copy()
            runs[i] = interpolate_waypoints(runs[i].T, int(np.ceil(length * self.density)))
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

        if name != "speedrun":
            segments_cw = find_segments(self.attributes, values=[1, 9, 101, 109])
            segments_cw_with_attributes = []
            for segment in segments_cw:
                segment = list(segment)  # Convert tuple to list
                segment.append(1)
                segments_cw_with_attributes.append(segment)
            segments_hw = find_segments(self.attributes, values=[4, 5, 104, 105])
            segments_hw_with_attributes = []
            for segment in segments_hw:
                segment = list(segment)  # Convert tuple to list
                segment.append(4)
                segments_hw_with_attributes.append(segment)
            # sort every segment in segments_cw and segments_hw by segment[0], combine into one list
            segments = sorted(segments_cw_with_attributes + segments_hw_with_attributes, key=lambda x: x[0])
            self.segments = segments
            # print("starts: ", [segment[0] for segment in segments])
            # print("ends: ", [segment[1] for segment in segments])

            starts_cw = []
            ends_cw = []
            # for i in range(len(segments)):
            # start, end, attribute = segments[i]
            for segment in segments:
                start, end, attribute = segment
                if attribute == 1:
                    density_factor = 1.5
                elif attribute == 4 or attribute == 5:
                    density_factor = 1 / self.hw_density_factor
                starts_cw.append(int(math.floor(start * density_factor)))
                ends_cw.append(int(math.ceil(end * density_factor)))
            self.starts_cw = starts_cw
            self.ends_cw = ends_cw
            # print("starts_cw: ", starts_cw) # good
            # print("ends_cw: ", ends_cw) # good

            starts = []
            ends = []
            for i in range(len(segments)):
                start, end, attribute = segments[i]
                starts.append(start)
                ends.append(end)
            self.starts = starts
            self.ends = ends

            new_waypoints = []
            new_attributes = []
            i = 0
            last_end = 0
            self.true_starts = []
            self.true_ends = []
            for i in range(len(segments)):
                # for i in range(2):
                start = starts[i]
                end = ends[i]
                # Add non-replaced segments to waypoints and attributes
                new_waypoints.extend(self.waypoints[last_end:start])
                new_attributes.extend(self.attributes[last_end:start])

                start_cw = starts_cw[i]
                end_cw = ends_cw[i]

                true_start = start
                self.true_starts.append(true_start)
                # if i == 1:
                #     break
                # Replace the segment in 'waypoints' and 'attributes' with the corresponding segment in 'waypoints_cw' and 'attributes_cw'
                attribute = segments[i][2]
                if attribute == 1:
                    wpts = self.waypoints_cw
                    attrs = self.attributes_cw
                else:
                    wpts = self.waypoints_hw
                    attrs = self.attributes_hw
                new_waypoints.extend(wpts[start_cw:end_cw])
                new_attributes.extend(attrs[start_cw:end_cw])

                last_end = end

                true_end = start + end_cw - start_cw
                self.true_ends.append(start + end_cw - start_cw)

            # Add the remaining part of the run after the last segment to waypoints and attributes
            new_waypoints.extend(self.waypoints[last_end:])
            new_attributes.extend(self.attributes[last_end:])
            self.waypoints = np.array(new_waypoints)
            self.attributes = np.array(new_attributes)

        # self.waypoints = filter_waypoints(self.waypoints, 0.01).T
        self.waypoints, self.attributes = filter_waypoints_and_attributes(self.waypoints, self.attributes, 0.01)
        self.waypoints = self.waypoints.T

        # Calculate the total path length of the waypoints
        total_path_length = np.sum(np.linalg.norm(self.waypoints[:, 1:] - self.waypoints[:, :-1], axis=0))
        print("total path length: ", total_path_length)

        self.waypoints_x = self.waypoints[0, :]
        self.waypoints_y = self.waypoints[1, :]

        self.num_waypoints = len(self.waypoints_x)
        # print("num_waypoints: ", self.num_waypoints)
        self.kappa, self.wp_theta, self.wp_normals = compute_smooth_curvature(self.waypoints_x, self.waypoints_y)
        # linear speed profile
        self.v_refs = self.v_ref / (1 + np.abs(self.kappa)) * (1 + np.abs(self.kappa))
        mask_cw = self.attributes == 1
        mask_hw1 = (self.attributes == 4)
        mask_hw2 = (self.attributes == 5)
        self.v_refs[mask_cw] *= 1 / 1.5
        self.v_refs[mask_hw1] *= self.hw_density_factor
        self.v_refs[mask_hw2] *= self.hw_density_factor
        # print("v_refs: \n", self.v_refs, ", wpts: ", self.waypoints.shape, ", attributes: ", self.attributes.shape)

        # nonlinear speed profile
        # K=1
        # epsilon=1
        # self.v_refs = self.v_ref * (K/(abs(self.kappa) + epsilon))**0.5
        num_ramp_waypoints = int(1 * self.density)  # 1 meters
        num_ramp_waypoints = min(num_ramp_waypoints, len(self.v_refs))
        # Linearly increase the reference speed from 0 to v_ref over the first 1 meters
        self.v_refs[:num_ramp_waypoints] = np.linspace(0, self.v_ref, num_ramp_waypoints)
        self.v_refs[-2:] = 0  # stop at the end
        # print("v_ref: ", self.v_ref)
        # print("vrefs: ", self.v_refs[0:100])
        k_steer = 0  # 0.4/np.amax(np.abs(self.kappa))
        self.steer_ref = k_steer * self.kappa
        # self.steer_ref = np.arctan(0.27 * self.kappa)
        # Extend waypoints and reference values by N
        self.waypoints_x = np.pad(self.waypoints_x, (0, self.N + 4), 'edge')
        self.waypoints_y = np.pad(self.waypoints_y, (0, self.N + 4), 'edge')
        self.kappa = np.pad(self.kappa, (0, self.N + 5), 'edge')
        self.wp_theta = np.pad(self.wp_theta, (0, self.N + 5), 'edge')
        self.wp_normals = np.pad(self.wp_normals, ((0, self.N + 5), (0, 0)), 'edge')
        self.v_refs = np.pad(self.v_refs, (0, self.N + 5), 'edge')
        self.steer_ref = np.pad(self.steer_ref, (0, self.N + 5), 'edge')
        self.state_refs = np.vstack((self.waypoints_x, self.waypoints_y, self.wp_theta[1:])).T
        self.input_refs = np.vstack((self.v_refs, self.steer_ref)).T
        self.waypoints = np.vstack((self.waypoints_x, self.waypoints_y)).T
        self.state_refs[:, 2] = smooth_yaw_angles(self.state_refs[:, 2])

        self.attributes = np.pad(self.attributes, (0, self.N + 5), 'edge')

        # print(self.attributes.shape, self.attributes)

        # for i in range(len(self.state_refs)):
        #     print(i, self.state_refs[i])
        # exit()
        # print("state_refs: ", self.state_refs.shape, ", input_refs: ", self.input_refs.shape)

    def change_lane(self, start_index, end_index, normals, shift_distance=0.36 - 0.1):
        self.state_refs[start_index:end_index, 0] += normals[start_index:end_index, 0] * shift_distance
        self.state_refs[start_index:end_index, 1] += normals[start_index:end_index, 1] * shift_distance
        return self.state_refs

    def illustrate_path(self, state_refs):
        # import matplotlib.pyplot as plt
        # print("shape: ", state_refs.shape)
        # plt.plot(state_refs[0,:], state_refs[1,:], 'b-')
        # plt.show()

        import cv2
        self.map = cv2.imread(os.path.dirname(os.path.realpath(__file__)) + '/maps/Track.png')
        size1 = 1000
        self.map = cv2.resize(self.map, (size1, int(1 / 1.38342246 * size1)))
        for i in range(0, state_refs.shape[1], 8):
            radius = 2
            color = (0, 255, 255)
            if self.attributes[i] == 4 or self.attributes[i] == 5:  # hard waypoints
                color = (0, 0, 255)
            if self.attributes[i] == 1:  # crosswalk
                color = (0, 255, 0)  # green
            if self.attributes[i] == 9:  # color is red
                color = (255, 0, 0)
            if self.attributes[i] == 7:  # color is yellow
                color = (255, 255, 0)
            if self.attributes[i] == 6:  # color is white
                color = (255, 255, 255)
            if self.attributes[i] >= 100:  # orange
                color = (0, 165, 255)
            if self.attributes[i] == 2 or self.attributes[i] == 102:  # color is purple
                color = (255, 0, 255)
            cv2.circle(self.map, (int(state_refs[0, i] / 20.696 * self.map.shape[1]), int((13.786 - state_refs[1, i]) / 13.786 * self.map.shape[0])), radius=int(radius), color=color, thickness=-1)
        cv2.imshow('map357', self.map)
        cv2.waitKey(0)


def handle_goto_service(req):
    current_path = os.path.dirname(os.path.realpath(__file__))
    vrefName = req.vrefName
    config_path = 'config/mpc_config' + vrefName + '.yaml'
    path = os.path.join(current_path, config_path)
    with open(path, 'r') as f:
        config = yaml.safe_load(f)
    T = config['T']
    N = config['N']
    constraint_name = 'constraints'

    if req.x0 <= -1 or req.y0 <= -1:
        initial_state = None
    else:
        initial_state = np.array([req.x0, req.y0, req.yaw0])

    v_ref = config[constraint_name]['v_ref']
    # print(f"v_ref: {v_ref}, N: {N}, T: {T}, x0: {initial_state}, dest: [{req.dest_x}, {req.dest_y}]")
    path = Path(v_ref=v_ref, N=N, T=T, x0=initial_state, name=None, dest=[req.dest_x, req.dest_y])

    # path.illustrate_path(path.state_refs.T)
    state_refs = Float32MultiArray(data=path.state_refs.flatten())
    input_refs = Float32MultiArray(data=path.input_refs.flatten())
    attributes = Float32MultiArray(data=path.attributes.flatten())
    normals = Float32MultiArray(data=path.wp_normals.flatten())

    return go_toResponse(state_refs, input_refs, attributes, normals)


def handle_goto_multiple_service(req):
    current_path = os.path.dirname(os.path.realpath(__file__))
    vrefName = req.vrefName
    config_path = 'config/mpc_config' + vrefName + '.yaml'
    path = os.path.join(current_path, config_path)
    with open(path, 'r') as f:
        config = yaml.safe_load(f)
    T = config['T']
    N = config['N']
    constraint_name = 'constraints'

    if req.x0 <= -1 or req.y0 <= -1:
        initial_state = None
    else:
        initial_state = np.array([req.x0, req.y0, req.yaw0])

    destinations = [[point.x, point.y] for point in req.destinations]

    v_ref = config[constraint_name]['v_ref']
    # print(f"v_ref: {v_ref}, N: {N}, T: {T}, x0: {initial_state}, dest: [{req.dest_x}, {req.dest_y}]")
    path = Path(v_ref=v_ref, N=N, T=T, x0=initial_state, name=None, dest=destinations)

    # path.illustrate_path(path.state_refs.T)
    state_refs = Float32MultiArray(data=path.state_refs.flatten())
    input_refs = Float32MultiArray(data=path.input_refs.flatten())
    attributes = Float32MultiArray(data=path.attributes.flatten())
    normals = Float32MultiArray(data=path.wp_normals.flatten())

    return go_to_multipleResponse(state_refs, input_refs, attributes, normals)


def handle_array_service(req):
    """
    Service callback function to return numpy arrays a, b, and c.
    """
    current_path = os.path.dirname(os.path.realpath(__file__))
    vrefName = req.vrefName
    config_path = 'config/mpc_config' + vrefName + '.yaml'
    # print("config_path: ", config_path)
    path = os.path.join(current_path, config_path)
    with open(path, 'r') as f:
        config = yaml.safe_load(f)
    T = config['T']
    N = config['N']
    constraint_name = 'constraints'

    if req.x0 <= -1 or req.y0 <= -1:
        initial_state = None
    else:
        initial_state = np.array([req.x0, req.y0, req.yaw0])
    # initial_state = None

    v_ref = config[constraint_name]['v_ref']
    # print("v_reeeeeeeeeeeeeeefffffffffff: ", v_ref)

    print("request received: ", req.pathName, ", x0: ", initial_state)
    path = Path(v_ref=v_ref, N=N, T=T, x0=initial_state, name=req.pathName)

    # path.illustrate_path(path.state_refs.T)
    state_refs = Float32MultiArray(data=path.state_refs.flatten())
    input_refs = Float32MultiArray(data=path.input_refs.flatten())
    attributes = Float32MultiArray(data=path.attributes.flatten())
    normals = Float32MultiArray(data=path.wp_normals.flatten())

    # print("sizes: ", len(state_refs.data), len(input_refs.data), len(attributes.data), len(normals.data))
    # import threading
    # threading.Thread(target=initiate_shutdown).start()
    return waypointsResponse(state_refs, input_refs, attributes, normals)


def initiate_shutdown():
    """
    Initiates node shutdown with a short delay to ensure service response is sent.
    """
    rospy.sleep(1)  # Short delay
    rospy.signal_shutdown("Service request processed. Shutting down.")


def visualize_waypoints2(waypoints):
    import matplotlib.pyplot as plt
    import numpy as np

    # Extract coordinates and orientations
    x = waypoints[:, 0]
    y = waypoints[:, 1]
    yaw = waypoints[:, 2]

    # Create figure with larger size
    plt.figure(figsize=(12, 12))

    # Plot complete path with semi-transparent line
    plt.plot(x, y, color='royalblue', alpha=0.4, linewidth=2, label='Path')

    # Highlight every 8th waypoint with markers and arrows
    step = 8
    highlight_x = x[::step]
    highlight_y = y[::step]
    highlight_yaw = yaw[::step]

    # Draw main waypoint markers
    plt.scatter(highlight_x, highlight_y, s=80, c='deepskyblue',
                edgecolors='navy', linewidths=0.5, marker='o',
                label='Key Waypoints', zorder=3)

    # Add orientation arrows
    for xi, yi, yiaw in zip(highlight_x, highlight_y, highlight_yaw):
        dx = np.cos(yiaw) * 0.6  # Increased arrow length
        dy = np.sin(yiaw) * 0.6
        plt.arrow(xi, yi, dx, dy, head_width=0.4, head_length=0.4,
                  fc='crimson', ec='darkred', width=0.05, zorder=4)

    # Emphasize start and end points
    start_marker = dict(color='limegreen', marker='s', s=250,
                        edgecolor='darkgreen', linewidth=2, zorder=5)
    end_marker = dict(color='orangered', marker='X', s=250,
                      edgecolor='darkred', linewidth=2, zorder=5)

    plt.scatter(x[0], y[0], **start_marker, label='Start')
    plt.scatter(x[-1], y[-1], **end_marker, label='End')

    # Add text annotations for start/end
    plt.annotate('START', (x[0], y[0]),
                 textcoords="offset points", xytext=(10, -5),
                 ha='left', color='darkgreen', fontweight='bold')
    plt.annotate('END', (x[-1], y[-1]),
                 textcoords="offset points", xytext=(10, -5),
                 ha='left', color='darkred', fontweight='bold')

    # Configure plot aesthetics
    plt.title('Enhanced Waypoint Visualization', fontsize=14, pad=20)
    plt.xlabel('X Coordinate', fontweight='bold')
    plt.ylabel('Y Coordinate', fontweight='bold')
    plt.legend(loc='upper right', framealpha=0.9)
    plt.grid(True, color='gainsboro', linestyle='--')
    plt.axis('equal')
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    rospy.init_node('waypointPathServer')
    s = rospy.Service('waypoint_path', waypoints, handle_array_service)
    rospy.loginfo("waypoint_path service is ready.")
    goto_service = rospy.Service('go_to', go_to, handle_goto_service)
    goto_multiple = rospy.Service('go_to_multiple', go_to_multiple, handle_goto_multiple_service)
    rospy.loginfo("go_to service is ready.")
    # global hw_density_factor
    # hw_density_factor = rospy.get_param('hw', default=1.33)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        # rospy.spin()
        rate.sleep()

    # current_dir = os.path.dirname(os.path.abspath(__file__))
    # print("current_dir: ", current_dir)
    # config_path=os.path.join(current_dir, 'config/mpc_config25.yaml')
    # print("config_path: ", config_path)
    # path = config_path
    # with open(path, 'r') as f:
    #     config = yaml.safe_load(f)
    # T = config['T']
    # N = config['N']
    # constraint_name = 'constraints'
    # cost_name = 'costs'
    # t_horizon = T * N

    # v_ref = config[constraint_name]['v_ref']
    # print("v_ref: ", v_ref)
    # x0 = np.array([0.35,2.726,-1.5708])
    # name = "run221"
    # path = Path(v_ref = v_ref, N = N, T = T, x0= x0, name = name)
    # np.savetxt(os.path.join(current_dir,'state_refs.txt'), path.state_refs, fmt='%.4f')
    # visualize_waypoints2(path.state_refs)
