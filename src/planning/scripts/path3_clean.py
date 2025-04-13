#!/usr/bin/env python3

import rospy
from utils.srv import waypoints, waypointsResponse, go_to, go_to_multiple, go_toResponse, go_to_multipleResponse
from std_msgs.msg import Float32MultiArray

import numpy as np
import os
import math
import yaml
from scipy.interpolate import interp1d
from global_planner import GlobalPlanner

def smooth_yaw_angles(yaw_angles):
    """
    Fix discontinuities in a sequence of yaw angles by correcting angle wrap-around.
    """
    diffs = np.diff(yaw_angles)
    diffs[diffs > np.pi * 0.8] -= 2 * np.pi
    diffs[diffs < -np.pi * 0.8] += 2 * np.pi
    smooth_yaw = np.concatenate(([yaw_angles[0]], yaw_angles[0] + np.cumsum(diffs)))
    return smooth_yaw

def compute_curvature_and_tangent(points):
    """
    Compute curvature, tangent angles, and normals using finite differences.
    :param points: Array of shape (n, 2) with x, y coordinates.
    :return: curvature (n,), tangent angles (n,), and normal vectors (n,2)
    """
    x = points[:, 0]
    y = points[:, 1]
    dx = np.gradient(x)
    dy = np.gradient(y)
    ddx = np.gradient(dx)
    ddy = np.gradient(dy)
    denom = (dx**2 + dy**2)**1.5
    denom[denom == 0] = np.finfo(float).eps
    curvature = np.abs(dx * ddy - dy * ddx) / denom
    tangent_angles = np.arctan2(dy, dx)
    normals = np.stack((np.cos(tangent_angles + np.pi/2), np.sin(tangent_angles + np.pi/2)), axis=-1)
    return curvature, tangent_angles, normals

def linear_interpolate_waypoints(points, num_points):
    """
    Interpolate points along a path using linear interpolation.
    :param points: Array of shape (n,2) of original (x,y) waypoints.
    :param num_points: The desired number of interpolated waypoints.
    :return: Array of shape (num_points,2) of interpolated waypoints.
    """
    # Compute arc-length of the path.
    segs = np.diff(points, axis=0)
    seg_lengths = np.linalg.norm(segs, axis=1)
    cum_length = np.concatenate(([0], np.cumsum(seg_lengths)))
    total_length = cum_length[-1]
    
    # New equally spaced distances along the arc-length.
    new_dists = np.linspace(0, total_length, num_points)
    interp_fx = interp1d(cum_length, points[:, 0])
    interp_fy = interp1d(cum_length, points[:, 1])
    new_x = interp_fx(new_dists)
    new_y = interp_fy(new_dists)
    return np.column_stack((new_x, new_y))

def interpolate_attributes_linear(original_points, original_attrs, new_points):
    """
    For each new waypoint, assign the attribute of the nearest original point.
    """
    new_attrs = []
    for p in new_points:
        dists = np.linalg.norm(original_points - p, axis=1)
        new_attrs.append(original_attrs[np.argmin(dists)])
    return np.array(new_attrs)

def filter_duplicate_points(points, attributes, tol=1e-6):
    """
    Remove consecutive (or nearly identical) duplicate points.
    """
    if len(points) == 0:
        return points, attributes
    filtered_pts = [points[0]]
    filtered_attrs = [attributes[0]]
    for i in range(1, len(points)):
        if np.linalg.norm(points[i] - filtered_pts[-1]) > tol:
            filtered_pts.append(points[i])
            filtered_attrs.append(attributes[i])
    return np.array(filtered_pts), np.array(filtered_attrs)

def to_xy_points(run):
    """
    Convert any run to have a shape (n_points, 2) representing (x, y) coordinates.
    If the run is given as a 2 x n_points array, it transposes it.
    If there are extra columns, only the first two are kept.
    """
    run = np.array(run)
    if run.ndim != 2:
        raise ValueError("run must be a 2D array")
    # If it is 2 x n_points, then transpose.
    if run.shape[0] == 2 and run.shape[1] != 2:
        run = run.T
    # If more than 2 columns, take only the first two.
    if run.shape[1] > 2:
        run = run[:, :2]
    return run

class Path:
    def __init__(self, v_ref, N, T, x0=None, name="speedrun", dest=None):
        use_spline = False
        self.hw_density_factor = rospy.get_param('hw', default=1.33)
        self.v_ref = v_ref
        self.N = N
        self.T = T
        self.global_planner = GlobalPlanner()
        
        # Load destination nodes.
        if dest is not None:
            # Process the provided destination(s)
            destinations = []
            if isinstance(dest[0], list):
                for d in dest:
                    destination = self.global_planner.find_closest_node(d[0], d[1])
                    destinations.append(destination)
            else:
                destination = self.global_planner.find_closest_node(dest[0], dest[1])
                destinations = [destination]
        else:
            # Otherwise, load from YAML if a valid name is provided.
            current_path = os.path.dirname(os.path.abspath(__file__))
            yaml_path = os.path.join(current_path, 'config', 'runs0412_modified.yaml')
            with open(yaml_path, 'r') as stream:
                data = yaml.safe_load(stream)
            destinations = data.get(name, [])
        print("destinations: ", destinations)
        # exit()            
        # Plan runs (segments) between sequential destination nodes.
        runs = []
        run_attrs = []
        if x0 is not None:
            start = self.global_planner.find_closest_node(x0[0], x0[1])
            end = self.global_planner.get_node_number(destinations[0])
            run, _, attr = self.global_planner.plan_path(start, end)
            runs.append(run)
            run_attrs.append(attr)
        for i in range(len(destinations) - 1):
            start = self.global_planner.get_node_number(destinations[i])
            end = self.global_planner.get_node_number(destinations[i + 1])
            run, _, attr = self.global_planner.plan_path(start, end)
            runs.append(run)
            run_attrs.append(attr)
        
        # Merge runs while ensuring consistency in point shape.
        merged_run = None
        merged_attrs = None
        for i, (run, attr) in enumerate(zip(runs, run_attrs)):
            run = to_xy_points(run)  # Ensure run shape is (n_points, 2)
            if i > 0:
                # If the first point of the current run is the same as the last point of the merged run, skip it.
                if np.linalg.norm(run[0] - merged_run[-1]) < 1e-6:
                    run = run[1:]
                    attr = attr[1:]
            if i == 0:
                merged_run = run
                merged_attrs = attr
            else:
                merged_run = np.vstack((merged_run, run))
                merged_attrs = np.hstack((merged_attrs, attr))
        
        # Remove any remaining duplicate points.
        merged_run, merged_attrs = filter_duplicate_points(merged_run, merged_attrs)
        
        # Determine base density: waypoints per meter.
        base_density = 1 / abs(v_ref) / T
        
        # Compute total length of the merged path.
        segs = np.diff(merged_run, axis=0)
        seg_lengths = np.linalg.norm(segs, axis=1)
        total_length = np.sum(seg_lengths)
        
        # Calculate the desired number of waypoints.
        num_waypoints = int(np.ceil(total_length * base_density))
        
        # Interpolate waypoints using the selected method.
        if use_spline:
            # (Placeholder: spline interpolation could be inserted here.)
            new_waypoints = linear_interpolate_waypoints(merged_run, num_waypoints)
        else:
            new_waypoints = linear_interpolate_waypoints(merged_run, num_waypoints)
        
        # Interpolate attributes onto the new waypoints.
        new_attributes = interpolate_attributes_linear(merged_run, merged_attrs, new_waypoints)
        
        # Compute yaw angles from the trajectory using gradients.
        dx = np.gradient(new_waypoints[:, 0])
        dy = np.gradient(new_waypoints[:, 1])
        yaw = np.arctan2(dy, dx)
        yaw = smooth_yaw_angles(yaw)
        
        # Build state references: each row is [x, y, yaw]
        state_refs = np.column_stack((new_waypoints, yaw))
        
        # Compute curvature, tangent, and normal vectors.
        curvature, tangent, normals = compute_curvature_and_tangent(new_waypoints)
        
        # Compute reference speeds with adjustments based on attributes.
        v_refs = np.full(len(new_waypoints), v_ref)
        for i, attr in enumerate(new_attributes):
            if attr == 1:
                v_refs[i] /= 1.5
            elif attr in [4, 5]:
                v_refs[i] *= self.hw_density_factor
        
        # Ramp up speed from 0 to v_ref over the first meter.
        num_ramp_points = int(min(1 * base_density, len(v_refs)))
        v_refs[:num_ramp_points] = np.linspace(0, v_ref, num_ramp_points)
        v_refs[-2:] = 0  # Ensure the final waypoints signal a stop.
        
        # Compute a steering reference (here simply proportional to curvature, with a gain of 0).
        steer_ref = 0 * curvature
        
        # Pad the state and input reference arrays.
        pad_length = self.N + 5
        def pad_array(arr, pad_length):
            return np.pad(arr, (0, pad_length), mode='edge')
        
        state_refs_padded = np.vstack((
            pad_array(state_refs[:, 0], pad_length),
            pad_array(state_refs[:, 1], pad_length),
            pad_array(state_refs[:, 2], pad_length)
        )).T
        
        v_refs_padded = pad_array(v_refs, pad_length)
        steer_ref_padded = pad_array(steer_ref, pad_length)
        
        # Save computed properties.
        self.waypoints = new_waypoints           # (n,2): x, y coordinates
        self.state_refs = state_refs_padded      # (n+pad, 3): x, y, yaw
        self.input_refs = np.column_stack((v_refs_padded, steer_ref_padded))
        self.wp_normals = normals
        self.curvature = curvature
        self.v_refs = v_refs
        self.attributes = new_attributes
        self.total_length = total_length
        
        print(f"Total path length: {total_length:.2f} meters with {len(new_waypoints)} waypoints.")
        # self.illustrate_path(state_refs.T)

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
    print("Handle go_to service called")
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
    print("Handle go_to_multiple service called")
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
    print("Handle array service called")
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

    v_ref = config[constraint_name]['v_ref']

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
    plt.savefig('waypoints_plot.png')
    plt.close()


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
    # # x0 = np.array([0.35,2.726,-1.5708])
    # x0 = None
    # name = "run136"
    # path = Path(v_ref = v_ref, N = N, T = T, x0= x0, name = name)
    # np.savetxt(os.path.join(current_dir,'state_refs.txt'), path.state_refs, fmt='%.4f')
    # visualize_waypoints2(path.state_refs)
