#!/usr/bin/env python3
# coding=UTF-8

from mpc_acados2_clean_rate_beta import Optimizer
import os 
import yaml
import time
import networkx as nx
import numpy as np

if "__main__" == __name__:
    current_dir = os.path.dirname(os.path.realpath(__file__))
    starting_points_path = os.path.join(current_dir, 'config/starting_points.yaml')
    with open(starting_points_path, 'r') as f:
        starting_points = yaml.safe_load(f)
        
    G = nx.read_graphml(current_dir + '/maps/Competition_track_graph_modified_new.graphml')
    pos = {}
    attribute = {}
    for node, data in G.nodes(data=True):
        x = data.get('x', 0.0)  # Default value 0.0 if 'x' is missing
        if 502 <= int(node) <= 521:
            y = data.get('y', 0.0)
        elif 483 <= int(node) <= 502:
            y = data.get('y', 0.0)
        else:
            y = data.get('y', 0.0)  # Default value 0.0 if 'y' is missing
        pos[node] = (x, 13.786 - y)
        attribute[node] = data.get('new_attribute', 0)
        
    for start in starting_points:
        start_str = 'run' + str(start)
        x0 = np.array(pos[str(start)])
        print(f"Running {start_str}")
        mpc = Optimizer(name=start_str, x0=x0)
    
        mpc.target_waypoint_index = 0
        maxiter = 6000
        print("current state: ", mpc.current_state)
        while True:
            if mpc.target_waypoint_index >= mpc.num_waypoints-1 or mpc.mpciter > maxiter:
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