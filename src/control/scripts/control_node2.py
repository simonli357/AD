#!/usr/bin/env python3
import time
import numpy as np
import rospy
import threading
import argparse
from utility import Utility
from mpc_acados import Optimizer
from sign_pose import estimate_object_pose2d, draw_scene
import timeit

class StateMachine:
    def __init__(self, args):
        rospy.init_node("mpc_node")

        # constants
        self.CAMERA_PARAMS = {'fx': 554.3826904296875, 'fy': 554.3826904296875, 'cx': 320, 'cy': 240} #gazebo
        self.CAMERA_POSE = {'x': 0, 'y': 0, 'z': 0.2}
        self.CAM_TO_CARFRONT = 0.21
        self.CAR_LENGTH = 0.464
        self.MIN_SIGN_DIST = 0.6 - 0.21
        self.MAX_SIGN_DIST = 1.3 - 0.21
        self.MAX_PARK_DIST = 1.0 - 0.21
        self.PARKSIGN_TO_CAR = 0.51
        self.PARK_OFFSET = 1.1 + 0.21
        self.PARKING_SPOT_LENGTH = 0.723
        self.OVERTAKE_DIST = 2.0 
        self.LANE_OFFSET = 0.31
        self.MIN_DIST_TO_CAR = 0.8
        self.MAX_CAR_DIST = 1.8
        self.SIGN_COOLDOWN = 1.0
        self.STATE_DICT = {"moving": 0, "approaching intersection": 1, "waiting for stop sign": 2, "waiting for traffic light": 3, "parking": 4, "parked": 5, "exiting parking": 6, "done": 7}
        self.INVERSE_STATE_DICT = {v: k for k, v in self.STATE_DICT.items()}
        self.OBJECT_DICT = {"oneway": 0, "highwayentrance": 1, "stopsign": 2, "roundabout": 3, "park": 4, "crosswalk": 5, "noentry": 6, "highwayexit": 7, "priority": 8, "lights": 9, "block": 10, "pedestrian": 11, "car": 12}
        # variables
        self.detected_dist = 0
        self.cooldown_timer = rospy.Time.now()
        self.objects = []
        self.car_states = []

        self.state = 0
        self.debug = True
        self.gaz_bool = "_gazebo_"
        self.args = args
        self.lock = threading.Lock()
        self.utils = Utility(self.lock, subLane=False, subImu=True, subModel=True, pubOdom=True, useEkf=args.useEkf, subSign=args.sign)
        x0 = np.array([self.utils.gps_x, self.utils.gps_y, self.utils.yaw])
        thread = threading.Thread(target=self.spin_thread)
        thread.start()
        print("creating optimizer..., x0: ", x0)
        self.mpc = Optimizer(gazebo=True, x0 = x0)
        self.mpc.lock = self.lock
        self.rate = rospy.Rate(1/self.mpc.T)
        # wait until the first state is received
        print("waiting for first state...")
        while not self.utils.initializationFlag:
            time.sleep(0.1)
        self.mpc.detected_index = 0
        
    def spin_thread(self):
        rospy.spin()
    def add_object(self, object_type, world_pose):
        object_dict = {
            'type': object_type,
            'pose': world_pose
        }
        self.objects.append(object_dict)
        self.car_states.append(self.utils.get_real_states())
    def update_mpc_state(self):
        with self.lock:
            if self.args.odom:
                # self.mpc.update_current_state(x=self.utils.odomX, y=self.utils.odomY, yaw=self.utils.yaw)
                self.mpc.update_current_state(x=self.utils.ekf_x, y=self.utils.ekf_y, yaw=self.utils.yaw)
                self.mpc.update_real_state(x=self.utils.gps_x, y=self.utils.gps_y, yaw=self.utils.yaw)
            else:
                self.mpc.update_current_state(x=self.utils.gps_x, y=self.utils.gps_y, yaw=self.utils.yaw)
        if self.debug:
            self.mpc.x_errors.append(self.utils.gps_x - self.mpc.next_trajectories[0, 0])
            self.mpc.y_errors.append(self.utils.gps_y - self.mpc.next_trajectories[0, 1])
            self.mpc.x_refs.append(self.mpc.next_trajectories[0, :])
            self.mpc.yaw_errors.append(self.mpc.current_state[2] - self.mpc.next_trajectories[0, 2])
    def solve(self):
        t_ = timeit.default_timer()
        u_res = self.mpc.update_and_solve()
        t2 = timeit.default_timer() - t_
        print("solvetime1: ", t2)
        self.utils.steer_command = -float(u_res[1]*180/np.pi)
        self.utils.velocity_command = float(u_res[0])
        self.utils.publish_cmd_vel(steering_angle=self.utils.steer_command, velocity=self.utils.velocity_command)
        if self.debug:
            self.mpc.index_t.append(t2)
            self.mpc.integrate_next_states(u_res)
            self.mpc.t_c.append(self.mpc.t0)
            self.mpc.u_c.append(u_res)
            self.mpc.xx.append(self.mpc.real_state) if self.args.odom else self.mpc.xx.append(self.mpc.current_state)
            self.mpc.mpciter = self.mpc.mpciter + 1
            # print("i) ", self.mpc.mpciter, "cur:", np.around(self.mpc.current_state, decimals=2), "ctrl:", 
            #     np.around(u_res, decimals=2), "idx:", self.mpc.target_waypoint_index, "time:", round(t2, 4))
        rospy.loginfo("Solve time: %f ms", (timeit.default_timer() - t_) * 1000)
    def change_state(self, state):
        print(f"changing state from {self.INVERSE_STATE_DICT[self.state]} to {self.INVERSE_STATE_DICT[state]}")
        self.state = state
    def stop_for(self, duration):
        timer = rospy.Time.now() + rospy.Duration(duration)
        while rospy.Time.now() < timer:
            self.utils.idle()
            self.rate.sleep()
    def run(self):
        # stop when last waypoint is reached
        # length = len(self.mpc.waypoints_x)
        # print("length: ", length)
        timer = rospy.Time.now()
        while True:
            if self.mpc.target_waypoint_index >= self.mpc.num_waypoints-1:
            # if self.mpc.target_waypoint_index >= 600 or self.mpc.target_waypoint_index >= self.mpc.num_waypoints-1:
                self.change_state(self.STATE_DICT["done"])
            if self.state == self.STATE_DICT["moving"]:
                t = time.time()
                if self.args.sign:
                    with self.lock:
                        # stop sign & traffic light
                        stopsign_index = self.utils.object_index(self.OBJECT_DICT["stopsign"])
                        if stopsign_index >= 0: 
                            dist = self.utils.object_distance(stopsign_index)
                            if rospy.Time.now() >= self.cooldown_timer and dist>0:
                                print("stop sign detected at a distance of: ", round(dist, 3))
                                box = self.utils.object_box(stopsign_index)
                                world_pose = estimate_object_pose2d(self.utils.get_real_states(), box, dist, self.CAMERA_PARAMS)
                                print("world pose: ", world_pose)
                                self.add_object("stopsign", world_pose)
                                self.detected_dist = dist
                                self.change_state(self.STATE_DICT["approaching intersection"])
                                continue
                        
                        light_index = self.utils.object_index(self.OBJECT_DICT["lights"])
                        if light_index >= 0:
                            dist = self.utils.object_distance(light_index)
                            if rospy.Time.now() >= self.cooldown_timer and dist>0:
                                print("traffic light detected at a distance of: ", round(dist, 3))
                                box = self.utils.object_box(light_index)
                                world_pose = estimate_object_pose2d(self.utils.get_real_states(), box, dist, self.CAMERA_PARAMS)
                                print("world pose: ", world_pose)
                                self.add_object("lights", world_pose)
                                self.detected_dist = dist
                                self.change_state(self.STATE_DICT["waiting for traffic light"])
                                continue
                        
                        # parking spot
                        park_index = self.utils.object_index(self.OBJECT_DICT["park"])
                        if park_index >= 0:
                            dist = self.utils.object_distance(park_index)
                            if dist < self.MAX_PARK_DIST:
                                print("parking spot detected at a distance of: ", round(dist, 3))
                                box = self.utils.object_box(park_index)
                                world_pose = estimate_object_pose2d(self.utils.get_real_states(), box, dist, self.CAMERA_PARAMS)
                                print("world pose: ", world_pose)
                                self.add_object("park", world_pose)
                                self.stop_for(1)
                                self.detected_dist = dist
                                self.change_state(self.STATE_DICT["parking"])
                                continue

                        # car 
                        car_index = self.utils.object_index(self.OBJECT_DICT["car"])
                        if car_index >= 0:
                            distance = self.utils.object_distance(car_index)
                            if distance < self.MAX_CAR_DIST and self.mpc.target_waypoint_index>= self.mpc.detected_index * 1.2 and park_index<0:
                                distance = max(distance+self.CAM_TO_CARFRONT, self.MIN_DIST_TO_CAR)-self.MIN_DIST_TO_CAR 
                                print("car detected at a distance of: ", round(distance+self.MIN_DIST_TO_CAR, 3), " changing lane")
                                box = self.utils.object_box(self.OBJECT_DICT["car"])
                                world_pose = estimate_object_pose2d(self.utils.get_real_states(), box, distance+self.CAR_LENGTH/2, self.CAMERA_PARAMS)
                                print("world pose: ", world_pose)
                                self.add_object("car", world_pose)
                                offset = int(distance * self.mpc.density)
                                self.mpc.detected_index = self.mpc.target_waypoint_index+int(self.OVERTAKE_DIST * self.mpc.density)
                                self.mpc.path.change_lane(self.mpc.target_waypoint_index+offset, self.mpc.detected_index+offset, self.mpc.wp_normals, shift_distance=self.LANE_OFFSET)
                # update state
                self.update_mpc_state()
                self.solve()
                self.rate.sleep()
                continue
            elif self.state == self.STATE_DICT["approaching intersection"]:
                    offset = max(0, self.detected_dist-self.MIN_SIGN_DIST)
                    print(f"stop sign detected at a distance of: {round(self.detected_dist, 3)}, offset: {round(offset, 3)}")
                    x, y = self.utils.gps_x, self.utils.gps_y
                    while True:
                        norm = np.linalg.norm(np.array([x, y])-np.array([self.utils.gps_x, self.utils.gps_y]))
                        print("norm: ", norm, ", offset: ", offset)
                        if norm >= offset:
                            print("waiting for stop sign...")
                            self.utils.velocity_command = 0
                            self.utils.publish_cmd_vel(steering_angle=self.utils.steer_command, velocity=self.utils.velocity_command)
                            break
                        self.update_mpc_state()
                        self.solve()
                        self.rate.sleep()
                    self.change_state(self.STATE_DICT["waiting for stop sign"])
                    continue
            elif self.state == self.STATE_DICT["waiting for stop sign"]:
                self.stop_for(3)
                self.cooldown_timer = rospy.Time.now() + rospy.Duration(self.SIGN_COOLDOWN)
                self.change_state(self.STATE_DICT["moving"])
            elif self.state == self.STATE_DICT["waiting for traffic light"]:
                self.stop_for(3)
                self.cooldown_timer = rospy.Time.now() + rospy.Duration(3)
                self.change_state(self.STATE_DICT["moving"])
            elif self.state == self.STATE_DICT["parking"]:
                offset = self.detected_dist + self.PARK_OFFSET
                car_index = self.utils.object_index(self.OBJECT_DICT["car"])
                if car_index >= 0:
                    car_dist = self.utils.object_distance(car_index)
                    dist_to_first_spot = self.detected_dist + self.PARKSIGN_TO_CAR - self.CAR_LENGTH/2
                    if car_dist - dist_to_first_spot > (self.PARKING_SPOT_LENGTH-self.CAR_LENGTH)/2*1.1:
                        print("car parked in second spot, proceed to park in first spot")
                    else:
                        print("car parked in first spot, proceed to park in second spot")
                        offset += self.PARKING_SPOT_LENGTH
                print("parking offset is: ", round(offset, 3))
                self.mpc.rate = rospy.Rate(1/self.mpc.T)
                print("changing rate from ", self.rate, " to ", self.mpc.rate)
                orientation = self.utils.get_current_orientation()
                frame = self.utils.get_real_states()
                frame[2] = orientation
                self.mpc.go_straight(utils=self.utils, cur_frame = frame)
                frame = self.utils.get_real_states()
                frame[2] = orientation
                self.mpc.park(utils=self.utils, cur_frame = frame)
                self.change_state(self.STATE_DICT["parked"])
            elif self.state == self.STATE_DICT["parked"]:
                timer = rospy.Time.now() + rospy.Duration(3)
                while rospy.Time.now() < timer:
                    self.utils.idle()
                    self.rate.sleep()
                self.change_state(self.STATE_DICT["exiting parking"])
            elif self.state == self.STATE_DICT["exiting parking"]:
                orientation = self.utils.get_current_orientation()
                frame = self.utils.get_real_states()
                frame[2] = orientation
                self.mpc.exit_park(utils=self.utils, cur_frame = frame)
                self.change_state(self.STATE_DICT["done"])
            elif self.state == self.STATE_DICT["done"]:
                for hsy in range(5):
                    self.utils.idle()
                    self.rate.sleep()
                break
                
        if self.debug:
            stats = self.mpc.compute_stats()
            self.mpc.current_state = [self.utils.gps_x, self.utils.gps_y, self.utils.yaw]
            print("current state: ", self.mpc.current_state)
        # park_offset = 0.1
        # self.mpc.park(park_offset, utils=self.utils)
        # self.mpc.exit_park(utils=self.utils)
        print("done")
        try:
            self.utils.call_trigger_service()
        except:
            print("service not available")
        if self.debug:
            print("objects: ", self.objects)
            draw_scene(self.car_states, self.objects, save=True)
            self.mpc.draw_result(stats, objects = self.objects, car_states = self.car_states) if self.args.plotSign else self.mpc.draw_result(stats)

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--useEkf', action='store_true', help='Boolean for whether to use ekf node or not')
    parser.add_argument('--odom', action='store_true', help='Boolean for whether to use odometry or gps data')
    parser.add_argument('--sign', action='store_true', help='Boolean for whether to use sign or not')
    parser.add_argument('--plotSign', action='store_true', help='Boolean for whether to use sign or not')
    args = parser.parse_args()
    sm = StateMachine(args)
    sm.run()
