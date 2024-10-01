#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu
import math

class ImuChangeDetector:
    def __init__(self):
        self.prev_orientation = None
        self.prev_ang_vel = None
        self.prev_lin_accel = None
        
        rospy.Subscriber('car1/imu', Imu, self.imu_callback)

    def compare_changes(self, current, previous):
        changes = [abs(current[i] - previous[i]) for i in range(len(current))]
        max_change_idx = changes.index(max(changes))
        return max_change_idx, max(changes)

    def imu_callback(self, msg):
        current_orientation = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        current_ang_vel = [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]
        current_lin_accel = [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]
        
        if self.prev_orientation is not None:
            max_idx, max_change = self.compare_changes(current_orientation, self.prev_orientation)
            orientation_labels = ['x', 'y', 'z', 'w']
            rospy.loginfo(f'Orientation changed the most in {orientation_labels[max_idx]} with a change of {max_change:.6f}')

        if self.prev_ang_vel is not None:
            max_idx, max_change = self.compare_changes(current_ang_vel, self.prev_ang_vel)
            ang_vel_labels = ['x', 'y', 'z']
            rospy.loginfo(f'Angular Velocity changed the most in {ang_vel_labels[max_idx]} with a change of {max_change:.6f}')

        if self.prev_lin_accel is not None:
            max_idx, max_change = self.compare_changes(current_lin_accel, self.prev_lin_accel)
            lin_accel_labels = ['x', 'y', 'z']
            rospy.loginfo(f'Linear Acceleration changed the most in {lin_accel_labels[max_idx]} with a change of {max_change:.6f}')
        
        self.prev_orientation = current_orientation
        self.prev_ang_vel = current_ang_vel
        self.prev_lin_accel = current_lin_accel

if __name__ == '__main__':
    rospy.init_node('imu_change_detector')
    detector = ImuChangeDetector()
    rospy.spin()
