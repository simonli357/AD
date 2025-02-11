#!/usr/bin/env python3
"""
This script loads control commands from an npy file, publishes them to Gazebo at 10Hz,
subscribes to the /gazebo/model_states topic to record the robot's (car1) path,
and then plots both the actual robot path and the reference path loaded from x_refs.npy.
"""

import rospy
import numpy as np
import matplotlib.pyplot as plt
from std_msgs.msg import String
from gazebo_msgs.msg import ModelStates
import os

current_dir = os.path.dirname(os.path.abspath(__file__))
class CommandPublisher:
    def __init__(self):
        # Initialize the ROS node.
        rospy.init_node('command_publisher', anonymous=True)

        # Publishers (we use std_msgs/String for publishing JSON strings):
        self.commands_pub = rospy.Publisher('/car1/command', String, queue_size=10)

        # Subscriber for Gazebo model states.
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_callback)

        # Get current directory (or set via a ROS parameter)
        self.current_dir = rospy.get_param('~current_dir', '.')

        # Load the control commands from file (each row: [velocity, steering_angle])
        try:
            self.u_controls = np.load(os.path.join(current_dir, "u_c_simple.npy"))
        except IOError:
            rospy.logerr("Could not load u_c.npy. Make sure the file exists in " + self.current_dir)
            rospy.signal_shutdown("Missing u_c.npy")
            return

        # Attempt to load the reference trajectory.
        try:
            self.x_refs = np.load(os.path.join(current_dir, "x_refs.npy"))
        except IOError:
            rospy.logwarn("x_refs.npy not found. Reference trajectory will not be displayed.")
            self.x_refs = None

        self.command_index = 0  # index for iterating through the commands
        self.rate = rospy.Rate(10)  # publish at 10 Hz

        # Set some parameters (change as needed).
        self.maxspeed = 1.0  # maximum speed (m/s)
        self.yaw = 0.0       # default yaw value

        # For storing the robot's path as (x, y) coordinates.
        self.path = []
        self.i = None  # index for "car1" in the model states

    def publish_cmd_vel(self, steering_angle, velocity=None, clip=True):
        """
        Publish command messages to drive the robot.
        
        steering_angle: in degrees.
        velocity: in m/s (if not provided, defaults to self.maxspeed).
        If clip is True, the steering_angle is limited to between -23 and 23 degrees.
        """
        if velocity is None:
            velocity = self.maxspeed
        if clip:
            steering_angle = np.clip(steering_angle, -23, 23)
        # Create the two JSON strings as in your reference.
        msg1 = String()
        msg2 = String()
        msg1.data = '{"action":"1","speed":' + str(velocity) + '}'
        msg2.data = '{"action":"2","steerAngle":' + str(float(steering_angle)) + '}'
        # Publish the messages.
        self.commands_pub.publish(msg1)
        self.commands_pub.publish(msg2)
        rospy.logdebug("Published: speed=%s, steering=%s", velocity, steering_angle)

    def model_callback(self, model):
        """
        Callback to process model state messages from Gazebo.
        This function finds the model named "car1" and records its (x, y) position.
        """
        if self.i is None:
            try:
                self.i = model.name.index("car1")
            except ValueError:
                rospy.logwarn("car1 not found in model states")
                return
        # Get the robot's pose.
        self.gps_x = model.pose[self.i].position.x
        self.gps_y = model.pose[self.i].position.y
        # Optionally, extract yaw from the quaternion if needed.
        # Append the current position to the path.
        self.path.append((self.gps_x, self.gps_y))

    def run(self):
        rospy.loginfo("Starting command publishing...")
        while not rospy.is_shutdown() and self.command_index < len(self.u_controls):
            # Get the current control command.
            cmd = self.u_controls[self.command_index]
            velocity = cmd[0]
            steering = cmd[1]
            # Publish the command.
            self.publish_cmd_vel(-steering * 180/np.pi, velocity)
            self.command_index += 1
            self.rate.sleep()

        rospy.loginfo("All commands published. Waiting for 5 seconds before shutdown...")
        rospy.sleep(5)  # Allow time for the robot to finish its motion.
        rospy.signal_shutdown("Finished publishing commands")

        # After shutdown, plot the recorded path along with the reference trajectory.
        self.plot_path()

    def plot_path(self):
        """
        Plot the path followed by the robot (recorded from model states) along with
        the reference trajectory loaded from x_refs.npy (if available).
        """
        if len(self.path) == 0:
            rospy.logwarn("No path data received. Nothing to plot.")
            return

        path_array = np.array(self.path)
        plt.figure(figsize=(10, 8))
        plt.plot(path_array[:, 0], path_array[:, 1], 'b-', label="Robot Path")
        
        # Plot the reference trajectory if available.
        if self.x_refs is not None and self.x_refs.shape[1] >= 2:
            plt.plot(self.x_refs[:, 0], self.x_refs[:, 1], 'g--', label="Reference Path")

        plt.xlabel("X [m]")
        plt.ylabel("Y [m]")
        plt.title("Path Followed by the Robot vs. Reference Trajectory")
        plt.legend()
        plt.grid(True)
        plt.axis('equal')
        plt.show()

if __name__ == '__main__':
    try:
        node = CommandPublisher()
        node.run()
    except rospy.ROSInterruptException:
        pass
