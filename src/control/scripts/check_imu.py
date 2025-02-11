#!/usr/bin/env python3
"""
imu_visualizer.py

This script subscribes to the /imu/data topic (sensor_msgs/Imu messages) and
visualizes the linear acceleration, angular velocity, and orientation (roll, pitch, yaw)
in real time using Matplotlib.

Usage:
    chmod +x imu_visualizer.py
    rosrun <your_package> imu_visualizer.py

Make sure the topic names match your setup.
"""

import rospy
from sensor_msgs.msg import Imu
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from tf.transformations import euler_from_quaternion
from collections import deque

# Define maximum number of samples to display (e.g., last ~10 seconds worth of data)
max_samples = 300

# Create deques to store time and sensor data with a fixed maximum length
time_data   = deque(maxlen=max_samples)
accel_x     = deque(maxlen=max_samples)
accel_y     = deque(maxlen=max_samples)
accel_z     = deque(maxlen=max_samples)
angular_x   = deque(maxlen=max_samples)
angular_y   = deque(maxlen=max_samples)
angular_z   = deque(maxlen=max_samples)
roll_data   = deque(maxlen=max_samples)
pitch_data  = deque(maxlen=max_samples)
yaw_data    = deque(maxlen=max_samples)

def imu_callback(msg):
    """
    Callback function for IMU data. It appends the new data into the deques.
    """
    # Get timestamp (in seconds)
    t = msg.header.stamp.to_sec()
    time_data.append(t)

    # Append linear acceleration data (in m/s^2)
    accel_x.append(msg.linear_acceleration.x)
    accel_y.append(msg.linear_acceleration.y)
    accel_z.append(msg.linear_acceleration.z)

    # Append angular velocity data (in rad/s)
    angular_x.append(msg.angular_velocity.x)
    angular_y.append(msg.angular_velocity.y)
    angular_z.append(msg.angular_velocity.z)

    # Convert quaternion orientation to Euler angles (roll, pitch, yaw in radians)
    q = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
    roll, pitch, yaw = euler_from_quaternion(q)
    roll_data.append(roll)
    pitch_data.append(pitch)
    yaw_data.append(yaw)
    print("yaw (deg): ", yaw*180/3.14159)

def animate(frame, axs):
    """
    This function is called periodically from FuncAnimation to update the plots.
    """
    # Clear and update the acceleration subplot
    axs[0].clear()
    axs[0].plot(time_data, accel_x, label='X')
    axs[0].plot(time_data, accel_y, label='Y')
    axs[0].plot(time_data, accel_z, label='Z')
    axs[0].set_title('Linear Acceleration (m/s^2)')
    axs[0].set_xlabel('Time (s)')
    axs[0].legend()
    axs[0].grid(True)

    # Clear and update the angular velocity subplot
    axs[1].clear()
    axs[1].plot(time_data, angular_x, label='X')
    axs[1].plot(time_data, angular_y, label='Y')
    axs[1].plot(time_data, angular_z, label='Z')
    axs[1].set_title('Angular Velocity (rad/s)')
    axs[1].set_xlabel('Time (s)')
    axs[1].legend()
    axs[1].grid(True)

    # Clear and update the orientation subplot (roll, pitch, yaw)
    axs[2].clear()
    axs[2].plot(time_data, roll_data, label='Roll')
    axs[2].plot(time_data, pitch_data, label='Pitch')
    axs[2].plot(time_data, yaw_data, label='Yaw')
    axs[2].set_title('Orientation (rad)')
    axs[2].set_xlabel('Time (s)')
    axs[2].legend()
    axs[2].grid(True)

def main():
    # Initialize the ROS node
    rospy.init_node('imu_visualizer', anonymous=True)
    
    # Subscribe to the IMU data topic (change topic if necessary)
    # imu_topic = '/camera/imu'
    # imu_topic = '/realsense_ros_node/imu/data'
    imu_topic = '/imu/data'
    # imu_topic = '/fcu/imu'
    # imu_topic = '/carla/ego_vehicle/imu'
    rospy.Subscriber(imu_topic, Imu, imu_callback)
    
    # Set up the Matplotlib figure and three subplots
    fig, axs = plt.subplots(3, 1, figsize=(10, 8))
    fig.tight_layout(pad=3.0)

    # Use FuncAnimation to update the plots in real time
    ani = animation.FuncAnimation(fig, animate, fargs=(axs,), interval=100)
    
    # Show the plot window. This call is blocking.
    plt.show()

    # If the plot window is closed, shutdown the node
    rospy.signal_shutdown("Plot window closed.")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
