#! /usr/bin/env python3

import rospy
import numpy as np
from utils.msg import localisation, IMU, encoder
import math
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry 
import time
import matplotlib.pyplot as plt
from sensor_msgs.msg import Imu
import tf
import argparse
from std_msgs.msg import String
import os
from std_srvs.srv import Trigger, TriggerResponse
from geometry_msgs.msg import PoseWithCovarianceStamped

class Odom():
    def __init__(self):
        rospy.init_node('localization_plot', anonymous=True)
        self.name = 'car1'
        self.odomState = np.zeros(2)
        self.gpsState = np.zeros(2)
        self.ekfState = np.zeros(2)
        self.gpsValuesList = []
        self.ekfValuesList = []
        self.odomValuesList = []

        self.yaw1 = 0.0
        self.yaw2 = 0.0
        self.yaw1List = []
        self.yaw2List = []
        self.accelList_x = []
        self.accelList_y = []

        self.groundTwist = np.zeros(2)
        self.measuredTwist = np.zeros(2)
        self.groundSpeedXList = []
        self.groundSpeedYList = []
        self.measuredSpeedXList = []
        self.measuredSpeedYList = []

        self.car_idx = None
        self.car_pose = None
        self.car_inertial = None

        rospy.on_shutdown(self.plot_data)
        rospy.wait_for_message("/"+self.name+"/command", String)

        # Subscribe to topics
        # self.localization_sub = rospy.Subscriber("/automobile/localisation", localisation, self.gps_callback, queue_size=3)
        # self.localization_sub = rospy.Subscriber("/automobile/localisation", localisation, self.gps_callback, queue_size=3)
        self.model_sub = rospy.Subscriber("/gazebo/model_states", ModelStates, self.gps_callback, queue_size=3)
        self.ekf_sub = rospy.Subscriber("/odometry/filtered", Odometry, self.ekf_callback, queue_size=3)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback, queue_size=3)
        # self.odom_sub = rospy.Subscriber("/gps", PoseWithCovarianceStamped, self.odom_callback, queue_size=3)
        self.imu1_sub = rospy.Subscriber("/"+self.name+"/imu", Imu, self.imu1_callback, queue_size=3)
        self.timer = rospy.Timer(rospy.Duration(1.0 /50.0), self.compare)
        stopTrigger = rospy.Service('trigger_service', Trigger, self.handle_trigger)
    
    def handle_trigger(self, req):
        print("Service has been triggered. Plotting data...")
        response = TriggerResponse(success=True, message="Triggered successfully!")
        self.plot_data()
        print("Plot saved")
        time.sleep(0.5)
        rospy.signal_shutdown("Service has been triggered. Shutting down.")
        return response
    def gps_callback(self, data):
        # self.gpsState[0] = data.posA
        # self.gpsState[1] = 15.0 - data.posB
        if self.car_idx is None:
            try:
                self.car_idx = data.name.index(self.name)
            except ValueError:
                return
        self.car_pose = data.pose[self.car_idx]
        self.car_inertial = data.twist[self.car_idx]
        self.gpsState[0] = self.car_pose.position.x
        # self.gpsState[1] = 15+self.car_pose.position.y
        self.gpsState[1] = self.car_pose.position.y
        self.groundTwist[0] = self.car_inertial.linear.x
        self.groundTwist[1] = self.car_inertial.linear.y
    def ekf_callback(self, data):
        self.ekfState[0] = data.pose.pose.position.x
        self.ekfState[1] = data.pose.pose.position.y
        self.yaw2 = tf.transformations.euler_from_quaternion([data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w])[2]
        self.measuredTwist[0] = data.twist.twist.linear.x
        self.measuredTwist[1] = data.twist.twist.linear.y
    def odom_callback(self, data):
        self.odomState[0] = data.pose.pose.position.x
        self.odomState[1] = data.pose.pose.position.y
    def imu1_callback(self, imu):
        self.yaw1 = tf.transformations.euler_from_quaternion([imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w])[2]
        self.accelList_x.append(imu.linear_acceleration.x)
        self.accelList_y.append(imu.linear_acceleration.y)

    def compare(self, event):
        self.yaw1List.append(self.yaw1)
        self.yaw2List.append(self.yaw2)

        self.groundSpeedXList.append(self.groundTwist[0])
        self.groundSpeedYList.append(self.groundTwist[1])
        self.measuredSpeedXList.append(self.measuredTwist[0])
        self.measuredSpeedYList.append(self.measuredTwist[1])

        self.ekfValuesList.append(self.ekfState.copy())
        self.gpsValuesList.append(self.gpsState.copy())
        self.odomValuesList.append(self.odomState.copy())
        ekf_error_x = self.ekfState[0] - self.gpsState[0]
        ekf_error_y = self.ekfState[1] - self.gpsState[1]
        
        odom_error_x = self.odomState[0] - self.gpsState[0]
        odom_error_y = self.odomState[1] - self.gpsState[1]

        print("GPS State: [{:.3f}, {:.3f}]".format(self.gpsState[0], self.gpsState[1]))
        print("EKF State: [{:.3f}, {:.3f}] | X Error: {:.3f} | Y Error: {:.3f}".format(self.ekfState[0], self.ekfState[1], ekf_error_x, ekf_error_y))
        print("Odom State: [{:.3f}, {:.3f}] | X Error: {:.3f} | Y Error: {:.3f}".format(self.odomState[0], self.odomState[1], odom_error_x, odom_error_y))
        print("----------")
    
    def plot_data(self):
        # rospy.signal_shutdown("Finished")
        labels = ['X', 'Y', 'Yaw', 'x vel', 'y vel', 'error vs yaw', 'Accel', 'XY']
        fig, axs = plt.subplots(2, len(labels), figsize=(35,10))
        labels = ['X', 'Y', 'XY']
        fig, axs = plt.subplots(2, len(labels), figsize=(15,10))

        groundYaw = (np.array(self.yaw1List)+np.pi)*180/np.pi
        measuredYaw = (np.array(self.yaw2List)+np.pi)*180/np.pi
        accel_x = np.array(self.accelList_x)
        accel_y = np.array(self.accelList_y)

        groundSpeedX = np.array(self.groundSpeedXList)
        groundSpeedY = np.array(self.groundSpeedYList)
        measuredSpeedX = np.array(self.measuredSpeedXList)
        measuredSpeedY = np.array(self.measuredSpeedYList)

        groundValues = np.array(self.gpsValuesList)
        measuredEKFValues = np.array(self.ekfValuesList)
        measuredOdomValues = np.array(self.odomValuesList)

        min_len = min(len(groundValues), len(measuredEKFValues), len(measuredOdomValues), len(groundYaw),
                       len(measuredYaw), len(groundSpeedX), len(groundSpeedY), len(measuredSpeedX), 
                       len(measuredSpeedY))
        groundValues = groundValues[:min_len]
        measuredEKFValues = measuredEKFValues[:min_len]
        measuredOdomValues = measuredOdomValues[:min_len]
        groundYaw = groundYaw[:min_len]
        measuredYaw = measuredYaw[:min_len]
        groundSpeedX = groundSpeedX[:min_len]
        groundSpeedY = groundSpeedY[:min_len]
        measuredSpeedX = measuredSpeedX[:min_len]
        measuredSpeedY = measuredSpeedY[:min_len]
        accel_x = accel_x[:min_len]
        accel_y = accel_y[:min_len]

        ekf_errors = measuredEKFValues - groundValues
        odom_errors = measuredOdomValues - groundValues
        yaw_errors = measuredYaw - groundYaw
        yaw_errors[yaw_errors > 6] -= 2 * np.pi
        yaw_errors[yaw_errors < -6] += 2 * np.pi
        x_vel_errors = measuredSpeedX - groundSpeedX
        y_vel_errors = measuredSpeedY - groundSpeedY

        print(groundValues.shape, measuredEKFValues.shape, measuredOdomValues.shape, groundYaw.shape, measuredYaw.shape)
        print("accels", accel_x.shape, accel_y.shape)

        # Plot the ground, EKF, and Odom values
        for i, label in enumerate(labels):
            # Ignore first 10% of the data to allow for the system to settle
            start_index = math.ceil(len(groundValues) * 0.005)
            end_index = int(len(groundValues) * 0.9)

            if label == 'Yaw':
                axs[0, i].plot(groundYaw[start_index:end_index], label='Ground Truth Yaw')
                axs[0, i].plot(measuredYaw[start_index:end_index], label='Measured Yaw')
                axs[0, i].set_title(f"{label} Over Time")
                axs[0, i].legend(loc='lower right')

                # Plot the error for Yaw
                axs[1, i].plot(yaw_errors[start_index:end_index], label='Yaw Error')
                axs[1, i].set_title(f"Error in {label}")
                axs[1, i].legend(loc='lower right')

                min_error_yaw = np.min(yaw_errors[start_index:end_index])
                max_error_yaw = np.max(yaw_errors[start_index:end_index])
                mean_error_yaw = np.mean(np.abs(yaw_errors[start_index:end_index]))
                sd_error_yaw = np.std(yaw_errors[start_index:end_index])
                stats_text = f"Yaw - Min: {min_error_yaw:.3f} Max: {max_error_yaw:.3f} Mean: {mean_error_yaw:.3f} SD: {sd_error_yaw:.3f}"
                axs[1, i].text(0.05, 0.95, stats_text, transform=axs[1, i].transAxes, verticalalignment='top')
            elif label == 'x vel':
                axs[0, i].plot(groundSpeedX[start_index:end_index], label='Ground Truth X Velocity')
                axs[0, i].plot(measuredSpeedX[start_index:end_index], label='Measured X Velocity')
                axs[0, i].set_title(f"{label} Over Time")
                axs[0, i].legend(loc='lower right')

                # Plot the error for X Velocity
                axs[1, i].plot(x_vel_errors[start_index:end_index], label='X Velocity Error')
                axs[1, i].set_title(f"Error in {label}")
                axs[1, i].legend(loc='lower right')

                min_error_x_vel = np.min(x_vel_errors[start_index:end_index])
                max_error_x_vel = np.max(x_vel_errors[start_index:end_index])
                mean_error_x_vel = np.mean(np.abs(x_vel_errors[start_index:end_index]))
                sd_error_x_vel = np.std(x_vel_errors[start_index:end_index])
                stats_text = f"X Vel - Min: {min_error_x_vel:.3f} Max: {max_error_x_vel:.3f} Mean: {mean_error_x_vel:.3f} SD: {sd_error_x_vel:.3f}"
                axs[1, i].text(0.05, 0.95, stats_text, transform=axs[1, i].transAxes, verticalalignment='top')
            elif label == 'y vel':
                axs[0, i].plot(groundSpeedY[start_index:end_index], label='Ground Truth Y Velocity')
                axs[0, i].plot(measuredSpeedY[start_index:end_index], label='Measured Y Velocity')
                axs[0, i].set_title(f"{label} Over Time")
                axs[0, i].legend(loc='lower right')

                # Plot the error for Y Velocity
                axs[1, i].plot(y_vel_errors[start_index:end_index], label='Y Velocity Error')
                axs[1, i].set_title(f"Error in {label}")
                axs[1, i].legend(loc='lower right')
                min_error_y_vel = np.min(y_vel_errors[start_index:end_index])
                max_error_y_vel = np.max(y_vel_errors[start_index:end_index])
                mean_error_y_vel = np.mean(np.abs(y_vel_errors[start_index:end_index]))
                sd_error_y_vel = np.std(y_vel_errors[start_index:end_index])

                stats_text = f"Y Vel - Min: {min_error_y_vel:.3f} Max: {max_error_y_vel:.3f} Mean: {mean_error_y_vel:.3f} SD: {sd_error_y_vel:.3f}"
                axs[1, i].text(0.05, 0.95, stats_text, transform=axs[1, i].transAxes, verticalalignment='top')
            elif label == 'Accel':
                axs[0, i].plot(accel_x[start_index:end_index], label='accel x')
                axs[0, i].set_title(f"{label} Over Time")
                axs[0, i].legend(loc='lower right')

                # Plot the error for EKF and Odom
                axs[1, i].plot(accel_y[start_index:end_index], label='accel y')
                axs[1, i].set_title(f"accel y")

                # Calculate statistics and add them to the error plot for EKF
                min_x = np.min(accel_x[start_index:end_index])
                max_x = np.max(accel_x[start_index:end_index])
                mean_x = np.mean(np.abs(accel_x[start_index:end_index]))
                sd_x = np.std(accel_x[start_index:end_index])

                # Calculate statistics and add them to the error plot for Odom
                min_y = np.min(accel_y[start_index:end_index])
                max_y = np.max(accel_y[start_index:end_index])
                mean_y = np.mean(np.abs(accel_y[start_index:end_index]))
                sd_y = np.std(accel_y[start_index:end_index])

                stats_text = f"X - Min: {min_x:.3f} Max: {max_x:.3f} Mean: {mean_x:.3f} SD: {sd_x:.3f}\n"
                stats_text += f"Y - Min: {min_y:.3f} Max: {max_y:.3f} Mean: {mean_y:.3f} SD: {sd_y:.3f}"
                
                axs[1, i].text(0.05, 0.95, stats_text, transform=axs[1, i].transAxes, verticalalignment='top')
                axs[1, i].legend(loc='lower right')
            elif label == 'error vs yaw':
                axs[0, i].scatter(groundYaw[start_index:end_index], ekf_errors[start_index:end_index, 0], label='EKF X Error vs Yaw', s=5)
                axs[0, i].set_title("EKF X Error vs Yaw")
                axs[0, i].legend(loc='lower right')
                
                axs[1, i].scatter(groundYaw[start_index:end_index], ekf_errors[start_index:end_index, 1], label='EKF Y Error vs Yaw', s=5)
                axs[1, i].set_title("EKF Y Error vs Yaw")
                axs[1, i].legend(loc='lower right')

                # Compute statistics for EKF errors with respect to Yaw
                mean_error_x = np.mean(ekf_errors[start_index:end_index, 0])
                std_dev_x = np.std(ekf_errors[start_index:end_index, 0])
                min_error_x = np.min(ekf_errors[start_index:end_index, 0])
                max_error_x = np.max(ekf_errors[start_index:end_index, 0])
                correlation_x = np.corrcoef(groundYaw[start_index:end_index], ekf_errors[start_index:end_index, 0])[0, 1]

                mean_error_y = np.mean(ekf_errors[start_index:end_index, 1])
                std_dev_y = np.std(ekf_errors[start_index:end_index, 1])
                min_error_y = np.min(ekf_errors[start_index:end_index, 1])
                max_error_y = np.max(ekf_errors[start_index:end_index, 1])
                correlation_y = np.corrcoef(groundYaw[start_index:end_index], ekf_errors[start_index:end_index, 1])[0, 1]

                stats_text_x = f"X Error - Mean: {mean_error_x:.3f}, SD: {std_dev_x:.3f}, Min: {min_error_x:.3f}, Max: {max_error_x:.3f}, Correlation: {correlation_x:.3f}"
                axs[0, i].text(0.05, 0.95, stats_text_x, transform=axs[0, i].transAxes, verticalalignment='top')

                stats_text_y = f"Y Error - Mean: {mean_error_y:.3f}, SD: {std_dev_y:.3f}, Min: {min_error_y:.3f}, Max: {max_error_y:.3f}, Correlation: {correlation_y:.3f}"
                axs[1, i].text(0.05, 0.95, stats_text_y, transform=axs[1, i].transAxes, verticalalignment='top')
            elif label == 'XY':
                # Plot paths for Ground Truth, EKF, and Odom
                axs[0, i].plot(groundValues[start_index:end_index, 0], groundValues[start_index:end_index, 1], label='Ground Truth')
                axs[0, i].plot(measuredEKFValues[start_index:end_index, 0], measuredEKFValues[start_index:end_index, 1], label='EKF')
                axs[0, i].plot(measuredOdomValues[start_index:end_index, 0], measuredOdomValues[start_index:end_index, 1], label='Odom')
                axs[0, i].set_title("Path in XY Plane")
                axs[0, i].set_xlabel("X")
                axs[0, i].set_ylabel("Y")
                axs[0, i].legend(loc='upper right')
                axs[0, i].axis('equal')  # Ensuring equal scaling on both axes
                
                axs[1, i].plot(groundYaw[start_index:end_index], label='Ground Truth Yaw')
                axs[1, i].plot(measuredYaw[start_index:end_index], label='Measured Yaw')
                axs[1, i].set_title("Yaw")
                axs[1, i].legend(loc='lower right')

                # min_error_yaw = np.min(yaw_errors[start_index:end_index])
                # max_error_yaw = np.max(yaw_errors[start_index:end_index])
                # mean_error_yaw = np.mean(np.abs(yaw_errors[start_index:end_index]))
                # sd_error_yaw = np.std(yaw_errors[start_index:end_index])
                # stats_text = f"Yaw - Min: {min_error_yaw:.3f} Max: {max_error_yaw:.3f} Mean: {mean_error_yaw:.3f} SD: {sd_error_yaw:.3f}"
                # axs[1, i].text(0.05, 0.95, stats_text, transform=axs[1, i].transAxes, verticalalignment='top')
            else:
                axs[0, i].plot(groundValues[start_index:end_index, i], label='Ground Truth')
                axs[0, i].plot(measuredEKFValues[start_index:end_index, i], label='EKF')
                axs[0, i].plot(measuredOdomValues[start_index:end_index, i], label='Odom')
                axs[0, i].set_title(f"{label} Over Time")
                axs[0, i].legend(loc='lower right')

                # Plot the error for EKF and Odom
                axs[1, i].plot(ekf_errors[start_index:end_index, i], label='EKF Error')
                axs[1, i].plot(odom_errors[start_index:end_index, i], label='Odom Error')
                axs[1, i].set_title(f"Error in {label} w.r.t GPS")

                # Calculate statistics and add them to the error plot for EKF
                min_error_ekf = np.min(ekf_errors[start_index:end_index, i])
                max_error_ekf = np.max(ekf_errors[start_index:end_index, i])
                mean_error_ekf = np.mean(np.abs(ekf_errors[start_index:end_index, i]))
                sd_error_ekf = np.std(ekf_errors[start_index:end_index, i])

                # Calculate statistics and add them to the error plot for Odom
                min_error_odom = np.min(odom_errors[start_index:end_index, i])
                max_error_odom = np.max(odom_errors[start_index:end_index, i])
                mean_error_odom = np.mean(np.abs(odom_errors[start_index:end_index, i]))
                sd_error_odom = np.std(odom_errors[start_index:end_index, i])

                stats_text = f"EKF - Min: {min_error_ekf:.3f} Max: {max_error_ekf:.3f} Mean: {mean_error_ekf:.3f} SD: {sd_error_ekf:.3f}\n"
                stats_text += f"Odom - Min: {min_error_odom:.3f} Max: {max_error_odom:.3f} Mean: {mean_error_odom:.3f} SD: {sd_error_odom:.3f}"
                
                axs[1, i].text(0.05, 0.95, stats_text, transform=axs[1, i].transAxes, verticalalignment='top')
                axs[1, i].legend(loc='lower right')

        plt.tight_layout()
        path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "testPlots")
        os.makedirs(path, exist_ok=True)
        name = os.path.join(path, "1109.png")
        plt.savefig(name)
        print("Plot saved")
        # plt.show()

if __name__ == '__main__':
    node = Odom()
    while not rospy.is_shutdown():
        rospy.spin()
        