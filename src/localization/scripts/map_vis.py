#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped
import tf.transformations as tf_trans
from gazebo_msgs.msg import ModelStates

class MapAndPoseVisualizer:
    def __init__(self):
        rospy.init_node("map_and_pose_visualizer", anonymous=True)
        
        # Subscribers
        self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self.map_callback)
        self.pose_sub = rospy.Subscriber("/slam_toolbox/pose", PoseWithCovarianceStamped, self.pose_callback)
        self.gazebo_sub = rospy.Subscriber("/gazebo/model_states", ModelStates, self.gazebo_callback)
        
        # Initialize map and pose variables
        self.map_image = None
        self.map_info = None
        self.robot_pose = None
        self.robot_orientation = None
        self.car_ground_truth = None
        self.car_idx = None
        self.robot_name = "car1"  # The name of the car model in Gazebo

        self.viewport_size = 400  # Size of the viewport in pixels (e.g., 200x200)
        rospy.loginfo("Map and pose visualizer node started.")
    
    def crop_viewport(self, image, pose):
        # Extract pose in map coordinates
        origin_x = self.map_info.origin.position.x
        origin_y = self.map_info.origin.position.y
        resolution = self.map_info.resolution

        robot_x = int((pose["x"] - origin_x) / resolution)
        robot_y = int((pose["y"] - origin_y) / resolution)

        # Define the viewport bounds
        half_viewport = self.viewport_size // 2
        start_x = max(0, robot_x - half_viewport)
        end_x = min(image.shape[1], robot_x + half_viewport)
        start_y = max(0, robot_y - half_viewport)
        end_y = min(image.shape[0], robot_y + half_viewport)

        # Crop the map
        cropped_image = image[start_y:end_y, start_x:end_x]
        return cropped_image
    def handle_keypress(self, key):
        if key == ord('w'):  # Move viewport up
            self.viewport_center["y"] += 10
        elif key == ord('s'):  # Move viewport down
            self.viewport_center["y"] -= 10
        elif key == ord('a'):  # Move viewport left
            self.viewport_center["x"] -= 10
        elif key == ord('d'):  # Move viewport right
            self.viewport_center["x"] += 10
    def map_callback(self, msg):
        self.map_info = msg.info

        # Convert the OccupancyGrid data to a 2D numpy array
        width = msg.info.width
        height = msg.info.height
        map_data = np.array(msg.data, dtype=np.int8).reshape((height, width))
        
        # Convert occupancy values to a visual format
        self.map_image = np.zeros((map_data.shape[0], map_data.shape[1], 3), dtype=np.uint8)  # RGB image
        self.map_image[map_data == -1] = [128, 128, 128]  # Unknown cells = gray
        self.map_image[map_data == 0] = [255, 255, 255]  # Free cells = white
        self.map_image[map_data == 100] = [0, 0, 0]      # Occupied cells = black
        
        # Flip vertically to align with OpenCV's coordinate system
        self.map_image = cv2.flip(self.map_image, 0)

    def pose_callback(self, msg):
        # Extract robot pose
        self.robot_pose = {
            "x": msg.pose.pose.position.x,
            "y": msg.pose.pose.position.y
        }

        # Extract robot orientation (quaternion to yaw)
        orientation_q = msg.pose.pose.orientation
        _, _, yaw = tf_trans.euler_from_quaternion(
            [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        )
        self.robot_orientation = yaw

    def gazebo_callback(self, msg):
        # Locate the car in the model states
        if self.car_idx is None:
            try:
                self.car_idx = msg.name.index(self.robot_name)
                rospy.loginfo(f"Car found in Gazebo model states at index {self.car_idx}.")
            except ValueError:
                rospy.logwarn("Car not found in Gazebo model states.")
                return

        # Extract the car's ground truth position
        car_pose = msg.pose[self.car_idx]
        self.car_ground_truth = {
            "x": car_pose.position.x,
            "y": car_pose.position.y
        }
        
    def draw_robot(self, image):
        if self.map_info is None or self.robot_pose is None or self.robot_orientation is None:
            return image

        # Convert pose to map coordinates
        origin_x = self.map_info.origin.position.x
        origin_y = self.map_info.origin.position.y
        resolution = self.map_info.resolution

        robot_x = int((self.robot_pose["x"] - origin_x) / resolution)
        robot_y = int((self.robot_pose["y"] - origin_y) / resolution)

        # Flip y-coordinate for OpenCV
        robot_y = image.shape[0] - robot_y

        # Draw the robot as a red circle
        radius = 4  # Radius of the robot marker in pixels
        color = (0, 0, 255)  # Red color in BGR
        cv2.circle(image, (robot_x, robot_y), radius, color, -1)

        # Draw an arrow indicating the orientation
        arrow_length = 10  # Length of the arrow in pixels
        arrow_tip_x = int(robot_x + arrow_length * np.cos(self.robot_orientation))
        arrow_tip_y = int(robot_y - arrow_length * np.sin(self.robot_orientation))  # Y-axis is flipped

        cv2.arrowedLine(image, (robot_x, robot_y), (arrow_tip_x, arrow_tip_y), color, 1, tipLength=0.3)

        return image

    def draw_ground_truth(self, image):
        if self.map_info is None:
            return image
        if self.car_ground_truth is None:
            rospy.logwarn("Car ground truth not available.")
            return image

        # Convert ground truth to map coordinates
        origin_x = self.map_info.origin.position.x
        origin_y = self.map_info.origin.position.y
        resolution = self.map_info.resolution

        car_x = int((self.car_ground_truth["x"] - origin_x) / resolution)
        car_y = int((self.car_ground_truth["y"] - origin_y) / resolution)

        # Flip y-coordinate for OpenCV
        car_y = image.shape[0] - car_y

        # Draw the car's ground truth position as a blue circle
        radius = 5  # Radius of the marker
        color = (255, 0, 0)  # Blue color in BGR
        cv2.circle(image, (car_x, car_y), radius, color, -1)

        return image
    
    def run(self):
        rospy.loginfo("Starting visualization loop...")
        while not rospy.is_shutdown():
            if self.map_image is not None:
                # Create a copy of the map image to draw the robot
                display_image = self.map_image.copy()

                # Draw the robot position and orientation
                display_image = self.draw_robot(display_image)
                display_image = self.draw_ground_truth(display_image)
                
                # Resize for better visualization
                scale = 4
                display_image_resized = cv2.resize(display_image, None, fx=scale, fy=scale, interpolation=cv2.INTER_NEAREST)

                cropped_image = self.crop_viewport(self.map_image, self.robot_pose)
                display_image_resized = cv2.resize(cropped_image, (self.viewport_size * 2, self.viewport_size * 2), interpolation=cv2.INTER_NEAREST)
                
                # Show the map with the robot position
                cv2.imshow("Map with Robot Pose and Orientation", display_image_resized)
                cv2.waitKey(10)
        
        cv2.destroyAllWindows()

if __name__ == "__main__":
    visualizer = MapAndPoseVisualizer()
    visualizer.run()
