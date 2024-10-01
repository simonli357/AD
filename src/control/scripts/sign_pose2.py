import math
import numpy as np

def compute_3D_position(bbox, distance, cx, cy, fx, fy):
    """
    Computes the 3D position of an object given its bounding box, distance, and camera parameters.

    Parameters:
    bbox (tuple): Bounding box of the object in the format (x, y, width, height).
    distance (float): Distance from the camera to the object in meters.
    cx (float): X-coordinate of the camera's optical center.
    cy (float): Y-coordinate of the camera's optical center.
    fx (float): Focal length of the camera along the X-axis.
    fy (float): Focal length of the camera along the Y-axis.

    Returns:
    tuple: 3D position (X, Y, Z) of the object in camera coordinates.
    """
    x, y, w, h = bbox
    # Compute the center of the bounding box
    center_x = x + w / 2.0
    center_y = y + h / 2.0

    # Convert from pixel coordinates to camera coordinates
    X = (center_x - cx) * distance / fx
    Y = (center_y - cy) * distance / fy
    Z = distance

    return (X, Y, Z)

def euler_to_rotation_matrix(roll, pitch, yaw):
    """
    Converts Euler angles to a rotation matrix.

    Parameters:
    roll, pitch, yaw (float): Euler angles in radians.

    Returns:
    numpy.ndarray: 3x3 rotation matrix.
    """
    R_x = np.array([[1, 0, 0],
                    [0, np.cos(roll), -np.sin(roll)],
                    [0, np.sin(roll), np.cos(roll)]])
    
    R_y = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                    [0, 1, 0],
                    [-np.sin(pitch), 0, np.cos(pitch)]])
    
    R_z = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                    [np.sin(yaw), np.cos(yaw), 0],
                    [0, 0, 1]])
    
    R = np.dot(R_z, np.dot(R_y, R_x))
    return R

def compute_world_position(camera_position, camera_orientation, object_position_camera_frame):
    """
    Computes the position of an object in the world frame given its position in the camera frame and the camera's pose.

    Parameters:
    camera_position (tuple): Position of the camera in the world frame (x, y, z).
    camera_orientation (tuple): Orientation of the camera in the world frame (roll, pitch, yaw) in radians.
    object_position_camera_frame (tuple): Position of the object in the camera frame (X, Y, Z).

    Returns:
    tuple: Position of the object in the world frame.
    """
    # Convert camera orientation (Euler angles) to a rotation matrix
    roll, pitch, yaw = camera_orientation
    R = euler_to_rotation_matrix(roll, pitch, yaw)

    # Convert object position in camera frame to a NumPy array
    object_position_camera_frame = np.array(object_position_camera_frame)

    # Rotate the object position from camera frame to world frame
    object_position_world_frame = np.dot(R, object_position_camera_frame)

    # Translate the object position based on camera position in world frame
    object_position_world_frame += np.array(camera_position)

    return tuple(object_position_world_frame)

# Example usage
camera_position = (0.82, 0.093, 0.2)  # x, y, z in world frame
camera_orientation = (1.57, 0.0, 1.57)  # roll, pitch, yaw in radians
# object_position_camera_frame = (0.5, 0.5, 2)  # X, Y, Z in camera frame
camera_params = {'fx': 554.3826904296875, 'fy': 554.3826904296875, 'cx': 320, 'cy': 240} 
bounding_box = np.array([542, 226, 639, 466]) # xmin, ymin, xmax, ymax
#bbox has x_center, y_center, width, height
bbox = np.array([bounding_box[0] + bounding_box[2] / 2.0, bounding_box[1] + bounding_box[3] / 2.0, bounding_box[2], bounding_box[3]]) # x, y, width, height
object_distance = 0.521

object_position_camera_frame = compute_3D_position(bbox, object_distance, **camera_params)
print("Object position in camera frame:", object_position_camera_frame)
world_position = compute_world_position(camera_position, camera_orientation, object_position_camera_frame)
print("Object position in world frame:", world_position)