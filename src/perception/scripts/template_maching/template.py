#!/usr/bin/env python3
import cv2
import numpy as np
import os
import yaml

import rospy
from sensor_msgs.msg import Image, Imu
from cv_bridge import CvBridge
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PoseWithCovarianceStamped

class Camera:
    def __init__(self, config):
        self.K = np.zeros([3, 3])  # Camera intrinsic matrix
        self.R = np.zeros([3, 3])  # Rotation matrix
        self.t = np.zeros([3, 1])  # Translation vector
        self.P = np.zeros([3, 4])  # Projection matrix
        self._initialize(config)
    def _initialize(self, config):
        self.setK(config["fx"], config["fy"], config["px"], config["py"])
        self.setR(np.deg2rad(config["yaw"]), np.deg2rad(config["pitch"]), np.deg2rad(config["roll"]))
        self.setT(config["XCam"], config["YCam"], config["ZCam"])
        self.updateP()
    def setK(self, fx, fy, px, py):
        self.K[0, 0] = fx
        self.K[1, 1] = fy
        self.K[0, 2] = px
        self.K[1, 2] = py
        self.K[2, 2] = 1.0
    def setR(self, yaw, pitch, roll):
        # Rotation matrices around each axis
        Rz = np.array([[np.cos(-yaw), -np.sin(-yaw), 0.0],
                       [np.sin(-yaw),  np.cos(-yaw), 0.0],
                       [0.0,           0.0,          1.0]])
        Ry = np.array([[np.cos(-pitch), 0.0, np.sin(-pitch)],
                       [0.0,            1.0, 0.0],
                       [-np.sin(-pitch), 0.0, np.cos(-pitch)]])
        Rx = np.array([[1.0,    0.0,           0.0],
                       [0.0,    np.cos(-roll), -np.sin(-roll)],
                       [0.0,    np.sin(-roll),  np.cos(-roll)]])
        # Switch axes (x = -y, y = -z, z = x)
        Rs = np.array([[0.0, -1.0, 0.0],
                       [0.0,  0.0, -1.0],
                       [1.0,  0.0, 0.0]])
        self.R = Rs.dot(Rz.dot(Ry.dot(Rx)))
    def setT(self, XCam, YCam, ZCam):
        X = np.array([XCam, YCam, ZCam])
        self.t = -self.R.dot(X)
    def updateP(self):
        Rt = np.zeros([3, 4])
        Rt[0:3, 0:3] = self.R
        Rt[0:3, 3] = self.t.ravel()
        self.P = self.K.dot(Rt)

current_dir = os.path.dirname(os.path.realpath(__file__))
camera_config_path = os.path.join(current_dir, "rs.yaml")
with open(os.path.abspath(camera_config_path)) as stream:
    frontConfig = yaml.safe_load(stream)
width_m = 1
height_m = 2
resolution = 200
cc = True  # Choose nearest neighbor if true; otherwise linear
pxPerM = (resolution, resolution)
outputRes = (int(width_m * pxPerM[0]), int(height_m * pxPerM[1]))
CONSTANT_SHIFT = 0.585
M = np.array([
    [1.0 / pxPerM[1], 0.0, CONSTANT_SHIFT],
    [0.0, -1.0 / pxPerM[0], width_m / 2.0],
    [0.0, 0.0, 0.0],
    [0.0, 0.0, 1.0]
])
cam = Camera(frontConfig)
IPM = np.linalg.inv(cam.P.dot(M))
interpMode = cv2.INTER_NEAREST if cc else cv2.INTER_LINEAR
# Global bridge and kernel
bridge = CvBridge()
kernel = np.ones((3, 3), np.uint8)
yaw_deg = 0
x, y = 0, 0

map_img = cv2.imread(os.path.join(current_dir, "Track.png"), cv2.IMREAD_GRAYSCALE)
width_m = 20.696
height_m = 13.786
desired_ppm = 50
desired_width_px = int(width_m * desired_ppm)
desired_height_px = int(height_m * desired_ppm)
map_img = cv2.resize(map_img, (desired_width_px, desired_height_px), interpolation=cv2.INTER_NEAREST)

def preprocess(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5, 5), 0)

    adaptive_thresh2 = cv2.adaptiveThreshold(
        gray, 255,
        cv2.ADAPTIVE_THRESH_MEAN_C,
        cv2.ADAPTIVE_THRESH_MEAN_C,
        199, -25
    )
    adaptive_refined2 = cv2.morphologyEx(adaptive_thresh2, cv2.MORPH_OPEN, kernel)
    adaptive_refined2 = cv2.morphologyEx(adaptive_refined2, cv2.MORPH_CLOSE, kernel)

    return adaptive_refined2

def rotate_image_and_mask(image, angle_deg, roi_mask=None):
    (h, w) = image.shape[:2]
    center = (w / 2, h / 2)
    rot_mat = cv2.getRotationMatrix2D(center, angle_deg, 1.0)

    cos = np.abs(rot_mat[0, 0])
    sin = np.abs(rot_mat[0, 1])
    new_w = int((h * sin) + (w * cos))
    new_h = int((h * cos) + (w * sin))

    rot_mat[0, 2] += (new_w / 2) - center[0]
    rot_mat[1, 2] += (new_h / 2) - center[1]

    rotated_img = cv2.warpAffine(image, rot_mat, (new_w, new_h), flags=cv2.INTER_NEAREST, borderValue=0)

    white_mask = roi_mask.copy() if roi_mask is not None else np.ones_like(image, dtype=np.uint8) * 255
    rotated_mask = cv2.warpAffine(white_mask, rot_mat, (new_w, new_h), flags=cv2.INTER_NEAREST, borderValue=0)

    return rotated_img, rotated_mask

def template_match(map_image, bev_image, mask=None, approx_position=None, search_radius=20, method=cv2.TM_CCOEFF_NORMED, use_full_map=False):
    if use_full_map or approx_position is None:
        roi = map_image
        offset_x = 0
        offset_y = 0
    else:
        h, w = bev_image.shape[:2]
        x, y = approx_position
        # Define ROI boundaries relative to the approx position and template size
        x1 = max(0, x - search_radius + w)
        y2 = map_image.shape[0] - max(0, y - search_radius)
        x2 = min(map_image.shape[1], x + search_radius + w)
        y1 = map_image.shape[0] - min(map_image.shape[0], y + search_radius)
        print("map shape:", map_image.shape, "x1:", x1, "y1:", y1, "x2:", x2, "y2:", y2)
        print("Approx position m:", (x / desired_ppm, y / desired_ppm), ", search_radius: ", search_radius/desired_ppm, ", ROI m:", (x1 / desired_ppm, y1 / desired_ppm), (x2 / desired_ppm, y2 / desired_ppm))
        roi = map_image[y1:y2, x1:x2]
        cv2.imshow("ROI", roi)
        cv2.waitKey(1)
        offset_x = x1
        offset_y = y1

    # Use the ROI mask if available
    if mask is not None and method in [cv2.TM_CCORR_NORMED, cv2.TM_SQDIFF]:
        result = cv2.matchTemplate(roi, bev_image, method, mask=mask)
    else:
        result = cv2.matchTemplate(roi, bev_image, method)

    min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(result)
    if method in [cv2.TM_SQDIFF, cv2.TM_SQDIFF_NORMED]:
        match_loc = min_loc
        match_val = min_val
    else:
        match_loc = max_loc
        match_val = max_val

    return (offset_x + match_loc[0], offset_y + match_loc[1]), match_val

def gps_callback(msg):
    global x, y
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    # add a uniform noise of 0.3m
    x += np.random.uniform(-0.3, 0.3)
    y += np.random.uniform(-0.3, 0.3)
def imu_callback(msg):
    global yaw_deg
    orientation = msg.orientation
    _, _, yaw_deg = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
    yaw_deg = np.degrees(yaw_deg)
def image_callback(msg):
    try:
        frame = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        frame = cv2.flip(frame, -1)
        processed = preprocess(frame)
        # processed = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        processed_warped = cv2.warpPerspective(
            processed, IPM, (outputRes[1], outputRes[0]),
            flags=interpMode
        )
        # resize to disired resolution
        processed_warped_resized = cv2.resize(
            processed_warped,
            (int(processed_warped.shape[1] * desired_ppm / resolution), int(processed_warped.shape[0] * desired_ppm / resolution)),
            interpolation=cv2.INTER_NEAREST
        )
        
        # Rotate image
        global yaw_deg, x, y
        rot_bev, rot_mask = rotate_image_and_mask(processed_warped_resized, yaw_deg)

        methods = [
          cv2.TM_CCOEFF_NORMED,
          cv2.TM_CCORR_NORMED, # bad
          cv2.TM_SQDIFF,
          cv2.TM_SQDIFF_NORMED,
        ]
        method = methods[0]
        mask_to_use = rot_mask if method in [cv2.TM_CCORR_NORMED, cv2.TM_SQDIFF] else None
        # best_pos, score = template_match(map_img, rot_bev, mask=mask_to_use, use_full_map=True, method=method)
        best_pos, score = template_match(map_img, rot_bev, mask=mask_to_use, use_full_map=False, method=method, approx_position=(int(x * desired_ppm), int(y * desired_ppm)), search_radius=100)
        print("Best match at:", best_pos, "Score:", score, "yaw:", yaw_deg)

        h, w = rot_bev.shape
        map_img_copy = map_img.copy()
        cv2.rectangle(map_img_copy, best_pos, (best_pos[0]+w, best_pos[1]+h), 128, 2)
        cv2.imshow("Match", map_img_copy)
        cv2.imshow("Processed image", processed)
        cv2.imshow("Processed BEV", processed_warped)
        cv2.imshow("Rotated BEV", rot_bev)
        # cv2.imshow("Mask", rot_mask)
        # cv2.imshow("Processed BEV", rot_bev)
        cv2.waitKey(1)

    except Exception as e:
        rospy.logerr("Error processing image: %s", str(e))

if __name__ == "__main__":
    rospy.init_node('template_matcher')
    rospy.Subscriber("/camera/color/image_raw", Image, image_callback)
    rospy.Subscriber("/car1/imu", Imu, imu_callback)
    rospy.Subscriber("/gps", PoseWithCovarianceStamped, gps_callback)
    rospy.loginfo("Template matcher node started. Listening for images...")
    rospy.spin()
