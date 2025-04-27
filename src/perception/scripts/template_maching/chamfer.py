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
        self.K = np.zeros((3, 3), dtype=np.float64)
        self.R = np.eye(3, dtype=np.float64)
        self.t = np.zeros((3, 1), dtype=np.float64)
        self.P = np.zeros((3, 4), dtype=np.float64)
        self._initialize(config)

    def _initialize(self, config):
        self.setK(config["fx"], config["fy"], config["px"], config["py"])
        self.setR(np.deg2rad(config["yaw"]),
                  np.deg2rad(config["pitch"]),
                  np.deg2rad(config["roll"]))
        self.setT(config["XCam"], config["YCam"], config["ZCam"])
        self.updateP()

    def setK(self, fx, fy, px, py):
        self.K[:] = 0
        self.K[0, 0] = fx
        self.K[1, 1] = fy
        self.K[0, 2] = px
        self.K[1, 2] = py
        self.K[2, 2] = 1.0

    def setR(self, yaw, pitch, roll):
        # ZYX Euler
        Rz = np.array([[ np.cos(-yaw), -np.sin(-yaw), 0],
                       [ np.sin(-yaw),  np.cos(-yaw), 0],
                       [            0,             0, 1]])
        Ry = np.array([[ np.cos(-pitch), 0, np.sin(-pitch)],
                       [              0, 1,              0],
                       [-np.sin(-pitch), 0, np.cos(-pitch)]])
        Rx = np.array([[1,             0,              0],
                       [0, np.cos(-roll), -np.sin(-roll)],
                       [0, np.sin(-roll),  np.cos(-roll)]])
        # switch axes
        Rs = np.array([[ 0, -1,  0],
                       [ 0,  0, -1],
                       [ 1,  0,  0]])
        self.R = Rs @ (Rz @ (Ry @ Rx))

    def setT(self, XCam, YCam, ZCam):
        C = np.array([XCam, YCam, ZCam], dtype=np.float64)
        self.t = -self.R @ C.reshape((3,1))

    def updateP(self):
        Rt = np.hstack((self.R, self.t))
        self.P = self.K @ Rt

# --- Load camera configuration and compute IPM warp matrix ---
current_dir = os.path.dirname(os.path.realpath(__file__))
camera_config_path = os.path.join(current_dir, "rs.yaml")
with open(camera_config_path, 'r') as f:
    frontConfig = yaml.safe_load(f)

# Bird's-eye parameters
width_m, height_m = 0.5, 2.0  # camera FoV footprint in meters
pxPerM = (200, 200)  # resolution: px per meter (y, x)
outputRes = (int(height_m * pxPerM[0]), int(width_m * pxPerM[1]))
CONSTANT_SHIFT = 0.585
M = np.array([
    [1.0/pxPerM[1],        0, CONSTANT_SHIFT],
    [           0, -1.0/pxPerM[0], width_m/2],
    [           0,        0,              0],
    [           0,        0,              1]
], dtype=np.float64)
cam = Camera(frontConfig)
IPM = np.linalg.inv(cam.P @ M)
interpMode = cv2.INTER_NEAREST

# --- Load global map and build distance transform once ---
map_img = cv2.imread(os.path.join(current_dir, "Track.png"), cv2.IMREAD_GRAYSCALE)
# resize to desired PPM
map_w_m, map_h_m = 20.696, 13.786  # map size in meters
desired_ppm = 50  # pixels per meter for map
map_w_px = int(map_w_m * desired_ppm)
map_h_px = int(map_h_m * desired_ppm)
map_img = cv2.resize(map_img, (map_w_px, map_h_px), interpolation=cv2.INTER_NEAREST)
# binarize map: white lanes = 255, background = 0
_, map_bin = cv2.threshold(map_img, 127, 255, cv2.THRESH_BINARY)
# invert so that lines=0, bg=255 for distanceTransform
map_inv = cv2.bitwise_not(map_bin)
# compute distance transform (float32), distances in pixels
map_dt = cv2.distanceTransform(map_inv, cv2.DIST_L2, 5)

# reusable bridge and kernel
driver = CvBridge()
kernel = np.ones((3,3), np.uint8)

yaw_deg = 0.0
x_gps = 0.0
y_gps = 0.0

# --- Chamfer matching function ---
def chamfer_match(dt_image, bev_image, mask=None,
                  approx_position=None, search_radius=50,
                  use_full_map=False):
    """
    Performs chamfer matching between a binary BEV image and
    a precomputed distance-transform map_dt.
    - dt_image: float32 distance transform of the map (pixels to nearest lane).
    - bev_image: binary image (0 background, >0 lanes).
    - mask: optional binary mask (same size as bev_image) to ignore noisy borders.
    - approx_position: (x,y) pixel coords in dt_image of where bev_image top-left should be.
    - search_radius: in pixels, how far around approx_position to search in x and y.
    - use_full_map: if True, ignore approx_position and search entire dt_image.

    Returns (best_x, best_y), best_cost.
    """
    # build binary mask of BEV
    bev_mask = (bev_image > 0).astype(np.float32)
    if mask is not None:
        bev_mask *= (mask > 0).astype(np.float32)
    num_pix = bev_mask.sum()
    if num_pix < 1:
        # no features to match
        if approx_position is not None:
            return approx_position, float('inf')
        else:
            return (0, 0), float('inf')

    h_b, w_b = bev_mask.shape
    # choose ROI in dt_image
    if use_full_map or approx_position is None:
        dt_roi = dt_image
        offset_x, offset_y = 0, 0
    else:
        x0, y0 = approx_position
        # clamp top-left search range
        x_min = max(0, x0 - search_radius)
        x_max = min(dt_image.shape[1] - w_b, x0 + search_radius)
        y_min = max(0, y0 - search_radius)
        y_max = min(dt_image.shape[0] - h_b, y0 + search_radius)
        # extract ROI large enough to slide bev over
        dt_roi = dt_image[y_min:y_max + h_b, x_min:x_max + w_b]
        offset_x, offset_y = x_min, y_min
    # perform correlation (sum of dt under bev_mask)
    # result dims = (dt_roi_h - h_b + 1, dt_roi_w - w_b + 1)
    cost_map = cv2.filter2D(dt_roi, cv2.CV_32F,
                            bev_mask, anchor=(0, 0),
                            borderType=cv2.BORDER_CONSTANT)
    # find best (minimal) cost
    min_val, _, min_loc, _ = cv2.minMaxLoc(cost_map)
    best_loc = min_loc  # (x, y) in cost_map
    best_x = offset_x + best_loc[0]
    best_y = offset_y + best_loc[1]
    # average distance per pixel
    best_cost = min_val / num_pix
    return (best_x, best_y), best_cost

# --- ROS callbacks ---
def gps_callback(msg):
    global x_gps, y_gps
    x_gps = msg.pose.pose.position.x
    y_gps = msg.pose.pose.position.y
    # simulate sensor noise
    x_gps += np.random.uniform(-0.3, 0.3)
    y_gps += np.random.uniform(-0.3, 0.3)


def imu_callback(msg):
    global yaw_deg
    _, _, yaw = euler_from_quaternion([
        msg.orientation.x,
        msg.orientation.y,
        msg.orientation.z,
        msg.orientation.w])
    yaw_deg = np.degrees(yaw)


def preprocess(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5,5), 0)
    thresh = cv2.adaptiveThreshold(
        blur, 255,
        cv2.ADAPTIVE_THRESH_MEAN_C,
        cv2.THRESH_BINARY,
        199, -25)
    # clean up
    opened = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
    closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, kernel)
    return closed


def rotate_image_and_mask(image, angle_deg, roi_mask=None):
    h, w = image.shape[:2]
    center = (w/2, h/2)
    M_rot = cv2.getRotationMatrix2D(center, angle_deg, 1.0)
    # compute new dims
    cos = abs(M_rot[0,0]); sin = abs(M_rot[0,1])
    new_w = int(h*sin + w*cos)
    new_h = int(h*cos + w*sin)
    M_rot[0,2] += (new_w/2 - center[0])
    M_rot[1,2] += (new_h/2 - center[1])
    rot_img = cv2.warpAffine(image, M_rot, (new_w, new_h), flags=cv2.INTER_NEAREST, borderValue=0)
    if roi_mask is not None:
        rot_mask = cv2.warpAffine(roi_mask, M_rot, (new_w, new_h), flags=cv2.INTER_NEAREST, borderValue=0)
    else:
        rot_mask = np.ones_like(rot_img, dtype=np.uint8) * 255
    return rot_img, rot_mask


def image_callback(msg):
    try:
        frame = driver.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        frame = cv2.flip(frame, -1)
        proc = preprocess(frame)
        bev = cv2.warpPerspective(proc, IPM, (outputRes[1], outputRes[0]), flags=interpMode)
        # match map resolution
        bev = cv2.resize(bev,
                         (int(bev.shape[1]*desired_ppm/pxPerM[1]),
                          int(bev.shape[0]*desired_ppm/pxPerM[0])),
                         interpolation=cv2.INTER_NEAREST)
        rot_bev, rot_mask = rotate_image_and_mask(bev, yaw_deg)
        approx_px = int(x_gps * desired_ppm)
        approx_py = int(y_gps * desired_ppm)
        # chamfer matching around GPS prior
        best_pos, cost = chamfer_match(
            map_dt, rot_bev, mask=rot_mask,
            approx_position=(approx_px, approx_py),
            search_radius=100,
            use_full_map=False)
        print(f"Chamfer best at {best_pos} cost={cost:.3f} yaw={yaw_deg:.1f}")
        # visualize
        mvis = cv2.cvtColor(map_bin, cv2.COLOR_GRAY2BGR)
        h_b, w_b = rot_bev.shape
        cv2.rectangle(mvis, best_pos, (best_pos[0]+w_b, best_pos[1]+h_b), (0,255,0), 2)
        cv2.imshow("Chamfer Match", mvis)
        cv2.imshow("BEV", bev)
        cv2.imshow("Rotated BEV", rot_bev)
        cv2.waitKey(1)
    except Exception as e:
        rospy.logerr(f"Error in image_callback: {e}")

if __name__ == '__main__':
    rospy.init_node('chamfer_localizer')
    rospy.Subscriber('/gps', PoseWithCovarianceStamped, gps_callback)
    rospy.Subscriber('/car1/imu', Imu, imu_callback)
    rospy.Subscriber('/camera/color/image_raw', Image, image_callback)
    rospy.loginfo("Chamfer-localization node started.")
    rospy.spin()
