#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import os
import yaml
import numpy as np
import cv2

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

class IPMNode:
    def __init__(self):
        rospy.init_node('ipm_node', anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.callback_front)
        self.image = None
    def callback_front(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.image = cv2.flip(self.image, -1)

class IPMProcessor:
    def __init__(self):
        self.node = IPMNode()
        self.current_dir = os.path.dirname(os.path.realpath(__file__))
        self.camera_config_path = os.path.join(self.current_dir, "rs.yaml")
        with open(os.path.abspath(self.camera_config_path)) as stream:
            self.frontConfig = yaml.safe_load(stream)
        self.width_m = 2
        self.height_m = 2
        self.resolution = 200
        self.cc = True  # Choose nearest neighbor if true; otherwise linear
        self.pxPerM = (self.resolution, self.resolution)
        self.outputRes = (int(self.width_m * self.pxPerM[0]), int(self.height_m * self.pxPerM[1]))
        CONSTANT_SHIFT = 0.585
        self.M = np.array([
            [1.0 / self.pxPerM[1], 0.0, CONSTANT_SHIFT],
            [0.0, -1.0 / self.pxPerM[0], self.width_m / 2.0],
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 1.0]
        ])
        self.cam = Camera(self.frontConfig)
        self.IPM = np.linalg.inv(self.cam.P.dot(self.M))
        self.interpMode = cv2.INTER_NEAREST if self.cc else cv2.INTER_LINEAR

    def process_images(self):
      while not rospy.is_shutdown():
          image = self.node.image
          if image is None:
              rospy.sleep(0.1)  # 10 Hz
              continue
          warpedImage = cv2.warpPerspective(
              image, self.IPM, (self.outputRes[1], self.outputRes[0]),
              flags=self.interpMode
          )
          cv2.imshow("Warped Image", warpedImage)
          cv2.waitKey(1)

    def run(self):
        try:
            self.process_images()
        except rospy.ROSInterruptException:
            pass
        cv2.destroyAllWindows()

if __name__ == "__main__":
    processor = IPMProcessor()
    processor.run()
