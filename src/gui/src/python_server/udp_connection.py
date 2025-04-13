import threading
import numpy as np
import cv2
import struct

from collections import OrderedDict, deque
from std_msgs.msg import Float32MultiArray
from python_server.msg.lane2_msg import Lane2Msg
from python_server.msg.sw_load_msg import SWLoadMsg
from PyQt5.QtGui import QPixmap, QImage


class UdpConnection:
    def __init__(self, udp_socket=None):
        self.socket = udp_socket
        self.MAX_DGRAM = 65507
        if udp_socket is not None:
            self.socket.settimeout(None)
            self.data_actions = OrderedDict({
                1: self.store_lane2,
                2: self.store_road_object,
                3: self.store_waypoint,
                4: self.store_sign,
                5: self.store_rgb_image,
                6: self.store_depth_image,
                7: self.store_steer,
                8: self.store_sw_load,
            })
            self.types = list(self.data_actions.keys())
            self.lane2_buf = deque([], 1)
            self.road_object_buf = deque([], 1)
            self.waypoint_buf = deque([], 1)
            self.sign_buf = deque([], 1)
            self.rgb_buf = deque([], 1)
            self.depth_buf = deque([], 1)
            self.steer_buf = deque([], 1)
            self.sw_load_buf = deque([], 1)
            threading.Thread(target=self.receive, daemon=True).start()

    def receive(self):
        while True:
            try:
                seg, _ = self.socket.recvfrom(self.MAX_DGRAM)
                if len(seg) < 6:
                    continue
                message_type = seg[4]
                bytes = seg[5:]
                if message_type in self.data_actions:
                    self.data_actions[message_type](bytes)
            except Exception as e:
                print(e)
                continue

    ###################
    # Data actions
    ###################

    def store_lane2(self, bytes):
        self.lane2_buf.append(bytes)

    def store_road_object(self, bytes):
        self.road_object_buf.append(bytes)

    def store_waypoint(self, bytes):
        self.waypoint_buf.append(bytes)

    def store_sign(self, bytes):
        self.sign_buf.append(bytes)

    def store_rgb_image(self, bytes):
        self.rgb_buf.append(bytes)

    def store_depth_image(self, bytes):
        self.depth_buf.append(bytes)

    def store_steer(self, bytes):
        self.steer_buf.append(bytes)

    def store_sw_load(self, bytes):
        self.sw_load_buf.append(bytes)

    ####################
    # Utility methods
    ####################

    def parse_lane2(self):
        try:
            if len(self.lane2_buf) > 0:
                return Lane2Msg().decode(self.lane2_buf[0])
            return None
        except Exception as e:
            print(e)
            return None

    def parse_road_object(self):
        try:
            if len(self.road_object_buf) > 0:
                return Float32MultiArray().deserialize(self.road_object_buf[0])
            return None
        except Exception as e:
            print(e)
            return None

    def parse_waypoint(self):
        try:
            if len(self.waypoint_buf) > 0:
                return Float32MultiArray().deserialize(self.waypoint_buf[0])
            return None
        except Exception as e:
            print(e)
            return None

    def parse_sign(self):
        try:
            if len(self.sign_buf) > 0:
                return Float32MultiArray().deserialize(self.sign_buf[0])
            return None
        except Exception as e:
            print(e)
            return None

    def parse_rgb_image(self):
        try:
            if len(self.rgb_buf) > 0:
                np_array = np.frombuffer(self.rgb_buf[0], dtype=np.uint8)
                cv_image = cv2.imdecode(np_array, cv2.IMREAD_COLOR)
                cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
                # Convert to QImage
                h, w, ch = cv_image.shape
                bytes_per_line = ch * w
                qt_image = QImage(cv_image.data, w, h, bytes_per_line, QImage.Format_RGB888)
                return QPixmap.fromImage(qt_image)
            return None
        except Exception as e:
            print(e)
            return None

    def parse_depth_image(self):
        try:
            if len(self.depth_buf) > 0:
                np_array = np.frombuffer(self.depth_buf[0], dtype=np.uint8)
                cv_image = cv2.imdecode(np_array, cv2.IMREAD_UNCHANGED)
                cv_image = (cv_image).astype(np.uint16)
                # depth_image = self.bridge.imgmsg_to_cv2(msg, "32FC1")
                # depth_image = self.bridge.imgmsg_to_cv2(depth_frame, "mono16")  # real
                # depth_image = cv2.resize(depth_image, (self.camera_w, self.camera_h))
                # Apply normalization with a focus on closer objects
                depth_normalized = cv2.normalize(cv_image, None, 50, 255, cv2.NORM_MINMAX)
                depth_colored = cv2.applyColorMap(depth_normalized.astype(np.uint8), cv2.COLORMAP_TURBO)  # TURBO colormap for better contrast
                # Convert to QImage
                h, w, ch = depth_colored.shape
                bytes_per_line = ch * w
                qt_image = QImage(depth_colored.data, w, h, bytes_per_line, QImage.Format_RGB888)
                return QPixmap.fromImage(qt_image)
            return None
        except Exception as e:
            print(e)
            return None

    def parse_steer(self):
        try:
            if len(self.steer_buf) > 0:
                bytes = self.steer_buf[0]
                return struct.unpack('f', bytes[:4])[0]
            return None
        except Exception as e:
            print(e)
            return None

    def parse_sw_load(self):
        try:
            if len(self.sw_load_buf) > 0:
                bytes = self.sw_load_buf[0]
                return SWLoadMsg().decode(bytes)
            return None
        except Exception as e:
            print(e)
            return None
