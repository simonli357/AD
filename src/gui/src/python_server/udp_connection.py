import threading
import numpy as np
import cv2
import struct

from collections import OrderedDict, deque
from std_msgs.msg import Float32MultiArray
from python_server.msg.lane2_msg import Lane2Msg
from python_server.msg.sw_load_msg import SWLoadMsg
from PyQt5.QtGui import QPixmap, QImage
from PyQt5.QtCore import QByteArray


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
                pix = QPixmap()
                ba = QByteArray(self.rgb_buf[0])
                pix.loadFromData(ba)
                return pix
            return None
        except Exception as e:
            print(e)
            return None

    def parse_depth_image(self):
        try:
            if len(self.depth_buf) > 0:
                pix = QPixmap()
                ba = QByteArray(self.depth_buf[0])
                pix.loadFromData(ba)
                return pix
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
