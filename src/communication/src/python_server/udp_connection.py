import threading
import time
from collections import OrderedDict, deque
from std_msgs.msg import Float32MultiArray
from python_server.msg.lane2_msg import Lane2Msg


class UdpConnection:
    def __init__(self, udp_socket=None):
        self.socket = udp_socket
        self.MAX_DGRAM = 65507
        if udp_socket is not None:
            self.data_actions = OrderedDict({
                1: self.store_lane2,
                2: self.store_road_object,
                3: self.store_waypoint,
                4: self.store_sign,
            })
            self.types = list(self.data_actions.keys())
            self.lane2_buf = deque([], 100)
            self.road_object_buf = deque([], 100)
            self.waypoint_buf = deque([], 100)
            self.sign_buf = deque([], 100)
            self.lane2 = Lane2Msg(b'\x02')
            self.road_object = Float32MultiArray()
            self.waypoint = Float32MultiArray()
            self.sign = Float32MultiArray()
            threading.Thread(target=self.receive, daemon=True).start()
            threading.Thread(target=self.process, daemon=True).start()

    def receive(self):
        while True:
            try:
                seg, _ = self.socket.recvfrom(self.MAX_DGRAM)
                if len(seg) < 5:
                    continue
                message_type = seg[4]
                bytes = seg[5:]
                if len(bytes) == 0:
                    continue
                if message_type in self.data_actions:
                    self.data_actions[message_type](bytes)
            except Exception as e:
                print(e)
                continue

    def process(self):
        while True:
            if len(self.lane2_buf) > 0:
                self.parse_lane2(self.lane2_buf.popleft())
            if len(self.road_object_buf) > 0:
                self.parse_road_object(self.road_object_buf.popleft())
            if len(self.waypoint_buf) > 0:
                self.parse_waypoint(self.waypoint_buf.popleft())
            if len(self.sign_buf) > 0:
                self.parse_sign(self.sign_buf.popleft())
            time.sleep(0.008)

    def store_lane2(self, bytes):
        self.lane2_buf.append(bytes)

    def store_road_object(self, bytes):
        self.road_object_buf.append(bytes)

    def store_waypoint(self, bytes):
        self.waypoint_buf.append(bytes)

    def store_sign(self, bytes):
        self.sign_buf.append(bytes)

    def parse_lane2(self, bytes):
        try:
            self.lane2.decode(bytes)
        except Exception as e:
            print(e)

    def parse_road_object(self, bytes):
        try:
            self.road_object.deserialize(bytes)
        except Exception as e:
            print(e)

    def parse_waypoint(self, bytes):
        try:
            self.waypoint.deserialize(bytes)
        except Exception as e:
            print(e)

    def parse_sign(self, bytes):
        try:
            self.sign.deserialize(bytes)
        except Exception as e:
            print(e)
