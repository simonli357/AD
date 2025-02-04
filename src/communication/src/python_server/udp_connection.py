import threading
from collections import OrderedDict
from std_msgs.msg import Float32MultiArray
from python_server.msg.lane2_msg import Lane2Msg


class UdpConnection:
    def __init__(self, udp_socket=None):
        self.socket = udp_socket
        self.MAX_DGRAM = 65507
        if udp_socket is not None:
            self.data_actions = OrderedDict({
                1: self.parse_lane2,
                2: self.parse_road_object,
                3: self.parse_waypoint,
                4: self.parse_sign,
            })
            self.types = list(self.data_actions.keys())
            self.lane2 = Lane2Msg(b'\x02')
            self.road_object = Float32MultiArray()
            self.waypoint = Float32MultiArray()
            self.sign = Float32MultiArray()
            threading.Thread(target=self.receive, daemon=True).start()

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
