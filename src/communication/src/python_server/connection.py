import threading
import struct
from collections import OrderedDict
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String
from python_server.service_calls.go_to_srv import GoToSrv
from python_server.service_calls.go_to_cmd_srv import GoToCmdSrv
from python_server.service_calls.set_states_srv import SetStatesSrv
from python_server.service_calls.waypoints_srv import WaypointsSrv
from python_server.msg.lane2_msg import Lane2Msg
from python_server.msg.trigger_msg import TriggerMsg


class Connection:
    def __init__(self, client_socket=None):
        self.socket = client_socket
        if client_socket is not None:
            self.data_actions = OrderedDict({
                b'\x01': self.parse_string,
                b'\x02': self.parse_lane2,
                b'\x03': self.parse_trigger,
                b'\x04': self.parse_road_object,
                b'\x05': self.parse_waypoint,
                b'\x06': self.parse_sign,
                b'\x07': self.parse_message,
                b'\x08': self.parse_go_to_srv,
                b'\x09': self.parse_go_to_cmd_srv,
                b'\x0a': self.parse_set_states_srv,
                b'\x0b': self.parse_waypoints_srv,
                b'\x0c': self.parse_start_srv,
            })
            self.types = list(self.data_actions.keys())
            self.strings = []
            self.lane2 = Lane2Msg(b'\x02')
            self.triggers = TriggerMsg(b'\x03')
            self.road_objects = []
            self.waypoints = []
            self.signs = []
            self.messages = []
            self.go_to_srv_msg = GoToSrv(b'\x08')
            self.go_to_cmd_srv_msg = GoToCmdSrv(b'\x09')
            self.set_states_srv_msg = SetStatesSrv(b'\x0a')
            self.waypoints_srv_msg = WaypointsSrv(b'\x0b')
            self.start_srv_msg = False
            threading.Thread(target=self.receive, daemon=True).start()
            self.send_string("ack")

    def recvall(self, length):
        data = b""
        while len(data) < length:
            chunk = self.socket.recv(min(4096, length - len(data)))
            if not chunk:
                raise ConnectionError("Connection lost")
            data += chunk
        return data

    def receive(self):
        header_size = 5
        message_size = 4
        while True:
            # Receive the header (5 bytes)
            while True:
                header = self.socket.recv(header_size)
                if len(header) < header_size:
                    continue
                break
            length = struct.unpack('<I', header[:message_size])[0]
            message_type = header[message_size:header_size]
            # Receive the data based on the length from the header
            data = self.recvall(length)
            # Process the data
            if message_type in self.data_actions:
                self.data_actions[message_type](data)

    def send_string(self, str):
        data = str.encode('utf-8')
        length = struct.pack('<I', len(str))
        bytes = length + self.types[0] + data
        self.socket.sendall(bytes)

    def send_trigger(self, request, response):
        bytes = self.triggers.encode(request, response)
        self.socket.sendall(bytes)

    def send_go_to_srv(self, vrefName, x0, y0, yaw0, dest_x, dest_y):
        bytes = self.go_to_srv_msg.encode(vrefName, x0, y0, dest_x, dest_y)
        self.socket.sendall(bytes)

    def send_go_to_cmd_srv(self, dest_x, dest_y):
        bytes = self.go_to_cmd_srv_msg.encode(dest_x, dest_y)
        self.socket.sendall(bytes)

    def send_set_states_srv(self, x, y):
        bytes = self.set_states_srv_msg.encode(x, y)
        self.socket.sendall(bytes)

    def send_waypoints_srv(self, pathName, vrefName, x0, y0, yaw0):
        bytes = self.waypoints_srv_msg.encode(pathName, vrefName, x0, y0, yaw0)
        self.socket.sendall(bytes)

    def send_start_srv(self, start):
        str = "start" if start else "stop"
        data = str.encode('utf-8')
        length = struct.pack('<I', len(str))
        bytes = length + self.types[11] + data
        self.socket.sendall(bytes)

    def parse_string(self, bytes):
        self.strings.append(bytes.decode('utf-8'))

    def parse_lane2(self, bytes):
        try:
            self.lane2.decode(bytes)
        except Exception as e:
            print(e)

    def parse_trigger(self, bytes):
        try:
            self.triggers.decode(bytes)
        except Exception as e:
            print(e)

    def parse_road_object(self, bytes):
        try:
            self.road_objects.append(Float32MultiArray().deserialize(bytes))
        except Exception as e:
            print(e)

    def parse_waypoint(self, bytes):
        try:
            self.waypoints.append(Float32MultiArray().deserialize(bytes))
        except Exception as e:
            print(e)

    def parse_sign(self, bytes):
        try:
            self.signs.append(Float32MultiArray().deserialize(bytes))
        except Exception as e:
            print(e)

    def parse_message(self, bytes):
        try:
            self.messages.append(String().deserialize(bytes))
        except Exception as e:
            print(e)

    def parse_go_to_srv(self, bytes):
        try:
            self.go_to_srv_msg.decode(bytes)
        except Exception as e:
            print(e)

    def parse_go_to_cmd_srv(self, bytes):
        try:
            self.go_to_cmd_srv_msg.decode(bytes)
        except Exception as e:
            print(e)

    def parse_set_states_srv(self, bytes):
        try:
            self.set_states_srv_msg.decode(bytes)
        except Exception as e:
            print(e)

    def parse_waypoints_srv(self, bytes):
        try:
            self.waypoints_srv_msg.decode(bytes)
        except Exception as e:
            print(e)

    def parse_start_srv(self, bytes):
        try:
            self.start_srv_msg = bytes == b'\x01'
        except Exception as e:
            print(e)
