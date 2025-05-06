import threading
import struct
import time
import socket

from collections import OrderedDict, deque
from std_msgs.msg import String
from python_server.service_calls.go_to_srv import GoToSrv
from python_server.service_calls.go_to_cmd_srv import GoToCmdSrv
from python_server.service_calls.set_states_srv import SetStatesSrv
from python_server.service_calls.waypoints_srv import WaypointsSrv
from python_server.msg.trigger_msg import TriggerMsg
from python_server.msg.params_msg import ParamsMsg
from python_server.msg.run_msg import RunMsg


class TcpConnection:
    def __init__(self, client_socket, on_packet, is_host=True, dashboard=False):
        self.socket = client_socket
        self.alive = True
        self.is_host = is_host
        self.is_dashboard = dashboard
        self.socket.settimeout(None)
        self.lock = threading.Lock()

        self.buf_size = 2097152
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, self.buf_size)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, self.buf_size)

        self.on_packet = on_packet
        self.on_start = None
        self.on_message = None
        self.on_run = None
        self.on_goto = None
        self.on_set_states = None
        self.on_waypoint = None
        self.on_params = None

        self.data_actions = OrderedDict({
            b'\x01': self.parse_string,
            b'\x02': self.parse_trigger,
            b'\x03': self.parse_message,
            b'\x04': self.parse_go_to_srv,
            b'\x05': self.parse_go_to_cmd_srv,
            b'\x06': self.parse_set_states_srv,
            b'\x07': self.parse_waypoints_srv,
            b'\x08': self.parse_start_srv,
            b'\x09': self.parse_params,
            b'\x0a': self.parse_run,
        })
        self.types = list(self.data_actions.keys())
        self.strings = deque()
        self.triggers = TriggerMsg(b'\x02')
        self.message = String()
        self.run_msg = RunMsg(b'\x0a')
        self.graph_msg = deque()
        self.go_to_srv_msg = GoToSrv(b'\x04')
        self.go_to_cmd_srv_msg = GoToCmdSrv(b'\x05')
        self.set_states_srv_msg = SetStatesSrv(b'\x06')
        self.waypoints_srv_msg = WaypointsSrv(b'\x07')
        self.start_srv_msg = False
        self.params = ParamsMsg(b'\x09')
        self.receiver = threading.Thread(target=self.receive, daemon=True)
        self.receiver.start()
        if is_host:
            self.send_string("ack")
        else:
            self.send_string("dashboard_client")
            self.send_string("ack")

    def refresh_run(self):
        self.send_string("refresh_run")

    def recvall(self, length):
        data = b""
        try:
            while len(data) < length:
                chunk = self.socket.recv(min(self.buf_size, length - len(data)))
                if not chunk:
                    return None
                data += chunk
            return data
        except Exception:
            return None

    def receive(self):
        header_size = 5
        message_size = 4
        while self.alive:
            try:
                # Receive the header (5 bytes)
                while self.alive:
                    try:
                        header = self.recvall(header_size)
                        if header is None:
                            continue
                        break
                    except Exception as e:
                        print(e)
                        continue
                length = struct.unpack('<I', header[:message_size])[0]
                message_type = header[message_size:header_size]
                # Receive the data based on the length from the header
                data = self.recvall(length)
                if data is None:
                    continue
                packet = header + data
                if self.on_packet:
                    self.on_packet(self, packet, self.lock)
                if self.is_dashboard and self.is_host:
                    continue
                # Process the data
                if message_type in self.data_actions:
                    self.data_actions[message_type](data)
            except Exception as e:
                print(e)
                continue

    ###################
    # Encode
    ###################

    def send_string(self, string):
        data = string.encode('utf-8')
        length = struct.pack('<I', len(string))
        bytes = length + self.types[0] + data
        with self.lock:
            self.socket.sendall(bytes)

    def send_trigger(self, request, response):
        bytes = self.triggers.encode(request, response)
        with self.lock:
            self.socket.sendall(bytes)

    def send_go_to_srv(self, vrefName, x0, y0, yaw0, dest_x, dest_y):
        bytes = self.go_to_srv_msg.encode(vrefName, x0, y0, dest_x, dest_y)
        with self.lock:
            self.socket.sendall(bytes)

    def send_go_to_cmd_srv(self, cursor_coords):
        bytes = self.go_to_cmd_srv_msg.encode(cursor_coords)
        with self.lock:
            self.socket.sendall(bytes)

    def send_set_states_srv(self, x, y):
        bytes = self.set_states_srv_msg.encode(x, y)
        with self.lock:
            self.socket.sendall(bytes)

    def send_waypoints_srv(self, vrefName, pathName, x0, y0, yaw0):
        bytes = self.waypoints_srv_msg.encode(vrefName, pathName, x0, y0, yaw0)
        with self.lock:
            self.socket.sendall(bytes)

    def send_start_srv(self, start):
        str = "start" if start else "stop"
        data = str.encode('utf-8')
        length = struct.pack('<I', len(str))
        bytes = length + self.types[7] + data
        with self.lock:
            self.socket.sendall(bytes)

    ###################
    # Decode
    ###################

    def parse_string(self, bytes):
        self.strings.append(bytes.decode('utf-8'))

    def parse_trigger(self, bytes):
        try:
            self.triggers.decode(bytes)
            req, res = self.triggers.msgs
        except Exception as e:
            print(e)

    def parse_message(self, bytes):
        try:
            self.message.deserialize(bytes)
            while self.on_message is None:
                time.sleep(0.2)
            self.on_message(self.message.data)
        except Exception as e:
            print(e)

    def parse_run(self, bytes):
        try:
            self.run_msg.decode(bytes)
            while self.on_run is None:
                time.sleep(0.2)
            self.on_run(self.run_msg)
        except Exception as e:
            print(e)

    def parse_go_to_cmd_srv(self, bytes):
        try:
            self.go_to_cmd_srv_msg.decode(bytes)
            while self.on_goto is None:
                time.sleep(0.2)
            self.on_goto(self.go_to_cmd_srv_msg)
        except Exception as e:
            print(e)

    def parse_set_states_srv(self, bytes):
        try:
            self.set_states_srv_msg.decode(bytes)
            while self.on_set_states is None:
                time.sleep(0.2)
            self.on_set_states(self.set_states_srv_msg.success)
        except Exception as e:
            print(e)

    def parse_waypoints_srv(self, bytes):
        try:
            self.waypoints_srv_msg.decode(bytes)
            while self.on_waypoint is None:
                time.sleep(0.2)
            self.on_waypoint(self.waypoints_srv_msg)
        except Exception as e:
            print(e)

    def parse_start_srv(self, bytes):
        try:
            # self.start_srv_msg = bytes == b'\x01'
            while self.on_start is None:
                time.sleep(0.2)
            self.on_start()
        except Exception as e:
            print(e)

    def parse_params(self, bytes):
        try:
            self.params.decode(bytes)
            while self.on_params is None:
                time.sleep(0.2)
            self.on_params(self.params)
        except Exception as e:
            print(e)

    def parse_go_to_srv(self, bytes):
        try:
            self.go_to_srv_msg.decode(bytes)
        except Exception as e:
            print(e)
