import socket
import signal
import sys
import struct
import threading
from python_server.tcp_connection import TcpConnection
from python_server.udp_connection import UdpConnection
from python_server.video import VideoConnection


class Server:
    def __init__(self):
        self.tcp_port = 49153
        self.udp_port = 49154
        self.udp_rgb_port = 49155
        self.udp_depth_port = 49156
        self.tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_rgb_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_depth_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.utility_node_client = TcpConnection()
        self.sign_node_client = TcpConnection()
        self.udp_connection = UdpConnection(self.udp_socket)
        self.rgb_stream = VideoConnection(self.udp_rgb_socket, 'bgr8')
        self.depth_stream = VideoConnection(self.udp_depth_socket, '32FC1')

    def initialize(self):
        signal.signal(signal.SIGINT, self.handle_signal)
        self.udp_socket.bind(('', self.udp_port))
        self.udp_rgb_socket.bind(('', self.udp_rgb_port))
        self.udp_depth_socket.bind(('', self.udp_depth_port))
        self.tcp_socket.bind(('', self.tcp_port))
        self.tcp_socket.listen(2)
        threading.Thread(target=self.listen, daemon=True).start()

    def listen(self):
        while True:
            client_tcp_socket, _ = self.tcp_socket.accept()
            threading.Thread(target=self.process_client, args=(client_tcp_socket,), daemon=True).start()

    def handle_signal(self, signal, frame):
        print("Caught SIGINT (Ctrl+C), closing sockets...")
        if self.tcp_socket:
            self.tcp_socket.close()
            self.udp_socket.close()
            self.udp_rgb_socket.close()
            self.udp_depth_socket.close()
        sys.exit(0)

    def get_client_type(self, socket):
        header_size = 5
        message_size = 4
        while True:
            while True:
                header = socket.recv(header_size)
                if len(header) < header_size:
                    continue
                break
            length = struct.unpack('<I', header[:message_size])[0]
            message_type = header[message_size:header_size]
            data = socket.recv(length)
            if message_type == b'\x01':
                return data.decode('utf-8')

    def process_client(self, client_socket):
        client_type = self.get_client_type(client_socket)
        if client_type == "utility_node_client":
            print("Utility Client connected")
            self.utility_node_client = TcpConnection(client_socket)
        if client_type == "sign_node_client":
            print("Sign Client connected")
            self.sign_node_client = TcpConnection(client_socket)
