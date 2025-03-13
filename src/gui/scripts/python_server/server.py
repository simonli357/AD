import socket
import struct
import threading
from python_server.tcp_connection import TcpConnection
from python_server.udp_connection import UdpConnection


class Server:
    def __init__(self):
        self.tcp_port = 49153
        self.udp_port = 49154
        self.tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.tcp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.utility_node_client = TcpConnection()
        self.udp_connection = UdpConnection(self.udp_socket)
        self.alive = True
        self.listener = None

    def initialize(self):
        self.udp_socket.bind(('0.0.0.0', self.udp_port))
        self.tcp_socket.bind(('0.0.0.0', self.tcp_port))
        self.tcp_socket.listen(2)
        listener = threading.Thread(target=self.listen, daemon=True)
        listener.start()

    def listen(self):
        while self.alive:
            client_tcp_socket, _ = self.tcp_socket.accept()
            threading.Thread(target=self.process_client, args=(client_tcp_socket,), daemon=True).start()

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
