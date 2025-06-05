import socket
import struct
import threading
import time

from python_server.tcp_connection import TcpConnection
from python_server.udp_connection import UdpConnection


class Server:
    def __init__(self, main_window, host=True, host_ip="127.0.0.1"):
        self.main_window = main_window
        self.is_host = host
        self.host_ip = host_ip
        self.tcp_port = 49153
        self.udp_port = 49154
        self.tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.tcp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self.udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.tcp_client = None
        self.dashboard_clients = {}
        self.udp_connection = None
        self.alive = True
        self.listener = None

    def initialize(self):
        self.udp_socket.bind(('', self.udp_port))
        self.udp_connection = UdpConnection(self.udp_socket, self)

        if self.is_host:
            self.tcp_socket.bind(('', self.tcp_port))
            self.tcp_socket.listen(5)
            listener = threading.Thread(target=self.listen, daemon=True)
            listener.start()
        else:
            print("Connecting to host dashboard")
            while True:
                try:
                    self.tcp_socket.connect((self.host_ip, self.tcp_port))
                    break
                except OSError:
                    time.sleep(0.5)
            print("Succesfully connected to host dashboard")
            self.tcp_client = TcpConnection(self.tcp_socket, self.on_packet, is_host=self.is_host, dashboard=False)
            self.main_window.set_callbacks()

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
            self.tcp_client = TcpConnection(client_socket, self.on_packet, is_host=self.is_host, dashboard=False)
            self.main_window.set_callbacks()
        if client_type == "dashboard_client":
            key = client_socket.getpeername()[0]
            print(f"Dashboard Client Connected with IP: {key}")
            if key in self.dashboard_clients:
                old_client = self.dashboard_clients[key]
                old_client.alive = False
                try:
                    old_client.socket.close()
                except Exception:
                    pass
            self.dashboard_clients[key] = TcpConnection(client_socket, self.on_packet, is_host=self.is_host, dashboard=True)

    def on_packet(self, source, packet):
        if source.is_host and not source.is_dashboard:
            for key, db in self.dashboard_clients.items():
                try:
                    with db.lock:
                        db.socket.sendall(packet)
                except OSError:
                    pass
        elif source.is_host and source.is_dashboard:
            try:
                with self.tcp_client.lock:
                    self.tcp_client.socket.sendall(packet)
            except Exception as e:
                print(e)
