import socket
import signal
import sys
import time
import struct
import threading
from python_server.connection import Connection


class Server:
    def __init__(self, port):
        self.port = port
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.utility_node_client = None
        self.signs_node_client = None

    def initialize(self):
        signal.signal(signal.SIGINT, self.handle_signal)
        self.server_socket.bind(('', self.port))
        self.server_socket.listen(2)
        num_clients = 0
        while True:
            client_socket, client_address = self.server_socket.accept()
            threading.Thread(target=self.process_client, args=(client_socket,), daemon=True).start()
            num_clients += 1
            if num_clients == 2:
                # while not (self.signs_node_client):
                while not (self.utility_node_client and self.signs_node_client):
                    time.sleep(0.1)
                break
        print("All clients are connected.")

    def handle_signal(self, signal, frame):
        print("Caught SIGINT (Ctrl+C), closing socket...")
        if self.server_socket:
            self.server_socket.close()
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
            self.utility_node_client = Connection(client_socket)
        elif client_type == "sign_node_client":
            print("Signs Node Client connected")
            self.signs_node_client = Connection(client_socket)
        else:
            raise ConnectionError("Invalid client type")
