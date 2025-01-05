import socket
import struct
import threading
from connection import Connection


class Server:
    def __init__(self, port):
        self.port = port
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.utility_client = None
        self.camera_node_client = None
        self.signs_node_client = None

    def initialize(self):
        threading.Thread(target=self.listen, daemon=True).start()

    def recvall(self, length):
        data = b""
        while len(data) < length:
            chunk = self.socket.recv(min(8192, length - len(data)))
            if not chunk:
                raise ConnectionError("Connection lost")
            data += chunk
        return data

    def get_client_type(self, socket):
        header_size = 5
        message_size = 4
        while True:
            # Receive the header (5 bytes)
            while True:
                header = socket.recv(header_size)
                if len(header) < header_size:
                    continue
                break
            length = struct.unpack('>I', header[:message_size])[0]
            message_type = header[message_size:header_size]
            # Receive the data based on the length from the header
            data = self.recvall(length)
            # Process the data
            if message_type == 'b\x01':
                return data.decode('utf-8')
            else:
                return None

    def listen(self):
        self.server_socket.bind(('', self.port))
        self.server_socket.listen(7)
        while (True):
            client_socket, client_address = self.server_socket.accept()
            client_type = self.get_client_type(client_socket)
            if client_type == "utility_client":
                self.utility_client = Connection(client_socket)
            elif client_type == "camera_node_client":
                self.camera_node_client = Connection(client_socket)
            elif client_type == "signs_node_client":
                self.signs_node_client = Connection(client_socket)
            else:
                raise ConnectionError("Invalid client type")
