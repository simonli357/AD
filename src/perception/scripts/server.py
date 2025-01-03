import socket
from connection import Connection


class Server:
    def __init__(self, port):
        self.port = port
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.connection = None

    def initialize(self):
        self.server_socket.bind(('', self.port))
        self.server_socket.listen(1)
        client_socket, client_address = self.server_socket.accept()
        self.connection = Connection(client_socket)

    def get_client(self):
        return self.connection
