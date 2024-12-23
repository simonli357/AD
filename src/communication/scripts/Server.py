import socket
import threading


class Server:
    def __init__(self, port):
        self.port = port
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    def initialize(self):
        self.server_socket.bind(("127.0.0.1", self.port))
        self.server_socket.listen(1)
        threading.Thread(target=self.listen, daemon=True).start()

    def listen(self):
        while (True):
            client_socket, client_address = self.server_socket.accept()
