import threading


class Connection:
    def __init__(self, client_socket):
        self.socket = client_socket
        threading.Thread(target=self.receive, daemon=True).start()

    def receive():
        while True:
            data = self.socket.recv(1024)

    def sendString(self, str):
        self.socket.sendall(str.encode('utf-8'))
