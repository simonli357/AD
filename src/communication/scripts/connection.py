import threading
import struct
from collections import OrderedDict


class Connection:
    def __init__(self, client_socket):
        self.socket = client_socket
        # Maps type to the first byte of the message. [fn -> send, fn -> parse, queue]
        self.data_actions = OrderedDict({
            b'\x01': self.parse_string,
        })
        self.types = list(self.data_actions.keys())
        threading.Thread(target=self.receive, daemon=True).start()

    def receive(self):
        while True:
            # Receive the header (5 bytes)
            while True:
                header = self.socket.recv(5)
                if len(header) < 5:
                    continue
                break
            length = struct.unpack('>I', header[:4])[0]
            message_type = header[4:5]
            # Receive the data based on the length from the header
            while True:
                data = self.socket.recv(length)
                if len(data) < length:
                    continue
                break

            # Process the data
            if message_type in self.data_actions:
                self.data_actions[message_type](data)

    def send_string(self, str):
        data = str.encode('utf-8')
        length = struct.pack('>I', len(str))
        bytes = length + self.types[0] + data
        self.socket.sendall(bytes)

    def parse_string(self, data):
        print(data.decode('utf-8'))
