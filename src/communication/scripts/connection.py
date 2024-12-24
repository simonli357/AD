import threading
import struct
from collections import OrderedDict
from sensor_msgs.msg import Image
from io import BytesIO


class Connection:
    def __init__(self, client_socket):
        self.socket = client_socket
        self.data_actions = OrderedDict({
            '0x01': self.parse_string,
            '0x02': self.parse_image
        })
        self.types = [key.encode('utf-8') for key in self.data_actions.keys()]
        self.strings = []
        self.images = []
        threading.Thread(target=self.receive, daemon=True).start()

    def recvall(self, length):
        data = b""
        while len(data) < length:
            chunk = self.socket.recv(min(1024, length - len(data)))
            if not chunk:
                raise ConnectionError("Connection lost")
            data += chunk
        return data

    def receive(self):
        header_size = 5
        message_size = 4
        while True:
            # Receive the header (5 bytes)
            while True:
                header = self.socket.recv(header_size)
                if len(header) < header_size:
                    continue
                break
            length = struct.unpack('>I', header[:message_size])[0]
            message_type = header[message_size:header_size]
            message_type_str = '0x' + message_type.hex()
            print(length)
            print(message_type_str)
            # Receive the data based on the length from the header
            data = self.recvall(length)
            # Process the data
            if message_type_str in self.data_actions:
                self.data_actions[message_type_str](data)

    def send_string(self, str):
        data = str.encode('utf-8')
        length = struct.pack('>I', len(str))
        bytes = length + self.types[0] + data
        self.socket.sendall(bytes)

    def parse_string(self, data):
        self.strings.append(data.decode('utf-8'))

    def parse_image(self, data):
        print("parsing")
        try:
            img_msg = Image()
            img_msg.deserialize(data)
            self.images.append(img_msg)
        except Exception as e:
            print(e)
