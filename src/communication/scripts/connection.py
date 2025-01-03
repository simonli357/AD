import threading
import struct
from collections import OrderedDict
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String


class Connection:
    def __init__(self, client_socket):
        self.socket = client_socket
        self.data_actions = OrderedDict({
            b'\x01': self.parse_string,
            b'\x02': self.parse_image,
            b'\x03': self.parse_float32_multi_array,
            b'\x04': self.parse_message,
        })
        self.types = list(self.data_actions.keys())
        self.strings = []
        self.images = []
        self.arrays = []
        self.messages = []
        threading.Thread(target=self.receive, daemon=True).start()

    def recvall(self, length):
        data = b""
        while len(data) < length:
            chunk = self.socket.recv(min(900000, length - len(data)))
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
            # Receive the data based on the length from the header
            data = self.recvall(length)
            # Process the data
            if message_type in self.data_actions:
                self.data_actions[message_type](data)

    def send_string(self, str):
        data = str.encode('utf-8')
        length = struct.pack('>I', len(str))
        bytes = length + self.types[0] + data
        self.socket.sendall(bytes)

    def parse_string(self, data):
        self.strings.append(data.decode('utf-8'))

    def parse_image(self, data):
        try:
            img_msg = Image()
            img_msg.deserialize(data)
            self.images.append(img_msg)
        except Exception as e:
            print(e)

    def parse_float32_multi_array(self, data):
        try:
            float32_array = Float32MultiArray()
            float32_array.deserialize(data)
            self.arrays.append(float32_array)
        except Exception as e:
            print(e)

    def parse_message(self, data):
        try:
            msg = String()
            msg.deserialize(data)
            self.messages.append(msg)
        except Exception as e:
            print(e)
