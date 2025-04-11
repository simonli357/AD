from python_server import decoder
from python_server import encoder
from geometry_msgs.msg import PoseWithCovarianceStamped
from collections import deque

import struct


class GpsMsg:
    def __init__(self, data_type):
        self.bytes_length = 4
        self.num_elements = 4
        self.data_type = data_type
        self.pose = deque()

    def decode(self, bytes):
        try:
            splits = decoder.split(bytes)
            self.pose.append(PoseWithCovarianceStamped().deserialize(splits[0]))
        except Exception as e:
            print(e)

    def encode(self, x0: float, y0: float, yaw0: float, path: str):
        data_bytes = []
        data_bytes.append(struct.pack('<f', x0))
        data_bytes.append(struct.pack('<f', y0))
        data_bytes.append(struct.pack('<f', yaw0))
        data_bytes.append(path.encode('utf-8'))
        data_lengths = [len(element) for element in data_bytes]
        data_length = sum(data_lengths)
        lengths_length = (self.num_elements + 1) * self.bytes_length
        lengths_length_bytes = struct.pack('<I', lengths_length)
        lengths_bytes = lengths_length_bytes + b''.join([struct.pack('<I', element) for element in data_lengths])
        data_bytes = b''.join(data_bytes)
        return encoder.serialize(self.data_type, lengths_length, data_length, lengths_bytes, data_bytes)
