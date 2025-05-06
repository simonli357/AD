from python_server import decoder
import struct


class RunMsg:
    def __init__(self, data_type):
        self.bytes_length = 4
        self.num_elements = 5
        self.data_type = data_type
        self.vref_name = None
        self.path_name = None
        self.x_init = None
        self.y_init = None
        self.yaw_init = None

    def decode(self, bytes):
        splits = decoder.split(bytes)
        self.vref_name = struct.unpack('f', splits[0])[0]
        self.path_name = splits[1].decode('utf-8')
        self.x_init = struct.unpack('f', splits[2])[0]
        self.y_init = struct.unpack('f', splits[3])[0]
        self.yaw_init = struct.unpack('f', splits[4])[0]
