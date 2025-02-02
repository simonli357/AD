from python_server import decoder
import struct
from std_msgs.msg import Header


class Lane2Msg:
    def __init__(self, data_type):
        self.bytes_length = 4
        self.num_elements = 5
        self.data_type = data_type
        self.header = None
        self.center = None
        self.stopline = None
        self.crosswalk = None
        self.dotted = None

    def decode(self, bytes):
        splits = decoder.split(bytes)
        self.header = Header().deserialize(splits[0])
        self.center = struct.unpack('f', splits[1])[0]
        self.stopline = struct.unpack('i', splits[2])[0]
        self.crosswalk = splits[3] == b'\x01'
        self.dotted = splits[4] == b'\x01'
