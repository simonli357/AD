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
        lane_msg = Lane2Msg(self.data_type)
        lane_msg.header = Header().deserialize(splits[0])
        lane_msg.center = struct.unpack('f', splits[1])[0]
        lane_msg.stopline = struct.unpack('i', splits[2])[0]
        lane_msg.crosswalk = splits[3] == b'\x01'
        lane_msg.dotted = splits[4] == b'\x01'
        return lane_msg
