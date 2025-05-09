from python_server import decoder
import struct
from std_msgs.msg import Float32MultiArray


class SWLoadMsg:
    def __init__(self):
        self.bytes_length = 4
        self.num_elements = 5
        self.cores_usage = None
        self.ram_usage = None
        self.temperature = None
        self.heap_usage = None
        self.stack_usage = None

    def decode(self, bytes):
        splits = decoder.split(bytes)
        lane_msg = SWLoadMsg()
        lane_msg.cores_usage = Float32MultiArray().deserialize(splits[0]).data
        lane_msg.ram_usage = struct.unpack('f', splits[1])[0]
        lane_msg.temperature = struct.unpack('f', splits[2])[0]
        lane_msg.heap_usage = struct.unpack('f', splits[3])[0]
        lane_msg.stack_usage = struct.unpack('f', splits[4])[0]
        return lane_msg
