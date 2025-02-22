from python_server import decoder
from std_msgs.msg import Float64MultiArray


class LanesMsg:
    def __init__(self):
        self.bytes_length = 4
        self.num_elements = 4
        self.lane1 = None
        self.lane2 = None

    def decode(self, bytes):
        splits = decoder.split(bytes)
        lane_msg = LanesMsg()
        lane1x = Float64MultiArray().deserialize(splits[0]).data
        lane1y = Float64MultiArray().deserialize(splits[1]).data
        lane2x = Float64MultiArray().deserialize(splits[2]).data
        lane2y = Float64MultiArray().deserialize(splits[3]).data
        lane_msg.lane1 = list(zip(lane1x, lane1y))
        lane_msg.lane2 = list(zip(lane2x, lane2y))
        return lane_msg
