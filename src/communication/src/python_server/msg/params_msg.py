from python_server import decoder
from collections import deque
from std_msgs.msg import Float64MultiArray
import numpy as np


class ParamsMsg:
    def __init__(self, data_type):
        self.bytes_length = 4
        self.num_elements = 2
        self.data_type = data_type
        self.state_refs = deque()
        self.attributes = deque()

    def decode_double_array(self, msg):
        nparr = np.array(msg.data)
        return nparr.reshape(-1, 1)

    def decode(self, bytes):
        splits = decoder.split(bytes)
        state_refs = Float64MultiArray().deserialize(splits[0])
        attributes = Float64MultiArray().deserialize(splits[1])
        self.state_refs.append(self.decode_double_array(state_refs))
        self.attributes.append(self.decode_double_array(attributes))
