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

    def decode(self, bytes):
        splits = decoder.split(bytes)
        state_refs = Float64MultiArray().deserialize(splits[0])
        attributes = Float64MultiArray().deserialize(splits[1])
        self.state_refs.append(np.array(state_refs.data).reshape(3, -1))
        self.attributes.append(np.array(attributes.data))
