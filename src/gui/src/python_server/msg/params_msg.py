from python_server import decoder
from std_msgs.msg import Float32MultiArray
import numpy as np


class ParamsMsg:
    def __init__(self, data_type):
        self.bytes_length = 4
        self.num_elements = 2
        self.data_type = data_type
        self.state_refs = None
        self.attributes = None

    def decode(self, bytes):
        splits = decoder.split(bytes)
        state_refs = Float32MultiArray().deserialize(splits[0])
        attributes = Float32MultiArray().deserialize(splits[1])
        self.state_refs = np.array(state_refs.data).reshape(3, -1)
        self.attributes = np.array(attributes.data)
