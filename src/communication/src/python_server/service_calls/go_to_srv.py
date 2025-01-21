from python_server import decoder
from python_server import encoder
import struct
from std_msgs.msg import Float32MultiArray


class GoToSrv:
    def __init__(self, data_type):
        self.bytes_length = 4
        self.num_elements = 6
        self.data_type = data_type
        self.state_refs = Float32MultiArray()
        self.input_refs = Float32MultiArray()
        self.wp_attributes = Float32MultiArray()
        self.wp_normals = Float32MultiArray()

    def decode(self, bytes):
        splits = decoder.split(bytes)
        self.state_refs.deserialize(splits[0])
        self.input_refs.deserialize(splits[1])
        self.wp_attributes.deserialize(splits[2])
        self.wp_normals.deserialize(splits[3])

    def encode(self, vrefName, x0, y0, yaw0, dest_x, dest_y):
        data_bytes = []
        data_bytes.append(vrefName.encode('utf-8'))
        data_bytes.append(str(x0).encode('utf-8'))
        data_bytes.append(str(y0).encode('utf-8'))
        data_bytes.append(str(yaw0).encode('utf-8'))
        data_bytes.append(str(dest_x).encode('utf-8'))
        data_bytes.append(str(dest_y).encode('utf-8'))
        data_lengths = [len(element) for element in data_bytes]
        data_length = sum(data_lengths)
        lengths_length = (self.num_elements + 1) * self.bytes_length
        lengths_length_bytes = struct.pack('<I', lengths_length)
        lengths_bytes = lengths_length_bytes + b''.join([struct.pack('<I', element) for element in data_lengths])
        data_bytes = b''.join(data_bytes)
        return encoder.serialize(self.data_type, lengths_length, data_length, lengths_bytes, data_bytes)
