from python_server import decoder
from python_server import encoder
import struct
import io
from std_msgs.msg import Float32MultiArray, Float64MultiArray


class GoToCmdSrv:
    def __init__(self, data_type):
        self.bytes_length = 4
        self.num_elements = 2
        self.data_type = data_type
        self.state_refs = Float32MultiArray()
        self.input_refs = Float32MultiArray()
        self.wp_attributes = Float32MultiArray()
        self.wp_normals = Float32MultiArray()
        self.success = False

    def decode(self, bytes):
        try:
            splits = decoder.split(bytes)
            self.state_refs.deserialize(splits[0])
            self.input_refs.deserialize(splits[1])
            self.wp_attributes.deserialize(splits[2])
            self.wp_normals.deserialize(splits[3])
            self.success = splits[4] == b'\x01'
        except Exception as e:
            print(e)

    def encode(self, cursor_coords):
        x_coords = [x for x, y in cursor_coords]
        y_coords = [y for x, y in cursor_coords]
        msg_x = Float64MultiArray()
        msg_y = Float64MultiArray()
        msg_x.data = x_coords
        msg_y.data = y_coords
        x_buff = io.BytesIO()
        y_buff = io.BytesIO()
        msg_x.serialize(x_buff)
        msg_y.serialize(y_buff)
        data_bytes = []
        data_bytes.append(x_buff.getvalue())
        data_bytes.append(y_buff.getvalue())
        x_buff.seek(0)
        y_buff.seek(0)
        data_lengths = [len(element) for element in data_bytes]
        data_length = sum(data_lengths)
        lengths_length = (self.num_elements + 1) * self.bytes_length
        lengths_length_bytes = struct.pack('<I', lengths_length)
        lengths_bytes = lengths_length_bytes + b''.join([struct.pack('<I', element) for element in data_lengths])
        data_bytes = b''.join(data_bytes)
        return encoder.serialize(self.data_type, lengths_length, data_length, lengths_bytes, data_bytes)
