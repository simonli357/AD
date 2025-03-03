from python_server import decoder
import struct


class ObjectMsg:
    def __init__(self, data_type):
        self.bytes_length = 4
        self.num_elements = 5
        self.data_type = data_type
        self.obj_type = None
        self.x = None
        self.y = None

    def decode(self, bytes):
        splits = decoder.split(bytes)
        object_msg = ObjectMsg(self.data_type)
        object_msg.obj_type = splits[0].decode('utf-8')
        object_msg.x = struct.unpack('f', splits[1])[0]
        object_msg.y = struct.unpack('f', splits[2])[0]
        return object_msg
