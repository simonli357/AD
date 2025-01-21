from python_server import encoder
import struct


class SetStatesSrv:
    def __init__(self, data_type):
        self.bytes_length = 4
        self.num_elements = 2
        self.data_type = data_type
        self.success = False

    def decode(self, bytes):
        if bytes == b'\x01':
            self.success = True

    def encode(self, x, y):
        data_bytes = []
        data_bytes.append(str(x).encode('utf-8'))
        data_bytes.append(str(y).encode('utf-8'))
        data_lengths = [len(element) for element in data_bytes]
        data_length = sum(data_lengths)
        lengths_length = (self.num_elements + 1) * self.bytes_length
        lengths_length_bytes = struct.pack('<I', lengths_length)
        lengths_bytes = lengths_length_bytes + b''.join([struct.pack('<I', element) for element in data_lengths])
        data_bytes = b''.join(data_bytes)
        return encoder.serialize(self.data_type, lengths_length, data_length, lengths_bytes, data_bytes)
