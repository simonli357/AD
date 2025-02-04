from python_server import decoder
from python_server import encoder
import io
import struct
from std_srvs.srv import TriggerRequest, TriggerResponse
from collections import deque


class TriggerMsg:
    def __init__(self, data_type):
        self.bytes_length = 4
        self.num_elements = 0
        self.data_type = data_type
        self.msgs = deque()

    def decode(self, bytes):
        splits = decoder.split(bytes)
        request = TriggerRequest()
        response = TriggerResponse()
        if len(splits) == 1:
            request.deserialize(splits[0])
        if len(splits) == 2:
            request.deserialize(splits[0])
            response.deserialize(splits[1])
        self.msgs.append((request, response))

    def encode(self, request, response):
        self.num_elements = 0
        data_bytes = []
        request_buff = io.BytesIO()
        response_buff = io.BytesIO()
        request.serialize(request_buff)
        response.serialize(response_buff)
        if len(request_buff.getvalue()) > 0:
            data_bytes.append(request_buff.getvalue())
            self.num_elements += 1
        if len(response_buff.getvalue()) > 0:
            data_bytes.append(response_buff.getvalue())
            self.num_elements += 1
        request_buff.seek(0)
        response_buff.seek(0)
        data_lengths = [len(element) for element in data_bytes]
        data_length = sum(data_lengths)
        lengths_length = (self.num_elements + 1) * self.bytes_length
        lengths_length_bytes = struct.pack('<I', lengths_length)
        lengths_bytes = lengths_length_bytes + b''.join([struct.pack('<I', element) for element in data_lengths])
        data_bytes = b''.join(data_bytes)
        return encoder.serialize(self.data_type, lengths_length, data_length, lengths_bytes, data_bytes)
