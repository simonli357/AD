import struct


def serialize(data_type, lenghts_length, data_length, lengths, data):
    size = struct.pack('>I', lenghts_length + data_length)
    header_bytes = size + data_type
    return header_bytes + lengths + data
