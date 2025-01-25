import struct


def split(bytes):
    bytes_length = 4
    lengths_length = struct.unpack('<I', bytes[:bytes_length])[0]
    num_elements = (lengths_length - bytes_length) // bytes_length
    splits = []
    size_offset = bytes_length
    data_offset = lengths_length
    for i in range(num_elements):
        size = struct.unpack('<I', bytes[size_offset:size_offset + bytes_length])[0]
        size_offset += bytes_length
        split = bytes[data_offset:data_offset + size]
        splits.append(split)
        data_offset += size
    return splits
