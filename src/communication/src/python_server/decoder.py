import struct


def split(bytes):
    bytes_length = 4
    lengths_length = struct.unpack('>I', bytes[:4])
    num_elements = (lengths_length - bytes_length) / bytes_length
    splits = []
    offset = 0
    for i in range(num_elements):
        s = (i + 1) * bytes_length
        size = struct.unpack('>I', bytes[s - 1:s + bytes_length])
        start = lengths_length - 1 + offset
        split = bytes[start:start + size]
        splits.append(split)
        offset += size
    return splits
