from python_server import decoder
import struct


class ImuCalibMsg:
    def __init__(self):
        self.bytes_length = 4
        self.num_elements = 4
        self.sys_calib = None
        self.gyro_calib = None
        self.mag_calib = None
        self.accel_calib = None

    def decode(self, bytes):
        splits = decoder.split(bytes)
        imu_calib_msg = ImuCalibMsg()
        imu_calib_msg.sys_calib = struct.unpack('f', splits[0])[0]
        imu_calib_msg.gyro_calib = struct.unpack('f', splits[1])[0]
        imu_calib_msg.mag_calib = struct.unpack('f', splits[2])[0]
        imu_calib_msg.accel_calib = struct.unpack('f', splits[3])[0]
        return imu_calib_msg
