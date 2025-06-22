#include "msg/ImuMsg.hpp"
#include <cstdint>

void ImuMsg::encode(float sys_calib, float gyro_calib, float mag_calib, float accel_calib) {
    this->sys_calib = sys_calib;
    this->gyro_calib = gyro_calib;
    this->mag_calib = mag_calib;
    this->accel_calib = accel_calib;
	sys_length = sizeof(sys_calib);
	gyro_calib_length = sizeof(gyro_calib);
    mag_calib_length = sizeof(mag_calib);
	accel_calib_length = sizeof(accel_calib);
	data_length = sys_length + gyro_calib_length + mag_calib_length + accel_calib_length;
}

uint32_t ImuMsg::compute_lengths_length() { return lengths_length; }

uint32_t ImuMsg::compute_data_length() { return data_length; }

std::vector<uint8_t> ImuMsg::get_lengths() {
	std::vector<uint8_t> lengths(lengths_length);
	std::memcpy(lengths.data(), &lengths_length, bytes_length);
	std::memcpy(lengths.data() + bytes_length, &sys_length, bytes_length);
	std::memcpy(lengths.data() + bytes_length * 2, &gyro_calib_length, bytes_length);
	std::memcpy(lengths.data() + bytes_length * 3, &mag_calib_length, bytes_length);
    std::memcpy(lengths.data() + bytes_length * 4, &accel_calib_length, bytes_length);
	return lengths;
}

std::vector<uint8_t> ImuMsg::get_data() {
	std::vector<uint8_t> data(data_length);

	size_t offset = 0;
	std::memcpy(data.data() + offset, &sys_calib, sys_length);
	offset += sys_length;

	std::memcpy(data.data() + offset, &gyro_calib, gyro_calib_length);
	offset += gyro_calib_length;

	std::memcpy(data.data() + offset, &mag_calib, mag_calib_length);
	offset += mag_calib_length;

    std::memcpy(data.data() + offset, &accel_calib, accel_calib_length);

	return data;
}
