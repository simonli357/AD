#pragma once

#include "Encoder.hpp"
#include <cstddef>
#include <cstdint>
#include <vector>

class ImuMsg : public Encoder {
  public:
	ImuMsg() = default;
	ImuMsg(ImuMsg &&) = default;
	ImuMsg(const ImuMsg &) = default;
	ImuMsg &operator=(ImuMsg &&) = delete;
	ImuMsg &operator=(const ImuMsg &) = delete;
	~ImuMsg() = default;

	float sys_calib;
	float gyro_calib;
	float mag_calib;
	float accel_calib;

	void encode(float sys_calib, float gyro_calib, float mag_calib, float accel_calib);

  private:
	const size_t bytes_length = 4;
	const size_t num_elements = 4;
	uint32_t lengths_length = (num_elements + 1) * bytes_length;
	uint32_t data_length;
	uint32_t sys_length;
	uint32_t gyro_calib_length;
	uint32_t mag_calib_length;
	uint32_t accel_calib_length;
	uint32_t compute_lengths_length() override;
	uint32_t compute_data_length() override;
	std::vector<uint8_t> get_lengths() override;
	std::vector<uint8_t> get_data() override;
};
