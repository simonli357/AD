#pragma once

#include "std_msgs/Float32MultiArray.h"
#include <cstdint>
#include <vector>

class Encoder {
  public:
	Encoder(uint8_t &data_type);
	Encoder(Encoder &&) = default;
	Encoder(const Encoder &) = default;
	Encoder &operator=(Encoder &&) = delete;
	Encoder &operator=(const Encoder &) = delete;
	virtual ~Encoder() = default;

	std::vector<uint8_t> serialize();
	std::vector<uint8_t> serializeFloat32MultiArray(std_msgs::Float32MultiArray &array);

  private:
	const size_t header_size = 5;
	const size_t message_size = 4;
	uint8_t data_type;
	virtual uint32_t compute_lengths_length(); // Size of lengths array
	virtual uint32_t compute_data_length();	   // Size of data
	virtual std::vector<uint8_t> get_lengths();
	virtual std::vector<uint8_t> get_data();
};
