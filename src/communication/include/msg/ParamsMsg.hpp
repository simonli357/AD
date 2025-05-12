#pragma once

#include "Encoder.hpp"
#include "std_msgs/Float32MultiArray.h"
#include <cstdint>
#include <vector>

class ParamsMsg : public Encoder {
  public:
	ParamsMsg() = default;
	ParamsMsg(ParamsMsg &&) = default;
	ParamsMsg(const ParamsMsg &) = default;
	ParamsMsg &operator=(ParamsMsg &&) = delete;
	ParamsMsg &operator=(const ParamsMsg &) = delete;
	~ParamsMsg() = default;

	std::vector<double> state_refs;
	std::vector<double> attributes;

	void encode(const std::vector<double> &state_refs, const std::vector<double> &attributes);

  private:
	const size_t bytes_length = 4;
	const size_t num_elements = 2;
	std_msgs::Float32MultiArray state_refs_arr;
	std_msgs::Float32MultiArray attributes_arr;
	uint32_t lengths_length = (num_elements + 1) * bytes_length;
	uint32_t data_length;
	uint32_t state_refs_length;
	uint32_t attributes_length;
	uint32_t compute_lengths_length() override;
	uint32_t compute_data_length() override;
	std::vector<uint8_t> get_lengths() override;
	std::vector<uint8_t> get_data() override;
};
