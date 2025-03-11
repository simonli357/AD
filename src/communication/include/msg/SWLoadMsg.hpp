#pragma once

#include "Encoder.hpp"
#include "std_msgs/Float64MultiArray.h"
#include <cstdint>
#include <vector>

class SWLoadMsg : public Encoder {
  public:
	SWLoadMsg(std_msgs::Float64MultiArray &cores_usage, float cpu_usage, float ram_usage, float temperature, float heap_usage, float stack_usage);
	SWLoadMsg(SWLoadMsg &&) = default;
	SWLoadMsg(const SWLoadMsg &) = default;
	SWLoadMsg &operator=(SWLoadMsg &&) = delete;
	SWLoadMsg &operator=(const SWLoadMsg &) = delete;
	~SWLoadMsg() = default;

	std_msgs::Float64MultiArray &cores_usage;
	float cpu_usage;
	float ram_usage;
	float temperature;
	float heap_usage;
	float stack_usage;

  private:
	const size_t bytes_length = 4;
	const size_t num_elements = 6;
	uint32_t lengths_length = (num_elements + 1) * bytes_length;
	uint32_t data_length;
	uint32_t cores_usage_length;
	uint32_t cpu_usage_length;
	uint32_t ram_usage_length;
	uint32_t temperature_length;
	uint32_t heap_usage_length;
	uint32_t stack_usage_length;
	uint32_t compute_lengths_length() override;
	uint32_t compute_data_length() override;
	std::vector<uint8_t> get_lengths() override;
	std::vector<uint8_t> get_data() override;
};
