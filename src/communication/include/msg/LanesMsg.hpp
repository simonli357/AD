#pragma once

#include "Encoder.hpp"
#include "std_msgs/Float64MultiArray.h"
#include <cstdint>
#include <tuple>

class LanesMsg : public Encoder {
  public:
	LanesMsg();
	LanesMsg(std::vector<std::tuple<float, float>> &lane1, std::vector<std::tuple<float, float>> &lane2);
	LanesMsg(LanesMsg &&) = default;
	LanesMsg(const LanesMsg &) = default;
	LanesMsg &operator=(LanesMsg &&) = delete;
	LanesMsg &operator=(const LanesMsg &) = delete;
	~LanesMsg() = default;

  private:
	const size_t bytes_length = 4;
	std_msgs::Float64MultiArray lane1x = std_msgs::Float64MultiArray();
	std_msgs::Float64MultiArray lane1y = std_msgs::Float64MultiArray();
	std_msgs::Float64MultiArray lane2x = std_msgs::Float64MultiArray();
	std_msgs::Float64MultiArray lane2y = std_msgs::Float64MultiArray();
	const size_t num_elements = 4;
	uint32_t lengths_length = (num_elements + 1) * bytes_length;
	uint32_t data_length;
	uint32_t lane1x_length;
	uint32_t lane1y_length;
	uint32_t lane2x_length;
	uint32_t lane2y_length;
	uint32_t compute_lengths_length() override;
	uint32_t compute_data_length() override;
	std::vector<uint8_t> get_lengths() override;
	std::vector<uint8_t> get_data() override;
};
