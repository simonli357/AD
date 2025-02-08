#pragma once

#include "Decoder.hpp"
#include "Encoder.hpp"
#include "std_msgs/Float64MultiArray.h"
#include <cstdint>
#include <optional>
#include <vector>

class ParamsMsg : public Decoder<ParamsMsg>, public Encoder {
  public:
	ParamsMsg();
	ParamsMsg(std::vector<double> &state_refs, std::vector<double> &attributes);
	ParamsMsg(ParamsMsg &&) = default;
	ParamsMsg(const ParamsMsg &) = default;
	ParamsMsg &operator=(ParamsMsg &&) = delete;
	ParamsMsg &operator=(const ParamsMsg &) = delete;
	~ParamsMsg() = default;

	std::optional<std::vector<double>> state_refs;
	std::optional<std::vector<double>> attributes;

	std::unique_ptr<ParamsMsg> deserialize(std::vector<uint8_t> &bytes) override;

  private:
	const size_t num_elements = 2;
	std::optional<std_msgs::Float64MultiArray> state_refs_arr;
	std::optional<std_msgs::Float64MultiArray> attributes_arr;
	uint32_t lengths_length = (num_elements + 1) * bytes_length;
	uint32_t data_length;
	uint32_t state_refs_length;
	uint32_t attributes_length;
	uint32_t compute_lengths_length() override;
	uint32_t compute_data_length() override;
	std::vector<uint8_t> get_lengths() override;
	std::vector<uint8_t> get_data() override;
};
