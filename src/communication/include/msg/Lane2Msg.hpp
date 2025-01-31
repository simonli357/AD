#pragma once

#include "Decoder.hpp"
#include "Encoder.hpp"
#include "std_msgs/Header.h"
#include <cstdint>
#include <vector>

class Lane2Msg : public Decoder<Lane2Msg>, public Encoder {
  public:
	Lane2Msg(std_msgs::Header &header, float center, int32_t stopline, bool crosswalk, bool dotted);
	Lane2Msg(Lane2Msg &&) = default;
	Lane2Msg(const Lane2Msg &) = default;
	Lane2Msg &operator=(Lane2Msg &&) = delete;
	Lane2Msg &operator=(const Lane2Msg &) = delete;
	~Lane2Msg() = default;

	std_msgs::Header &header;
	float center;
	int32_t stopline;
	bool crosswalk;
	bool dotted;

	std::unique_ptr<Lane2Msg> deserialize(std::vector<uint8_t> &bytes) override;

  private:
	const size_t num_elements = 5;
	uint32_t lengths_length = num_elements * (bytes_length + 1);
	uint32_t data_length;
	uint32_t header_length;
	uint32_t center_length;
	uint32_t stopline_length;
	uint32_t crosswalk_length;
	uint32_t dotted_length;
	uint32_t compute_lengths_length() override;
	uint32_t compute_data_length() override;
	std::vector<uint8_t> get_lengths() override;
	std::vector<uint8_t> get_data() override;
};
