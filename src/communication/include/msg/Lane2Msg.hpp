#pragma once

#include "Encoder.hpp"
#include "std_msgs/Header.h"
#include <cstddef>
#include <cstdint>
#include <vector>

class Lane2Msg : public Encoder {
  public:
	Lane2Msg() = default;
	Lane2Msg(Lane2Msg &&) = default;
	Lane2Msg(const Lane2Msg &) = default;
	Lane2Msg &operator=(Lane2Msg &&) = delete;
	Lane2Msg &operator=(const Lane2Msg &) = delete;
	~Lane2Msg() = default;

	std_msgs::Header header;
	float center;
	bool stopline;
	float stopline_dist;
	bool crosswalk;
	bool dotted;

	void encode(const std_msgs::Header &header, float center, bool stopline, float stopline_dist, bool crosswalk, bool dotted);

  private:
	const size_t bytes_length = 4;
	const size_t num_elements = 6;
	uint32_t lengths_length = (num_elements + 1) * bytes_length;
	uint32_t data_length;
	uint32_t header_length;
	uint32_t center_length;
	uint32_t stopline_length;
	uint32_t stopline_dist_length;
	uint32_t crosswalk_length;
	uint32_t dotted_length;
	uint32_t compute_lengths_length() override;
	uint32_t compute_data_length() override;
	std::vector<uint8_t> get_lengths() override;
	std::vector<uint8_t> get_data() override;
};
