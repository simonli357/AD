#pragma once

#include "Decoder.hpp"
#include "Encoder.hpp"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <cstdint>
#include <optional>
#include <vector>

class GpsMsg : public Decoder<GpsMsg>, public Encoder {
  public:
    GpsMsg();
	GpsMsg(const geometry_msgs::PoseWithCovarianceStamped &pose);
	GpsMsg(float x0, float y0, float yaw0, std::string &path);
	GpsMsg(GpsMsg &&) = default;
	GpsMsg(const GpsMsg &) = default;
	GpsMsg &operator=(GpsMsg &&) = delete;
	GpsMsg &operator=(const GpsMsg &) = delete;
	~GpsMsg() = default;

	// Request
	std::optional<geometry_msgs::PoseWithCovarianceStamped> pose;

	// Response
	float x0;
	float y0;
	float yaw0;
	std::string path;

	std::unique_ptr<GpsMsg> deserialize(std::vector<uint8_t> &bytes) override;

  private:
	const size_t num_elements = 1;
	uint32_t lengths_length = (num_elements + 1) * bytes_length;
	uint32_t data_length;
	uint32_t pose_length;
	uint32_t compute_lengths_length() override;
	uint32_t compute_data_length() override;
	std::vector<uint8_t> get_lengths() override;
	std::vector<uint8_t> get_data() override;
};
