#pragma once

#include "Decoder.hpp"
#include "Encoder.hpp"
#include "std_msgs/Float32MultiArray.h"
#include <cstdint>
#include <optional>

using std_msgs::Float32MultiArray;

class WaypointsSrv : public Decoder<WaypointsSrv>, public Encoder {
  public:
    WaypointsSrv();
	WaypointsSrv(Float32MultiArray &state_refs, Float32MultiArray &input_refs, Float32MultiArray &wp_attributes, Float32MultiArray &wp_normals);
	WaypointsSrv(std::string pathName, std::string vrefName, float x0, float y0, float yaw0);
	WaypointsSrv(WaypointsSrv &&) = default;
	WaypointsSrv(const WaypointsSrv &) = default;
	WaypointsSrv &operator=(WaypointsSrv &&) = delete;
	WaypointsSrv &operator=(const WaypointsSrv &) = delete;
	~WaypointsSrv() = default;

	// Request
	std::string pathName;
	std::string vrefName;
	float x0;
	float y0;
	float yaw0;

	// Response
	std::optional<Float32MultiArray> state_refs;
	std::optional<Float32MultiArray> input_refs;
	std::optional<Float32MultiArray> wp_attributes;
	std::optional<Float32MultiArray> wp_normals;

	std::unique_ptr<WaypointsSrv> deserialize(std::vector<uint8_t> &bytes) override;

  private:
	const size_t num_elements = 4;
	uint32_t lengths_length = (num_elements + 1) * bytes_length;
	uint32_t data_length;
	uint32_t state_refs_length;
	uint32_t input_refs_length;
	uint32_t wp_attributes_length;
	uint32_t wp_normals_length;
	uint32_t compute_lengths_length() override;
	uint32_t compute_data_length() override;
	std::vector<uint8_t> get_lengths() override;
	std::vector<uint8_t> get_data() override;
};
