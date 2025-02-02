#pragma once

#include "Decoder.hpp"
#include "Encoder.hpp"
#include "std_msgs/Float32MultiArray.h"
#include <cstdint>
#include <optional>

using std_msgs::Float32MultiArray;

class GoToCmdSrv : public Decoder<GoToCmdSrv>, public Encoder {
  public:
    GoToCmdSrv();
	GoToCmdSrv(Float32MultiArray &state_refs, Float32MultiArray &input_refs, Float32MultiArray &wp_attributes, Float32MultiArray &wp_normals, bool success);
	GoToCmdSrv(float dest_x, float dest_y);
	GoToCmdSrv(GoToCmdSrv &&) = default;
	GoToCmdSrv(const GoToCmdSrv &) = default;
	GoToCmdSrv &operator=(GoToCmdSrv &&) = delete;
	GoToCmdSrv &operator=(const GoToCmdSrv &) = delete;
	~GoToCmdSrv() = default;

	// Request
	float dest_x;
	float dest_y;

	// Response
	std::optional<Float32MultiArray> state_refs;
	std::optional<Float32MultiArray> input_refs;
	std::optional<Float32MultiArray> wp_attributes;
	std::optional<Float32MultiArray> wp_normals;
	bool success;

	std::unique_ptr<GoToCmdSrv> deserialize(std::vector<uint8_t> &bytes) override;

  private:
	const size_t num_elements = 5;
	uint32_t lengths_length = (num_elements + 1) * bytes_length;
	uint32_t data_length;
	uint32_t state_refs_length;
	uint32_t input_refs_length;
	uint32_t wp_attributes_length;
	uint32_t wp_normals_length;
	uint32_t success_length;
	uint32_t compute_lengths_length() override;
	uint32_t compute_data_length() override;
	std::vector<uint8_t> get_lengths() override;
	std::vector<uint8_t> get_data() override;
};
