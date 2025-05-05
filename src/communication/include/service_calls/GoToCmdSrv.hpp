#pragma once

#include "Decoder.hpp"
#include "Encoder.hpp"
#include "std_msgs/Float32MultiArray.h"
#include <cstdint>
#include <tuple>

using std_msgs::Float32MultiArray;

class GoToCmdSrv : public Decoder<GoToCmdSrv>, public Encoder {
  public:
	GoToCmdSrv() = default;
	GoToCmdSrv(GoToCmdSrv &&) = default;
	GoToCmdSrv(const GoToCmdSrv &) = default;
	GoToCmdSrv &operator=(GoToCmdSrv &&) = delete;
	GoToCmdSrv &operator=(const GoToCmdSrv &) = delete;
	~GoToCmdSrv() = default;

	// Request
	std::vector<std::tuple<float, float>> coords;

	// Response
	Float32MultiArray state_refs;
	Float32MultiArray input_refs;
	Float32MultiArray wp_attributes;
	Float32MultiArray wp_normals;
	bool success;

	void deserialize(std::vector<uint8_t> &bytes) override;
	void encode(const Float32MultiArray &state_refs, const Float32MultiArray &input_refs, const Float32MultiArray &wp_attributes, const Float32MultiArray &wp_normals, bool success);

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
