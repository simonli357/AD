#pragma once

#include "Encoder.hpp"
#include <cstdint>

class RunMsg : public Encoder {
  public:
	RunMsg() = default;
	RunMsg(RunMsg &&) = default;
	RunMsg(const RunMsg &) = default;
	RunMsg &operator=(RunMsg &&) = delete;
	RunMsg &operator=(const RunMsg &) = delete;
	~RunMsg() = default;

	float v_ref;
	std::string path_name;
	float x_init;
	float y_init;
	float yaw_init;

	void encode(float v_ref, const std::string &path_name, float x_init, float y_init, float yaw_init);

  private:
	const size_t num_elements = 5;
	const size_t bytes_length = 4;
	uint32_t lengths_length = (num_elements + 1) * bytes_length;
	uint32_t data_length;
	uint32_t v_ref_length;
	uint32_t path_name_length;
	uint32_t x_init_length;
	uint32_t y_init_length;
	uint32_t yaw_init_length;
	uint32_t compute_lengths_length() override;
	uint32_t compute_data_length() override;
	std::vector<uint8_t> get_lengths() override;
	std::vector<uint8_t> get_data() override;
};
