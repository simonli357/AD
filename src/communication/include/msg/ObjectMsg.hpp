#pragma once

#include "Encoder.hpp"
#include <cstdint>

class ObjectMsg : public Encoder {
  public:
	ObjectMsg();
	ObjectMsg(std::string &type, float x, float y);
	ObjectMsg(ObjectMsg &&) = default;
	ObjectMsg(const ObjectMsg &) = default;
	ObjectMsg &operator=(ObjectMsg &&) = delete;
	ObjectMsg &operator=(const ObjectMsg &) = delete;
	~ObjectMsg() = default;

	std::string type;
	float x;
	float y;

  private:
	const size_t num_elements = 3;
	const size_t bytes_length = 4;
	uint32_t lengths_length = (num_elements + 1) * bytes_length;
	uint32_t data_length;
	uint32_t type_length;
	uint32_t x_length;
	uint32_t y_length;
	uint32_t compute_lengths_length() override;
	uint32_t compute_data_length() override;
	std::vector<uint8_t> get_lengths() override;
	std::vector<uint8_t> get_data() override;
};
