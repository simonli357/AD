#pragma once

#include "Decoder.hpp"
#include "Encoder.hpp"
#include <cstdint>

class SetStatesSrv : public Decoder<SetStatesSrv>, public Encoder {
  public:
	SetStatesSrv();
	SetStatesSrv(bool success);
	SetStatesSrv(float x, float y);
	SetStatesSrv(SetStatesSrv &&) = default;
	SetStatesSrv(const SetStatesSrv &) = default;
	SetStatesSrv &operator=(SetStatesSrv &&) = delete;
	SetStatesSrv &operator=(const SetStatesSrv &) = delete;
	~SetStatesSrv() = default;

	// Request
	float x;
	float y;

	// Response
	bool success;

	std::unique_ptr<SetStatesSrv> deserialize(std::vector<uint8_t> &bytes) override;

  private:
	const size_t num_elements = 1;
	uint32_t lengths_length = (num_elements + 1) * bytes_length;
	uint32_t data_length;
	uint32_t success_length;
	uint32_t compute_lengths_length() override;
	uint32_t compute_data_length() override;
	std::vector<uint8_t> get_lengths() override;
	std::vector<uint8_t> get_data() override;
};
