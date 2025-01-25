#pragma once

#include <cstdint>
#include <vector>

class Decoder {
  public:
	Decoder(std::vector<uint8_t> &bytes);
	Decoder(Decoder &&) = default;
	Decoder(const Decoder &) = default;
	Decoder &operator=(Decoder &&) = delete;
	Decoder &operator=(const Decoder &) = delete;
	~Decoder() = default;

	// Splits bytestream into array of byte array for each datatype based on lengths
	std::vector<std::vector<uint8_t>> split();

  private:
	const size_t bytes_length = 4;
	std::vector<uint8_t> &bytes;
};
