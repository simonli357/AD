#include "Decoder.hpp"
#include <cstdint>
#include <cstring>
#include <netinet/in.h>
#include <vector>

Decoder::Decoder(std::vector<uint8_t> &bytes) : bytes(bytes) {}

std::vector<std::vector<uint8_t>> Decoder::split() {
	uint32_t lengths_length = 0;
	std::memcpy(&lengths_length, &bytes[0], bytes_length);

	size_t num_elements = (lengths_length - bytes_length) / bytes_length;
	std::vector<std::vector<uint8_t>> splits(num_elements);
	size_t num_bytes = bytes.size();

	size_t size_offset = bytes_length;
	size_t data_offset = lengths_length;
	for (size_t i = 0; i < num_elements; i++) {
		// Get the size of the element
		uint32_t size = 0;
		std::memcpy(&size, &bytes[size_offset], bytes_length);
		size_offset += bytes_length;
		// Read the data and append
		std::vector<uint8_t> split(size);
		std::memcpy(split.data(), &bytes[data_offset], size);
		splits[i] = std::move(split);
		data_offset += size;
	}

	return splits;
}
