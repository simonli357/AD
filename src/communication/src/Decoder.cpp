#include "Decoder.hpp"
#include <cstdint>
#include <cstring>
#include <netinet/in.h>
#include <vector>

std::vector<std::vector<uint8_t>> Decoder::split() {
	uint32_t lengths_length = 0;
	std::memcpy(&lengths_length, &bytes[0], bytes_length);
	lengths_length = ntohl(lengths_length);

	size_t num_elements = (lengths_length - bytes_length) / bytes_length;
	std::vector<std::vector<uint8_t>> splits(num_elements);

	size_t offset = 0;
	for (size_t i = 0; i < num_elements; i++) {
		// Get the size of the element
		size_t s = (i + 1) * bytes_length;
		uint32_t size = 0;
		std::memcpy(&size, &bytes[s], bytes_length);
		size = ntohl(size);
		// Read the data and append
		size_t start = lengths_length + offset;
		std::vector<uint8_t> split(size);
		std::memcpy(split.data(), &bytes[start], size);
		splits[i] = std::move(split);
		offset += size;
	}

	return splits;
}
