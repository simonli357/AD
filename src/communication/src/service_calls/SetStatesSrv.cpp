#include "service_calls/SetStatesSrv.hpp"
#include <cstdint>
#include <cstring>
#include <netinet/in.h>
#include <vector>

void SetStatesSrv::encode(bool success) {
    this->success = success;
	success_length = sizeof(success);
	data_length = success_length;
}

void SetStatesSrv::deserialize(std::vector<uint8_t> &bytes) {
	std::vector<std::vector<uint8_t>> datatypes = split(bytes);
	x = float_from_bytes(datatypes[0]);
	y = float_from_bytes(datatypes[1]);
}

uint32_t SetStatesSrv::compute_lengths_length() { return lengths_length; }

uint32_t SetStatesSrv::compute_data_length() { return data_length; }

std::vector<uint8_t> SetStatesSrv::get_lengths() {
	std::vector<uint8_t> lengths(lengths_length);
	std::memcpy(lengths.data(), &lengths_length, bytes_length);
	std::memcpy(lengths.data() + bytes_length, &success_length, bytes_length);
	return lengths;
}

std::vector<uint8_t> SetStatesSrv::get_data() {
	std::vector<uint8_t> data(data_length);
	std::memcpy(data.data(), &success, success_length);
	return data;
}
