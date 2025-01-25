#include "service_calls/GoToCmdSrvResponse.hpp"
#include "std_msgs/Float32MultiArray.h"
#include <cstdint>
#include <cstring>
#include <netinet/in.h>
#include <vector>

using std_msgs::Float32MultiArray;

GoToCmdSrvResponse::GoToCmdSrvResponse(uint8_t &data_type, Float32MultiArray &state_refs, Float32MultiArray &input_refs, Float32MultiArray &wp_attributes,
									   Float32MultiArray &wp_normals, bool success)
	: Encoder(data_type), state_refs(state_refs), input_refs(input_refs), wp_attributes(wp_attributes), wp_normals(wp_normals), success(success) {}

uint32_t GoToCmdSrvResponse::compute_lengths_length() { return lengths_length; }

uint32_t GoToCmdSrvResponse::compute_data_length() { return data_length; }

std::vector<uint8_t> GoToCmdSrvResponse::get_lengths() {
	std::vector<uint8_t> lengths(lengths_length);

	std::memcpy(lengths.data(), &lengths_length, length_bytes);
	std::memcpy(lengths.data() + length_bytes, &state_refs_length, length_bytes);
	std::memcpy(lengths.data() + length_bytes * 2, &input_refs_length, length_bytes);
	std::memcpy(lengths.data() + length_bytes * 3, &wp_attributes_length, length_bytes);
	std::memcpy(lengths.data() + length_bytes * 4, &wp_normals_length, length_bytes);
	std::memcpy(lengths.data() + length_bytes * 5, &success_length, length_bytes);

	return lengths;
}

std::vector<uint8_t> GoToCmdSrvResponse::get_data() {
	std::vector<uint8_t> data(data_length);

	std::vector<uint8_t> state_refs_data = serializeFloat32MultiArray(state_refs);
	std::vector<uint8_t> input_refs_data = serializeFloat32MultiArray(input_refs);
	std::vector<uint8_t> wp_attributes_data = serializeFloat32MultiArray(wp_attributes);
	std::vector<uint8_t> wp_normals_data = serializeFloat32MultiArray(wp_normals);

	size_t offset = 0;
	std::memcpy(data.data(), state_refs_data.data(), state_refs_length);
	offset += state_refs_length;

	std::memcpy(data.data() + offset, input_refs_data.data(), input_refs_length);
	offset += input_refs_length;

	std::memcpy(data.data() + offset, wp_attributes_data.data(), wp_attributes_length);
	offset += wp_attributes_length;

	std::memcpy(data.data() + offset, wp_normals_data.data(), wp_normals_length);
	offset += wp_normals_length;

	data[offset + 1] = static_cast<uint8_t>(success);

	return data;
}
