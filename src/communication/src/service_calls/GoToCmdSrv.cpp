#include "service_calls/GoToCmdSrv.hpp"
#include "std_msgs/Float32MultiArray.h"
#include <cstdint>
#include <cstring>
#include <memory>
#include <netinet/in.h>
#include <optional>
#include <vector>

using std_msgs::Float32MultiArray;

GoToCmdSrv::GoToCmdSrv() {}

GoToCmdSrv::GoToCmdSrv(Float32MultiArray &state_refs, Float32MultiArray &input_refs, Float32MultiArray &wp_attributes, Float32MultiArray &wp_normals, bool success)
	: state_refs(state_refs), input_refs(input_refs), wp_attributes(wp_attributes), wp_normals(wp_normals), success(success) {
	state_refs_length = ros::serialization::serializationLength(state_refs);
	input_refs_length = ros::serialization::serializationLength(input_refs);
	wp_attributes_length = ros::serialization::serializationLength(wp_attributes);
	wp_normals_length = ros::serialization::serializationLength(wp_normals);
	success_length = sizeof(success);
	data_length = state_refs_length + input_refs_length + wp_attributes_length + wp_normals_length + success_length;
}

GoToCmdSrv::GoToCmdSrv(float dest_x, float dest_y) : dest_x(dest_x), dest_y(dest_y) {}

std::unique_ptr<GoToCmdSrv> GoToCmdSrv::deserialize(std::vector<uint8_t> &bytes) {
	std::vector<std::vector<uint8_t>> datatypes = split(bytes);
	float dest_x = float_from_bytes(datatypes[0]);
	float dest_y = float_from_bytes(datatypes[1]);
	return std::make_unique<GoToCmdSrv>(dest_x, dest_y);
}

uint32_t GoToCmdSrv::compute_lengths_length() { return lengths_length; }

uint32_t GoToCmdSrv::compute_data_length() { return data_length; }

std::vector<uint8_t> GoToCmdSrv::get_lengths() {
	std::vector<uint8_t> lengths(lengths_length);
	std::memcpy(lengths.data(), &lengths_length, bytes_length);
	std::memcpy(lengths.data() + bytes_length, &state_refs_length, bytes_length);
	std::memcpy(lengths.data() + bytes_length * 2, &input_refs_length, bytes_length);
	std::memcpy(lengths.data() + bytes_length * 3, &wp_attributes_length, bytes_length);
	std::memcpy(lengths.data() + bytes_length * 4, &wp_normals_length, bytes_length);
	std::memcpy(lengths.data() + bytes_length * 5, &success_length, bytes_length);
	return lengths;
}

std::vector<uint8_t> GoToCmdSrv::get_data() {
	std::vector<uint8_t> data(data_length);

	std::vector<uint8_t> state_refs_data = serializeFloat32MultiArray(state_refs.value());
	std::vector<uint8_t> input_refs_data = serializeFloat32MultiArray(input_refs.value());
	std::vector<uint8_t> wp_attributes_data = serializeFloat32MultiArray(wp_attributes.value());
	std::vector<uint8_t> wp_normals_data = serializeFloat32MultiArray(wp_normals.value());

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
