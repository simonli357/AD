#include "service_calls/GoToSrv.hpp"
#include "std_msgs/Float32MultiArray.h"
#include <cstdint>
#include <cstring>
#include <netinet/in.h>
#include <vector>

using std_msgs::Float32MultiArray;

void GoToSrv::encode(const Float32MultiArray &state_refs, const Float32MultiArray &input_refs, const Float32MultiArray &wp_attributes, const Float32MultiArray &wp_normals) {
    this->state_refs = state_refs;
    this->input_refs = input_refs;
    this->wp_attributes = wp_attributes;
    this->wp_normals = wp_normals;
	state_refs_length = ros::serialization::serializationLength(state_refs);
	input_refs_length = ros::serialization::serializationLength(input_refs);
	wp_attributes_length = ros::serialization::serializationLength(wp_attributes);
	wp_normals_length = ros::serialization::serializationLength(wp_normals);
	data_length = state_refs_length + input_refs_length + wp_attributes_length + wp_normals_length;
}

void GoToSrv::deserialize(std::vector<uint8_t> &bytes) {
	std::vector<std::vector<uint8_t>> datatypes = split(bytes);
	std::string vrefName(datatypes[0].begin(), datatypes[0].end());
    this->vrefName = vrefName;
	x0 = float_from_bytes(datatypes[1]);
	y0 = float_from_bytes(datatypes[2]);
	yaw0 = float_from_bytes(datatypes[3]);
	dest_x = float_from_bytes(datatypes[4]);
	dest_y = float_from_bytes(datatypes[5]);
}

uint32_t GoToSrv::compute_lengths_length() { return lengths_length; }

uint32_t GoToSrv::compute_data_length() { return data_length; }

std::vector<uint8_t> GoToSrv::get_lengths() {
	std::vector<uint8_t> lengths(lengths_length);
	std::memcpy(lengths.data(), &lengths_length, bytes_length);
	std::memcpy(lengths.data() + bytes_length, &state_refs_length, bytes_length);
	std::memcpy(lengths.data() + bytes_length * 2, &input_refs_length, bytes_length);
	std::memcpy(lengths.data() + bytes_length * 3, &wp_attributes_length, bytes_length);
	std::memcpy(lengths.data() + bytes_length * 4, &wp_normals_length, bytes_length);
	return lengths;
}

std::vector<uint8_t> GoToSrv::get_data() {
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

	return data;
}
