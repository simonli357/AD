#include "service_calls/GoToCmdSrv.hpp"
#include "ros/serialization.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float64MultiArray.h"
#include <cstdint>
#include <cstring>
#include <netinet/in.h>
#include <vector>

using std_msgs::Float32MultiArray;

void GoToCmdSrv::encode(const Float32MultiArray &state_refs, const Float32MultiArray &input_refs, const Float32MultiArray &wp_attributes, const Float32MultiArray &wp_normals, bool success) {
    this->state_refs = state_refs;
    this->input_refs = input_refs;
    this->wp_attributes = wp_attributes;
    this->wp_normals = wp_normals;
    this->success = success;
	state_refs_length = ros::serialization::serializationLength(state_refs);
	input_refs_length = ros::serialization::serializationLength(input_refs);
	wp_attributes_length = ros::serialization::serializationLength(wp_attributes);
	wp_normals_length = ros::serialization::serializationLength(wp_normals);
	success_length = sizeof(success);
	data_length = state_refs_length + input_refs_length + wp_attributes_length + wp_normals_length + success_length;
}

void GoToCmdSrv::deserialize(std::vector<uint8_t> &bytes) {
	std::vector<std::vector<uint8_t>> datatypes = split(bytes);
	std_msgs::Float64MultiArray x_coords;
	std_msgs::Float64MultiArray y_coords;
	ros::serialization::IStream x_stream(datatypes[0].data(), datatypes[0].size());
	ros::serialization::IStream y_stream(datatypes[1].data(), datatypes[1].size());
	ros::serialization::deserialize(x_stream, x_coords);
	ros::serialization::deserialize(y_stream, y_coords);
    coords.clear();
	for (size_t i = 0; i < x_coords.data.size(); ++i) {
		coords.emplace_back(static_cast<float>(x_coords.data[i]), static_cast<float>(y_coords.data[i]));
	}
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
