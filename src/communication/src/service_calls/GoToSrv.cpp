#include "service_calls/GoToSrv.hpp"
#include "std_msgs/Float32MultiArray.h"
#include <cstdint>
#include <cstring>
#include <netinet/in.h>
#include <optional>
#include <vector>

using std_msgs::Float32MultiArray;

GoToSrv::GoToSrv() {}

GoToSrv::GoToSrv(Float32MultiArray &state_refs, Float32MultiArray &input_refs, Float32MultiArray &wp_attributes, Float32MultiArray &wp_normals)
	: state_refs(state_refs), input_refs(input_refs), wp_attributes(wp_attributes), wp_normals(wp_normals) {
	state_refs_length = ros::serialization::serializationLength(state_refs);
	input_refs_length = ros::serialization::serializationLength(input_refs);
	wp_attributes_length = ros::serialization::serializationLength(wp_attributes);
	wp_normals_length = ros::serialization::serializationLength(wp_normals);
	data_length = state_refs_length + input_refs_length + wp_attributes_length + wp_normals_length;
}

GoToSrv::GoToSrv(std::string vrefName, float x0, float y0, float yaw0, float dest_x, float dest_y)
	: vrefName(vrefName), x0(x0), y0(y0), yaw0(yaw0), dest_x(dest_x), dest_y(dest_y) {}

std::unique_ptr<GoToSrv> GoToSrv::deserialize(std::vector<uint8_t> &bytes) {
	std::vector<std::vector<uint8_t>> datatypes = split(bytes);
	std::string vrefName(datatypes[0].begin(), datatypes[0].end());
	float x0 = float_from_bytes(datatypes[1]);
	float y0 = float_from_bytes(datatypes[2]);
	float yaw0 = float_from_bytes(datatypes[3]);
	float dest_x = float_from_bytes(datatypes[4]);
	float dest_y = float_from_bytes(datatypes[5]);
	return std::make_unique<GoToSrv>(vrefName, x0, y0, yaw0, dest_x, dest_y);
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

	return data;
}
