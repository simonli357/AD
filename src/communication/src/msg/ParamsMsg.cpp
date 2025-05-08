#include "msg/ParamsMsg.hpp"
#include "ros/serialization.h"
#include <cstdint>

void ParamsMsg::encode(const std::vector<double> &state_refs, const std::vector<double> &attributes) {
    this->state_refs = state_refs;
    this->attributes = attributes;
    state_refs_arr = double_vector_to_arr(state_refs);
    attributes_arr = double_vector_to_arr(attributes);
	state_refs_length = ros::serialization::serializationLength(state_refs_arr);
	attributes_length = ros::serialization::serializationLength(attributes_arr);
	data_length = state_refs_length + attributes_length;
}

uint32_t ParamsMsg::compute_lengths_length() { return lengths_length; }

uint32_t ParamsMsg::compute_data_length() { return data_length; }

std::vector<uint8_t> ParamsMsg::get_lengths() {
	std::vector<uint8_t> lengths(lengths_length);
	std::memcpy(lengths.data(), &lengths_length, bytes_length);
	std::memcpy(lengths.data() + bytes_length, &state_refs_length, bytes_length);
	std::memcpy(lengths.data() + bytes_length * 2, &attributes_length, bytes_length);
	return lengths;
}

std::vector<uint8_t> ParamsMsg::get_data() {
	std::vector<uint8_t> data(data_length);

	std::vector<uint8_t> state_refs_data = serializeFloat32MultiArray(state_refs_arr);
	std::vector<uint8_t> attributes_data = serializeFloat32MultiArray(attributes_arr);

	size_t offset = 0;
	std::memcpy(data.data(), state_refs_data.data(), state_refs_length);
	offset += state_refs_length;

	std::memcpy(data.data() + offset, attributes_data.data(), attributes_length);

	return data;
}
