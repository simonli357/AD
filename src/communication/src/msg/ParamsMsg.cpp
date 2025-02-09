#include "msg/ParamsMsg.hpp"
#include "ros/serialization.h"
#include <cstdint>

ParamsMsg::ParamsMsg() {}

ParamsMsg::ParamsMsg(std::vector<double> &state_refs, std::vector<double> &attributes)
	: state_refs(state_refs), attributes(attributes) {
    state_refs_arr = double_vector_to_arr(state_refs);
    attributes_arr = double_vector_to_arr(attributes);
	state_refs_length = ros::serialization::serializationLength(state_refs_arr.value());
	attributes_length = ros::serialization::serializationLength(attributes_arr.value());
	data_length = state_refs_length + attributes_length;
}

std::unique_ptr<ParamsMsg> ParamsMsg::deserialize(std::vector<uint8_t> &bytes) {
    return std::make_unique<ParamsMsg>();
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

	std::vector<uint8_t> state_refs_data = serializeFloat64MultiArray(state_refs_arr.value());
	std::vector<uint8_t> attributes_data = serializeFloat64MultiArray(attributes_arr.value());

	size_t offset = 0;
	std::memcpy(data.data(), state_refs_data.data(), state_refs_length);
	offset += state_refs_length;

	std::memcpy(data.data() + offset, attributes_data.data(), attributes_length);

	return data;
}
