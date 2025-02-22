#include "msg/LanesMsg.hpp"
#include "ros/serialization.h"
#include "std_msgs/Float64MultiArray.h"
#include <cstdint>
#include <cstring>
#include <netinet/in.h>

LanesMsg::LanesMsg() {}

LanesMsg::LanesMsg(std::vector<std::tuple<float, float>> &lane1, std::vector<std::tuple<float, float>> &lane2) {
	for (const auto &point : lane1) {
		lane1x.data.push_back(std::get<0>(point));
		lane1y.data.push_back(std::get<1>(point));
	}
	for (const auto &point : lane2) {
		lane2x.data.push_back(std::get<0>(point));
		lane2y.data.push_back(std::get<1>(point));
	}
	lane1x_length = ros::serialization::serializationLength(lane1x);
	lane1y_length = ros::serialization::serializationLength(lane1y);
	lane2x_length = ros::serialization::serializationLength(lane2x);
	lane2y_length = ros::serialization::serializationLength(lane2y);
	data_length = lane1x_length + lane1y_length + lane2x_length + lane2y_length;
}

uint32_t LanesMsg::compute_lengths_length() { return lengths_length; }

uint32_t LanesMsg::compute_data_length() { return data_length; }

std::vector<uint8_t> LanesMsg::get_lengths() {
	std::vector<uint8_t> lengths(lengths_length);
	std::memcpy(lengths.data(), &lengths_length, bytes_length);
	std::memcpy(lengths.data() + bytes_length, &lane1x_length, bytes_length);
	std::memcpy(lengths.data() + bytes_length * 2, &lane1y_length, bytes_length);
	std::memcpy(lengths.data() + bytes_length * 3, &lane2x_length, bytes_length);
	std::memcpy(lengths.data() + bytes_length * 4, &lane2y_length, bytes_length);
	return lengths;
}

std::vector<uint8_t> LanesMsg::get_data() {
	std::vector<uint8_t> data(data_length);

	std::vector<uint8_t> lane1x_data = serializeFloat64MultiArray(lane1x);
	std::vector<uint8_t> lane1y_data = serializeFloat64MultiArray(lane1y);
	std::vector<uint8_t> lane2x_data = serializeFloat64MultiArray(lane2x);
	std::vector<uint8_t> lane2y_data = serializeFloat64MultiArray(lane2y);

	size_t offset = 0;
	std::memcpy(data.data(), lane1x_data.data(), lane1x_length);
	offset += lane1x_length;

	std::memcpy(data.data() + offset, lane1y_data.data(), lane1y_length);
	offset += lane1y_length;

	std::memcpy(data.data() + offset, lane2x_data.data(), lane2x_length);
	offset += lane2x_length;

	std::memcpy(data.data() + offset, lane2y_data.data(), lane2y_length);

	return data;
}
