#include "Encoder.hpp"
#include "ros/serialization.h"
#include "std_msgs/Header.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_srvs/TriggerRequest.h"
#include "std_srvs/TriggerResponse.h"
#include <cstdint>
#include <cstring>
#include <netinet/in.h>
#include <vector>

std::vector<uint8_t> Encoder::serializeFloat32MultiArray(std_msgs::Float32MultiArray &array) {
	uint32_t length = ros::serialization::serializationLength(array);
	std::vector<uint8_t> data(length);
	ros::serialization::OStream stream(data.data(), length);
	ros::serialization::serialize(stream, array);
	return data;
}

std::vector<uint8_t> Encoder::serializeROSHeader(std_msgs::Header &header) {
	uint32_t length = ros::serialization::serializationLength(header);
	std::vector<uint8_t> data(length);
	ros::serialization::OStream stream(data.data(), length);
	ros::serialization::serialize(stream, header);
	return data;
}

std::vector<uint8_t> Encoder::serializePoseWithCovarianceStamped(geometry_msgs::PoseWithCovarianceStamped &pose) {
	uint32_t length = ros::serialization::serializationLength(pose);
	std::vector<uint8_t> data(length);
	ros::serialization::OStream stream(data.data(), length);
	ros::serialization::serialize(stream, pose);
	return data;
}

std::vector<uint8_t> Encoder::serializeTriggerRequest(std_srvs::TriggerRequest &request) {
	uint32_t length = ros::serialization::serializationLength(request);
	std::vector<uint8_t> data(length);
	ros::serialization::OStream stream(data.data(), length);
	ros::serialization::serialize(stream, request);
	return data;
}

std::vector<uint8_t> Encoder::serializeTriggerResponse(std_srvs::TriggerResponse &response) {
	uint32_t length = ros::serialization::serializationLength(response);
	std::vector<uint8_t> data(length);
	ros::serialization::OStream stream(data.data(), length);
	ros::serialization::serialize(stream, response);
	return data;
}

std::vector<uint8_t> Encoder::serialize(uint8_t data_type) {
	uint32_t lengths_length = compute_lengths_length();
	uint32_t data_length = compute_data_length();
	uint32_t size = lengths_length + data_length;
	std::vector<uint8_t> lengths = get_lengths();
	std::vector<uint8_t> data = get_data();

	// Compute Total Size
	size_t total_size = header_size + size;
	std::vector<uint8_t> full_message(total_size);

	// Header
	std::memcpy(full_message.data(), &size, message_size);

	// Data Type
	full_message[4] = data_type;

	// Sub Message Lengths
	std::memcpy(full_message.data() + header_size, lengths.data(), lengths_length);

	// Data
	std::memcpy(full_message.data() + header_size + lengths_length, data.data(), data_length);

	return full_message;
}

std_msgs::Float32MultiArray Encoder::double_vector_to_arr(const std::vector<double> &vec) {
    std::vector<float> float_vec;
    float_vec.reserve(vec.size());
    for (const auto& d : vec) {
        float_vec.push_back(static_cast<float>(d));
    }
	std_msgs::Float32MultiArray msg;
	msg.data = float_vec;
	return msg;
}

uint32_t Encoder::compute_lengths_length() { return 0; }

uint32_t Encoder::compute_data_length() { return 0; }

std::vector<uint8_t> Encoder::get_lengths() { return {}; }

std::vector<uint8_t> Encoder::get_data() { return {}; }
