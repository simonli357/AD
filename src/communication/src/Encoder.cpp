#include "Encoder.hpp"
#include "ros/serialization.h"
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

std::vector<uint8_t> Encoder::serialize() {
	uint32_t lengths_length = compute_lengths_length();
	uint32_t data_length = compute_data_length();
	uint32_t size = lengths_length + data_length;
	uint32_t big_endian_size = htonl(size);
	std::vector<uint8_t> lengths = get_lengths();
	std::vector<uint8_t> data = get_data();

	// Compute Total Size
	size_t total_size = header_size + size;
	std::vector<uint8_t> full_message(total_size);

	// Header
	std::memcpy(full_message.data(), &big_endian_size, message_size);

	// Data Type
	full_message[4] = data_type;

	// Sub Message Lengths
	std::memcpy(full_message.data() + header_size, lengths.data(), lengths_length);

	// Data
	std::memcpy(full_message.data() + header_size + lengths_length, data.data(), data_length);

	return full_message;
}
