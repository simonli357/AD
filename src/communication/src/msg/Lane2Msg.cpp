#include "msg/Lane2Msg.hpp"
#include "ros/serialization.h"
#include "std_msgs/Header.h"
#include <cstdint>

Lane2Msg::Lane2Msg(std_msgs::Header &header, float center, int32_t stopline, bool crosswalk, bool dotted)
	: header(header), center(center), stopline(stopline), crosswalk(crosswalk), dotted(dotted) {
	header_length = ros::serialization::serializationLength(header);
	center_length = sizeof(center);
	stopline_length = sizeof(stopline);
	crosswalk_length = sizeof(crosswalk);
	dotted_length = sizeof(dotted);
	data_length = header_length + center_length + stopline_length + crosswalk_length + dotted_length;
}

std::unique_ptr<Lane2Msg> Lane2Msg::deserialize(std::vector<uint8_t> &bytes) {
    std::vector<std::vector<uint8_t>> datatypes = split(bytes);
    ros::serialization::IStream stream(datatypes[0].data(), datatypes[0].size());
    
    std_msgs::Header header_msg;
    ros::serialization::deserialize(stream, header_msg);

    float center = bool_from_bytes(datatypes[1]);
    int32_t stopline = int32_t_from_bytes(datatypes[2]);
    bool crosswalk = bool_from_bytes(datatypes[3]);
    bool dotted = bool_from_bytes(datatypes[4]);
    
    return std::make_unique<Lane2Msg>(header_msg, center, stopline, crosswalk, dotted);
}

uint32_t Lane2Msg::compute_lengths_length() { return lengths_length; }

uint32_t Lane2Msg::compute_data_length() { return data_length; }

std::vector<uint8_t> Lane2Msg::get_lengths() {
	std::vector<uint8_t> lengths(lengths_length);
	std::memcpy(lengths.data(), &lengths_length, bytes_length);
	std::memcpy(lengths.data() + bytes_length, &header_length, bytes_length);
	std::memcpy(lengths.data() + bytes_length * 2, &center_length, bytes_length);
	std::memcpy(lengths.data() + bytes_length * 3, &stopline_length, bytes_length);
	std::memcpy(lengths.data() + bytes_length * 4, &crosswalk_length, bytes_length);
	std::memcpy(lengths.data() + bytes_length * 5, &dotted_length, bytes_length);
	return lengths;
}

std::vector<uint8_t> Lane2Msg::get_data() {
	std::vector<uint8_t> data(data_length);

	std::vector<uint8_t> header_data = serializeROSHeader(header);

	size_t offset = 0;
	std::memcpy(data.data(), header_data.data(), header_length);
	offset += header_length;

	std::memcpy(data.data() + offset, &center, center_length);
	offset += center_length;

	std::memcpy(data.data() + offset, &stopline, stopline_length);
	offset += stopline_length;

	std::memcpy(data.data() + offset, &crosswalk, crosswalk_length);
	offset += crosswalk_length;

	std::memcpy(data.data() + offset, &dotted, dotted_length);

	return data;
}
