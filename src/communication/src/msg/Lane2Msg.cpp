#include "msg/Lane2Msg.hpp"
#include "ros/serialization.h"
#include "std_msgs/Header.h"
#include <cstdint>

void Lane2Msg::encode(const std_msgs::Header &header, float center, bool stopline, float stopline_dist, bool crosswalk, bool dotted) {
    this->header = header;
    this->center = center;
    this->stopline = stopline;
    this->stopline_dist = stopline_dist;
    this->crosswalk = crosswalk;
    this->dotted = dotted;
	header_length = ros::serialization::serializationLength(header);
	center_length = sizeof(center);
	stopline_length = sizeof(stopline);
    stopline_dist_length = sizeof(stopline_dist);
	crosswalk_length = sizeof(crosswalk);
	dotted_length = sizeof(dotted);
	data_length = header_length + center_length + stopline_length + stopline_dist_length + crosswalk_length + dotted_length;
}

uint32_t Lane2Msg::compute_lengths_length() { return lengths_length; }

uint32_t Lane2Msg::compute_data_length() { return data_length; }

std::vector<uint8_t> Lane2Msg::get_lengths() {
	std::vector<uint8_t> lengths(lengths_length);
	std::memcpy(lengths.data(), &lengths_length, bytes_length);
	std::memcpy(lengths.data() + bytes_length, &header_length, bytes_length);
	std::memcpy(lengths.data() + bytes_length * 2, &center_length, bytes_length);
	std::memcpy(lengths.data() + bytes_length * 3, &stopline_length, bytes_length);
    std::memcpy(lengths.data() + bytes_length * 4, &stopline_dist_length, bytes_length);
	std::memcpy(lengths.data() + bytes_length * 5, &crosswalk_length, bytes_length);
	std::memcpy(lengths.data() + bytes_length * 6, &dotted_length, bytes_length);
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

    std::memcpy(data.data() + offset, &stopline_dist, stopline_dist_length);
    offset += stopline_dist_length;

	std::memcpy(data.data() + offset, &crosswalk, crosswalk_length);
	offset += crosswalk_length;

	std::memcpy(data.data() + offset, &dotted, dotted_length);

	return data;
}
