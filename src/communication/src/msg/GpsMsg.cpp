#include "msg/GpsMsg.hpp"
#include "ros/serialization.h"
#include <cstdint>
#include <string>

GpsMsg::GpsMsg() {}

GpsMsg::GpsMsg(float x0, float y0, float yaw0, std::string &path) : x0(x0), y0(y0), yaw0(yaw0), path(path) {}

GpsMsg::GpsMsg(const geometry_msgs::PoseWithCovarianceStamped &pose) : pose(pose) {
	pose_length = ros::serialization::serializationLength(pose);
	data_length = pose_length;
}

std::unique_ptr<GpsMsg> GpsMsg::deserialize(std::vector<uint8_t> &bytes) {
	std::vector<std::vector<uint8_t>> datatypes = split(bytes);

	float x0 = float_from_bytes(datatypes[0]);
	float y0 = float_from_bytes(datatypes[1]);
	float yaw0 = float_from_bytes(datatypes[2]);
	std::string path(datatypes[3].begin(), datatypes[3].end());

	return std::make_unique<GpsMsg>(x0, y0, yaw0, path);
}

uint32_t GpsMsg::compute_lengths_length() { return lengths_length; }

uint32_t GpsMsg::compute_data_length() { return data_length; }

std::vector<uint8_t> GpsMsg::get_lengths() {
	std::vector<uint8_t> lengths(lengths_length);
	std::memcpy(lengths.data(), &lengths_length, bytes_length);
	std::memcpy(lengths.data() + bytes_length, &pose_length, bytes_length);
	return lengths;
}

std::vector<uint8_t> GpsMsg::get_data() {
	std::vector<uint8_t> data(data_length);
	std::vector<uint8_t> pose_data = serializePoseWithCovarianceStamped(pose.value());
	std::memcpy(data.data(), pose_data.data(), pose_length);
	return data;
}
