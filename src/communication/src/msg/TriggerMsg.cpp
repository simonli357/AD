#include "msg/TriggerMsg.hpp"
#include "ros/serialization.h"
#include "std_srvs/Trigger.h"
#include "std_srvs/TriggerRequest.h"
#include <cstdint>

void TriggerMsg::encode(const std_srvs::Trigger &trigger) {
    request = trigger.request;
    response = trigger.response;
	request_length = ros::serialization::serializationLength(request);
    response_length = ros::serialization::serializationLength(response);
	data_length = request_length + response_length;
}

void TriggerMsg::deserialize(std::vector<uint8_t> &bytes) {
    std::vector<std::vector<uint8_t>> datatypes = split(bytes);
    if (datatypes.size() == 1) {
        ros::serialization::IStream response_stream(datatypes[0].data(), datatypes[0].size());
        std_srvs::TriggerResponse response;
        ros::serialization::deserialize(response_stream, response);
        this->response = response;
        return;
    }
    ros::serialization::IStream request_stream(datatypes[0].data(), datatypes[0].size());
    ros::serialization::IStream response_stream(datatypes[1].data(), datatypes[1].size());
    
    std_srvs::TriggerRequest request;
    std_srvs::TriggerResponse response;
    ros::serialization::deserialize(request_stream, request);
    ros::serialization::deserialize(response_stream, response);

    this->request = request;
    this->response = response;
}

uint32_t TriggerMsg::compute_lengths_length() { return lengths_length; }

uint32_t TriggerMsg::compute_data_length() { return data_length; }

std::vector<uint8_t> TriggerMsg::get_lengths() {
	std::vector<uint8_t> lengths(lengths_length);
	std::memcpy(lengths.data(), &lengths_length, bytes_length);
	std::memcpy(lengths.data() + bytes_length, &request_length, bytes_length);
	std::memcpy(lengths.data() + bytes_length * 2, &response_length, bytes_length);
	return lengths;
}

std::vector<uint8_t> TriggerMsg::get_data() {
	std::vector<uint8_t> data(data_length);

	std::vector<uint8_t> request_data = serializeTriggerRequest(request);
	std::vector<uint8_t> response_data = serializeTriggerResponse(response);

	size_t offset = 0;
	std::memcpy(data.data(), request_data.data(), request_length);
	offset += request_length;

	std::memcpy(data.data() + offset, response_data.data(), response_length);

	return data;
}
