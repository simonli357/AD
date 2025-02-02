#pragma once

#include "std_msgs/Float32MultiArray.h"
#include "std_srvs/TriggerRequest.h"
#include "std_srvs/TriggerResponse.h"
#include <cstdint>
#include <vector>

class Encoder {
  public:
	Encoder() = default;
	Encoder(Encoder &&) = default;
	Encoder(const Encoder &) = default;
	Encoder &operator=(Encoder &&) = delete;
	Encoder &operator=(const Encoder &) = delete;
	virtual ~Encoder() = default;

	std::vector<uint8_t> serialize(uint8_t data_type);
	std::vector<uint8_t> serializeFloat32MultiArray(std_msgs::Float32MultiArray &array);
	std::vector<uint8_t> serializeROSHeader(std_msgs::Header &header);
    std::vector<uint8_t> serializeTriggerRequest(std_srvs::TriggerRequest &request);
    std::vector<uint8_t> serializeTriggerResponse(std_srvs::TriggerResponse &response);

  private:
	const size_t header_size = 5;
	const size_t message_size = 4;
	virtual uint32_t compute_lengths_length();	// Size of lengths array
	virtual uint32_t compute_data_length();		// Size of data
	virtual std::vector<uint8_t> get_lengths(); // Bytes of lenghts array
	virtual std::vector<uint8_t> get_data();	// Bytes of data
};
