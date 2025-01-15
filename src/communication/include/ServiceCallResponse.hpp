#include "std_msgs/Float32MultiArray.h"
#include <cstdint>
#include <vector>

class ServiceCallResponse {
  public:
	ServiceCallResponse() = default;
	ServiceCallResponse(ServiceCallResponse &&) = default;
	ServiceCallResponse(const ServiceCallResponse &) = default;
	ServiceCallResponse &operator=(ServiceCallResponse &&) = delete;
	ServiceCallResponse &operator=(const ServiceCallResponse &) = delete;
	~ServiceCallResponse() = default;

	std::vector<uint8_t> serialize();
	std::vector<uint8_t> serializeFloat32MultiArray(uint32_t length, std_msgs::Float32MultiArray &array);

  private:
	const size_t header_size = 5;
	const size_t message_size = 4;
	uint8_t data_type;
	virtual uint32_t compute_lengths_length(); // Size of lengths array
	virtual uint32_t compute_data_length();	   // Size of data
	virtual std::vector<uint8_t> get_lengths();
	virtual std::vector<uint8_t> get_data();
};
