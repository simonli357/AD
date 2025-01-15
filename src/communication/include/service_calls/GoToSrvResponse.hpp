#include "ServiceCallResponse.hpp"
#include "ros/serialization.h"
#include "std_msgs/Float32MultiArray.h"
#include <cstdint>

using std_msgs::Float32MultiArray;

class GoToSrvResponse : private ServiceCallResponse {
  public:
	GoToSrvResponse(Float32MultiArray &state_refs, Float32MultiArray &input_refs, Float32MultiArray &wp_attributes, Float32MultiArray &wp_normals);
	GoToSrvResponse(GoToSrvResponse &&) = default;
	GoToSrvResponse(const GoToSrvResponse &) = default;
	GoToSrvResponse &operator=(GoToSrvResponse &&) = delete;
	GoToSrvResponse &operator=(const GoToSrvResponse &) = delete;
	~GoToSrvResponse() = default;

  private:
	const size_t num_elements = 4;
	const size_t length_bytes = 4;
	Float32MultiArray &state_refs;
	Float32MultiArray &input_refs;
	Float32MultiArray &wp_attributes;
	Float32MultiArray &wp_normals;
	uint32_t state_refs_length = ros::serialization::serializationLength(state_refs);
	uint32_t input_refs_length = ros::serialization::serializationLength(input_refs);
	uint32_t wp_attributes_length = ros::serialization::serializationLength(wp_attributes);
	uint32_t wp_normals_length = ros::serialization::serializationLength(wp_normals);
	uint32_t lengths_length = num_elements * length_bytes;
	uint32_t data_length = state_refs_length + input_refs_length + wp_attributes_length + wp_normals_length;
	uint32_t compute_lengths_length() override;
	uint32_t compute_data_length() override;
	std::vector<uint8_t> get_lengths() override;
	std::vector<uint8_t> get_data() override;
};
