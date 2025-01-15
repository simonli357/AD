#include "std_msgs/Float32MultiArray.h"
#include "ServiceCallResponse.hpp"

using std_msgs::Float32MultiArray;

class GoToSrvResponse : public ServiceCallResponse {
  public:
	GoToSrvResponse(Float32MultiArray state_refs, Float32MultiArray input_refs, Float32MultiArray wp_attributes, Float32MultiArray wp_normals);
	GoToSrvResponse(GoToSrvResponse &&) = default;
	GoToSrvResponse(const GoToSrvResponse &) = default;
	GoToSrvResponse &operator=(GoToSrvResponse &&) = delete;
	GoToSrvResponse &operator=(const GoToSrvResponse &) = delete;
	~GoToSrvResponse() = default;

  private:
    Float32MultiArray state_refs;
    Float32MultiArray input_refs;
    Float32MultiArray wp_attributes;
    Float32MultiArray wp_normals;
};
