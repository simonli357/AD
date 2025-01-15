#include "std_msgs/Float32MultiArray.h"

using std_msgs::Float32MultiArray;

class GoToSrvResponse {
  public:
	GoToSrvResponse(Float32MultiArray state_refs, Float32MultiArray input_refs, Float32MultiArray wp_attributes, Float32MultiArray wp_normals);
	GoToSrvResponse(GoToSrvResponse &&) = default;
	GoToSrvResponse(const GoToSrvResponse &) = default;
	GoToSrvResponse &operator=(GoToSrvResponse &&) = default;
	GoToSrvResponse &operator=(const GoToSrvResponse &) = default;
	~GoToSrvResponse() = default;

  private:
    Float32MultiArray state_refs;
    Float32MultiArray input_refs;
    Float32MultiArray wp_attributes;
    Float32MultiArray wp_normals;
};
