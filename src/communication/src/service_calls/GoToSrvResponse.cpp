#include "std_msgs/Float32MultiArray.h"
#include "service_calls/GoToSrvResponse.hpp"

using std_msgs::Float32MultiArray;

GoToSrvResponse::GoToSrvResponse(Float32MultiArray state_refs, Float32MultiArray input_refs, Float32MultiArray wp_attributes, Float32MultiArray wp_normals)
	: state_refs(state_refs), input_refs(input_refs), wp_attributes(wp_attributes), wp_normals(wp_normals) {}
