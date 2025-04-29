#include "Controller.hpp"
#include <gazebo_msgs/SetModelState.h>
#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/spinner.h>

int main(int argc, char **argv) {
	ros::init(argc, argv, "traffic_node");
	ros::NodeHandle nh;

	auto model_states_client = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
	model_states_client.waitForExistence();

	Controller car2(nh, model_states_client, 0.32, "car2");

	ros::AsyncSpinner spinner(1);
	spinner.start();

    ros::waitForShutdown();
	return 0;
}
