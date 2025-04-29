#include "TrafficManager.hpp"
#include <gazebo_msgs/SetModelState.h>
#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/spinner.h>

int main(int argc, char **argv) {
	ros::init(argc, argv, "traffic_node");
	ros::NodeHandle nh;
    auto tele = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    tele.waitForExistence();

    TrafficManager traffic_manager(nh, tele);

	ros::AsyncSpinner spinner(1);
	spinner.start();

    ros::waitForShutdown();
    traffic_manager.stop_cars();
	return 0;
}
