#include "TrafficManager.hpp"
#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/spinner.h>

int main(int argc, char **argv) {
	ros::init(argc, argv, "traffic_node");
	ros::NodeHandle nh;

    TrafficManager traffic_manager(nh);

	ros::AsyncSpinner spinner(1);
	spinner.start();

    ros::waitForShutdown();
    traffic_manager.stop_cars();
	return 0;
}
