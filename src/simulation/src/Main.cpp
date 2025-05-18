#include "TrafficManager.hpp"
#include "HighwayManager.hpp"
#include <gazebo_msgs/SetModelState.h>
#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/spinner.h>
#include <string>

int main(int argc, char **argv) {
	ros::init(argc, argv, "traffic_node");
	ros::NodeHandle nh;
	auto tele = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
	tele.waitForExistence();

	TrafficManager *traffic_manager = nullptr;
	HighwayManager *highway_manager = nullptr;

	std::string run_type = "traffic";

	if (!nh.getParam("/run_type", run_type)) {
		std::cout << "Missing param: run_type" << std::endl;
		exit(1);
	}

	if (run_type == "trafic") {
		TrafficManager manager(nh, tele);
        traffic_manager = &manager;
	}

	if (run_type == "highway") {
        HighwayManager manager(nh, tele);
        highway_manager = &manager;
	}

	ros::AsyncSpinner spinner(1);
	spinner.start();

	ros::waitForShutdown();

    if (traffic_manager) {
        traffic_manager->stop_cars();
    }

    if (highway_manager) {
        traffic_manager->stop_cars();
    }

	return 0;
}
