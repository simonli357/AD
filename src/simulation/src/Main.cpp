#include "DottedManager.hpp"
#include "HighwayManager.hpp"
#include "RoundaboutManager.hpp"
#include "SpeedCurveManager.hpp"
#include "TrafficManager.hpp"
#include <gazebo_msgs/SetModelState.h>
#include <memory>
#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/spinner.h>
#include <string>

int main(int argc, char **argv) {
	ros::init(argc, argv, "traffic_node");
	ros::NodeHandle nh;
	auto tele = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
	tele.waitForExistence();

	std::unique_ptr<TrafficManager> traffic_manager;
	std::unique_ptr<HighwayManager> highway_manager;
	std::unique_ptr<SpeedCurveManager> speed_curve_manager;
	std::unique_ptr<RoundaboutManager> roundabout_manager;
	std::unique_ptr<DottedManager> dotted_manager;

	std::string run_type = "traffic";
	double car_speed = 0.1;

	if (!nh.getParam("/run_type", run_type)) {
		std::cout << "Missing param: run_type" << std::endl;
		return 1;
	}

	if (!nh.getParam("/car_speed", car_speed)) {
		std::cout << "Missing param: car_speed (optional)" << std::endl;
	}

	if (run_type == "traffic") {
		traffic_manager = std::make_unique<TrafficManager>(nh, tele);
		traffic_manager->initialize();
	} else if (run_type == "highway") {
		highway_manager = std::make_unique<HighwayManager>(nh, tele, car_speed);
		highway_manager->initialize();
	} else if (run_type == "speedcurve") {
		speed_curve_manager = std::make_unique<SpeedCurveManager>(nh, tele, car_speed);
		speed_curve_manager->initialize();
	} else if (run_type == "roundabout") {
		roundabout_manager = std::make_unique<RoundaboutManager>(nh, tele, car_speed);
		roundabout_manager->initialize();
	} else if (run_type == "dotted") {
		dotted_manager = std::make_unique<DottedManager>(nh, tele, car_speed);
		dotted_manager->initialize();
	} else {
		std::cerr << "Unknown run_type: " << run_type << std::endl;
		return 1;
	}

	ros::AsyncSpinner spinner(1);
	spinner.start();

	ros::waitForShutdown();

	return 0;
}
