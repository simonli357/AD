#include "SpeedCurveManager.hpp"
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
    SpeedCurveManager *speed_curve_manager = nullptr;

	std::string run_type = "traffic";
    double car_speed = 0.1;

	if (!nh.getParam("/run_type", run_type)) {
		std::cout << "Missing param: run_type" << std::endl;
		exit(1);
	}

	if (!nh.getParam("/car_speed", car_speed)) {
		std::cout << "Missing param: car_speed (optional)" << std::endl;
	}

	if (run_type == "trafic") {
		TrafficManager manager(nh, tele);
        traffic_manager = &manager;
        traffic_manager->initialize();
	} else if (run_type == "highway") {
        HighwayManager manager(nh, tele, car_speed);
        highway_manager = &manager;
        highway_manager->initialize();
	} else if (run_type == "speedcurve") {
        SpeedCurveManager manager(nh, tele, car_speed);
        speed_curve_manager = &manager;
        speed_curve_manager->initialize();
    }

	ros::AsyncSpinner spinner(1);
	spinner.start();

	ros::waitForShutdown();

    if (traffic_manager) {
        traffic_manager->stop_cars();
        free(traffic_manager);
    }

    if (highway_manager) {
        highway_manager->stop_cars();
        free(highway_manager);
    }

    if (speed_curve_manager) {
        speed_curve_manager->stop_cars();
        free(speed_curve_manager);
    }

	return 0;
}
