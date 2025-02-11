#include "htn/World.hpp"
#include "utility.hpp"
#include <iostream>
#include <ncurses.h>
#include <ros/ros.h>
#include <signal.h>
#include <std_msgs/Byte.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include <string>
#include <thread>

std::unique_ptr<World> world;

void signalHandler(int signum) {
	if (world) {
		world->utils.stop_car();
		world->call_trigger_service();
	}
	if (world && world->utils.serial && world->utils.serial->is_open()) {
		world->utils.serial->close();
	}
	world->utils.serial.reset();
	ros::shutdown();
	exit(signum);
}

int main(int argc, char **argv) {
	std::cout.precision(3);
	// create anonymous node handle
	ros::init(argc, argv, "mpc_node", ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
	ros::NodeHandle nh;
	double T, v_ref, T_park;
	int N;
	bool sign, ekf, lane, real;
	std::string name;
	std::string nodeName = ros::this_node::getName();
	std::cout << "node name: " << nodeName << std::endl;
	bool success = nh.getParam(nodeName + "/lane", lane) && nh.getParam(nodeName + "/ekf", ekf) && nh.getParam(nodeName + "/sign", sign) && nh.getParam("T", T) && nh.getParam("N", N) &&
				   nh.getParam("constraints/v_ref", v_ref);
	double x0, y0, yaw0, vref;
	success = success && nh.getParam(nodeName + "/name", name) && nh.getParam(nodeName + "/vref", vref) && nh.getParam(nodeName + "/x0", x0) && nh.getParam(nodeName + "/y0", y0) &&
			  nh.getParam(nodeName + "/yaw0", yaw0);
	success = success && nh.getParam("/T_park", T_park);
	success = success && nh.getParam(nodeName + "/real", real);
	if (!success) {
		std::cout << "Failed to get parameters" << std::endl;
		exit(1);
	} else {
		std::cout << "Successfully loaded parameters" << std::endl;
	}
	if (vref > 30)
		vref = 35.;
	std::cout << "ekf: " << ekf << ", sign: " << sign << ", T: " << T << ", N: " << N << ", vref: " << vref << ", real: " << real << std::endl;

	signal(SIGINT, signalHandler);

	world = std::make_unique<World>(nh, T, N, vref, sign, ekf, lane, T_park, name, x0, y0, yaw0, real);

	return 0;
}
