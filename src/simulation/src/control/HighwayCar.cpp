#include "HighwayCar.hpp"
#include <iostream>
#include <ostream>
#include <string>

HighwayCar::HighwayCar(void *traffic_manager, ros::NodeHandle &nh, double vref, const std::string &car_name, const std::string &path_name) : Car(traffic_manager, nh, vref, car_name) { this->car_path = path_name; }

void HighwayCar::start() {
	get_path();
	Vertex start = path[0];
	std::cout << "HIGHWAY CAR: " << car_name << " initialized." << std::endl;
	traffic_manager->task_manager->run([this] { run(); });
}

void HighwayCar::get_path() {
	path.clear();
	planner->set_constraints(vref, N, T, car_path);
	planner->print_path();
	path = planner->plan_path();
}
