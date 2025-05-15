#include "control/HighwayCar.hpp"

HighwayCar::HighwayCar(TrafficManager &traffic_manager, ros::NodeHandle &nh, double vref, std::string car_name) : Car(traffic_manager, nh, vref, car_name) {}

void HighwayCar::start() {
    plan_path();

	Vertex start = path[0];

	std::cout << "HIGHWAY CAR" << car_name << " initialized." << std::endl;
	main = std::thread(&HighwayCar::run, this);
}

void HighwayCar::run() {

}

void HighwayCar::plan_path() {

}
