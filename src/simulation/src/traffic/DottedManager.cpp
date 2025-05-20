#include "DottedManager.hpp"
#include "control/HighwayCar.hpp"
#include "control/Pedestrian.hpp"
#include <gazebo_msgs/SetModelState.h>
#include <geometry_msgs/PoseStamped.h>
#include <memory>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

DottedManager::DottedManager(ros::NodeHandle &nh, ros::ServiceClient &client, double car_speed) : TrafficManager(nh, client), car_speed(car_speed) {}

DottedManager::~DottedManager() { task_manager->wait(); }

void DottedManager::initialize() {
	spawn_ego_car();
	car2 = std::make_unique<HighwayCar>(this, nh, car_speed, "car_008", "runDotted305");
	car3 = std::make_unique<HighwayCar>(this, nh, car_speed, "car_019", "runDotted318");
	car2->start();
    car3->start();
}

void DottedManager::spawn_ego_car() {
	double start_x = 16.2735;
	double start_y = 4.02074;
	move_car_to("car1", start_x, start_y, 180 * (M_PI / 180.0));
}

void DottedManager::stop_cars() {
	car2->stop();
    car3->stop();
}
