#include "RoundaboutManager.hpp"
#include "control/HighwayCar.hpp"
#include "control/Pedestrian.hpp"
#include <gazebo_msgs/SetModelState.h>
#include <geometry_msgs/PoseStamped.h>
#include <memory>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

RoundaboutManager::RoundaboutManager(ros::NodeHandle &nh, ros::ServiceClient &client, double car_speed) : TrafficManager(nh, client), car_speed(car_speed) {}

RoundaboutManager::~RoundaboutManager() { task_manager->wait(); }

void RoundaboutManager::initialize() {
	spawn_ego_car();
	car2 = std::make_unique<HighwayCar>(this, nh, car_speed, "car_008", "runRoundabout342");
	car3 = std::make_unique<HighwayCar>(this, nh, car_speed, "car_019", "runRoundabout368");
	car2->start();
    car3->start();
}

void RoundaboutManager::spawn_ego_car() {
	double start_x = 16.25;
	double start_y = 9.046;
	move_car_to("car1", start_x, start_y, 90 * (M_PI / 180.0));
}

void RoundaboutManager::stop_cars() {
	car2->stop();
    car3->stop();
}
