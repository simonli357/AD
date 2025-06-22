#include "HighwayManager.hpp"
#include "control/HighwayCar.hpp"
#include "control/Pedestrian.hpp"
#include <gazebo_msgs/SetModelState.h>
#include <geometry_msgs/PoseStamped.h>
#include <memory>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

HighwayManager::HighwayManager(ros::NodeHandle &nh, ros::ServiceClient &client, double car_speed) : TrafficManager(nh, client), car_speed(car_speed) {}

HighwayManager::~HighwayManager() { task_manager->wait(); }

void HighwayManager::initialize() {
	spawn_ego_car();
	car2 = std::make_unique<HighwayCar>(this, nh, car_speed, "car_008", "runHighway502");
    // car3 = std::make_unique<HighwayCar>(this, nh, 0.4, "car_019", "runHighway464");
	car2->start();
    // car3->start();
}

void HighwayManager::spawn_ego_car() {
	double start_x = 5.25;
	double start_y = 11.80;
	move_car_to("car1", start_x, start_y, 0);
}

void HighwayManager::stop_cars() {
	car2->stop();
    car3->stop();
}
