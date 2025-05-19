#include "SpeedCurveManager.hpp"
#include "control/HighwayCar.hpp"
#include "control/Pedestrian.hpp"
#include <gazebo_msgs/SetModelState.h>
#include <geometry_msgs/PoseStamped.h>
#include <memory>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

SpeedCurveManager::SpeedCurveManager(ros::NodeHandle &nh, ros::ServiceClient &client, double car_speed) : TrafficManager(nh, client), car_speed(car_speed) {}

SpeedCurveManager::~SpeedCurveManager() { task_manager->wait(); }

void SpeedCurveManager::initialize() {
	spawn_ego_car();
	car2 = std::make_unique<HighwayCar>(this, nh, car_speed, "car_008", "runSpeedCurve151");
	car3 = std::make_unique<HighwayCar>(this, nh, car_speed, "car_019", "runSpeedCurve166");
	car2->start();
    car3->start();
}

void SpeedCurveManager::spawn_ego_car() {
	double start_x = 5.14;
	double start_y = 5.776;
	move_car_to("car1", start_x, start_y, 0);
}

void SpeedCurveManager::stop_cars() {
	car2->stop();
    car3->stop();
}
