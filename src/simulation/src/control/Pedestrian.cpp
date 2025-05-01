#include "Pedestrian.hpp"

Pedestrian::Pedestrian(TrafficManager &traffic_manager, ros::NodeHandle &nh, std::string name) : traffic_manager(traffic_manager), nh(nh) {}
