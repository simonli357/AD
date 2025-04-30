#pragma once

#include "PathPlanner.hpp"
#include <functional>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <memory>
#include <ros/node_handle.h>
#include <ros/service_client.h>
#include <ros/subscriber.h>
#include <string>
#include <tbb/concurrent_hash_map.h>
#include <tbb/spin_rw_mutex.h>

class Controller;

class TrafficManager {
  public:
	TrafficManager(ros::NodeHandle &nh, ros::ServiceClient &client);
	TrafficManager(TrafficManager &&) = delete;
	TrafficManager(const TrafficManager &) = delete;
	TrafficManager &operator=(TrafficManager &&) = delete;
	TrafficManager &operator=(const TrafficManager &) = delete;
	~TrafficManager();

	using Graph = PathPlanner::Graph;
	using Vertex = PathPlanner::Vertex;
	using Edge = PathPlanner::Edge;
	using VD = boost::graph_traits<Graph>::vertex_descriptor;
	using ATTR = Track::ATTRIBUTE;

	ros::NodeHandle &nh;
	ros::ServiceClient &client;
	std::unique_ptr<PathPlanner> planner;

	std::array<double, 4> spawn_area = {0, 0, 5.85, 7.50};

	ros::Subscriber car1;
	std::unique_ptr<Controller> car2;
	std::unique_ptr<Controller> car3;
	std::unique_ptr<Controller> car4;
	std::unique_ptr<Controller> car5;
	std::unique_ptr<Controller> car6;
	std::unique_ptr<Controller> car7;
	std::unique_ptr<Controller> car8;

	tbb::concurrent_hash_map<std::string, std::pair<double, double>> car_positions;
	mutable tbb::spin_rw_mutex rw_mutex;

	void set_car_position(const std::string &car_name, double x, double y);
	bool car_in_front(const std::string &ego_car, const std::function<bool(double, double)> &pred) const;
	void stop_cars();
	double random_speed();
	void spawn_ego_car();
	void ego_car_gps_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);
	void move_car_to(const std::string &car_name, double x, double y, double yaw);
};
