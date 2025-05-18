#pragma once

#include "PathPlanner.hpp"
#include <functional>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <memory>
#include <oneapi/tbb/task_arena.h>
#include <oneapi/tbb/task_group.h>
#include <ros/node_handle.h>
#include <ros/service_client.h>
#include <ros/subscriber.h>
#include <string>
#include <tbb/concurrent_hash_map.h>
#include <tbb/spin_rw_mutex.h>

class Car;
class Pedestrian;

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
	double gazebo_z = 0.032940;

	std::shared_ptr<tbb::task_group> task_manager;

	std::array<double, 4> spawn_area = {0, 0, 5.85, 7.50};

	ros::Subscriber car1;
	std::optional<size_t> car_idx;

	std::unique_ptr<Car> car2;
	std::unique_ptr<Car> car3;
	std::unique_ptr<Car> car4;
	std::unique_ptr<Car> car5;
	std::unique_ptr<Car> car6;
	std::unique_ptr<Car> car7;
	std::unique_ptr<Car> car8;
	std::unique_ptr<Car> car9;
	std::unique_ptr<Car> car10;

	std::unique_ptr<Pedestrian> pedestrian;

	tbb::concurrent_hash_map<std::string, std::pair<double, double>> car_positions;
	mutable tbb::spin_rw_mutex rw_mutex;

	void set_car_position(const std::string &car_name, double x, double y);
	void get_car_position(const std::string &car_name, double &out_x, double &out_y);
	bool car_in_front(const std::string &ego_car, const std::function<bool(double, double)> &pred) const;
	void stop_cars();
	double random_speed();
	void spawn_ego_car();
	void ego_car_gps_callback(const gazebo_msgs::ModelStates::ConstPtr &msg);
	void move_car_to(const std::string &car_name, double x, double y, double yaw);

  private:
	tbb::task_arena thread_pool{tbb::this_task_arena::max_concurrency() / 4};
};
