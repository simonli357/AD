#pragma once

#include <functional>
#include <ros/node_handle.h>
#include <ros/subscriber.h>
#include <string>
#include <tbb/concurrent_hash_map.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tbb/spin_rw_mutex.h>

class Controller;

class TrafficManager {
  public:
	TrafficManager(ros::NodeHandle &nh);
	TrafficManager(TrafficManager &&) = delete;
	TrafficManager(const TrafficManager &) = delete;
	TrafficManager &operator=(TrafficManager &&) = delete;
	TrafficManager &operator=(const TrafficManager &) = delete;
	~TrafficManager();

	ros::NodeHandle &nh;

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
    void ego_car_gps_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);
};
