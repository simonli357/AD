#include "TrafficManager.hpp"
#include "Controller.hpp"
#include <memory>
#include <string>

TrafficManager::TrafficManager(ros::NodeHandle &nh) : nh(nh) {
	car1 = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/gps", 1, &TrafficManager::ego_car_gps_callback, this);
	car2 = std::make_unique<Controller>(*this, nh, 0.32, "car2");
	car3 = std::make_unique<Controller>(*this, nh, 0.32, "car3");
	car4 = std::make_unique<Controller>(*this, nh, 0.32, "car4");
	car5 = std::make_unique<Controller>(*this, nh, 0.32, "car5");
	car6 = std::make_unique<Controller>(*this, nh, 0.32, "car6");
	car7 = std::make_unique<Controller>(*this, nh, 0.32, "car7");
	car8 = std::make_unique<Controller>(*this, nh, 0.32, "car8");
}

TrafficManager::~TrafficManager() {}

void TrafficManager::set_car_position(const std::string &car_name, double x, double y) {
	tbb::spin_rw_mutex::scoped_lock lock(rw_mutex, true);
	tbb::concurrent_hash_map<std::string, std::pair<double, double>>::accessor a;
	car_positions.insert(a, car_name);
	a->second = {x, y};
}

bool TrafficManager::car_in_front(const std::string &ego_car, const std::function<bool(double, double)> &pred) const {
	tbb::spin_rw_mutex::scoped_lock lock(rw_mutex, false);
	for (auto it = car_positions.begin(); it != car_positions.end(); ++it) {
		if (it->first == ego_car)
			continue;
		double cx = it->second.first;
		double cy = it->second.second;
		if (pred(cx, cy)) {
			return true;
		}
	}
	return false;
}

void TrafficManager::ego_car_gps_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg) {
	double x = msg->pose.pose.position.x;
	double y = msg->pose.pose.position.y;
	set_car_position("car1", x, y);
}

void TrafficManager::stop_cars() {
	car2->stop();
	car3->stop();
	car4->stop();
	car5->stop();
	car6->stop();
	car7->stop();
	car8->stop();
}
