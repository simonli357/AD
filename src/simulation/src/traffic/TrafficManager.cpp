#include "TrafficManager.hpp"
#include "Controller.hpp"
#include "PathPlanner.hpp"
#include <geometry_msgs/PoseStamped.h>
#include <memory>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <vector>

TrafficManager::TrafficManager(ros::NodeHandle &nh) : nh(nh) {
	planner = std::make_unique<PathPlanner>(0.32, 40, 0.1);
    teleport_pub = nh.advertise<geometry_msgs::PoseStamped>("/car1/localisation/teleport", 1);
	spawn_ego_car();
	car1 = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/gps", 1, &TrafficManager::ego_car_gps_callback, this);
	car2 = std::make_unique<Controller>(*this, nh, random_speed(), "car2");
	car3 = std::make_unique<Controller>(*this, nh, random_speed(), "car3");
	car4 = std::make_unique<Controller>(*this, nh, random_speed(), "car4");
	car5 = std::make_unique<Controller>(*this, nh, random_speed(), "car5");
	car6 = std::make_unique<Controller>(*this, nh, random_speed(), "car6");
	car7 = std::make_unique<Controller>(*this, nh, random_speed(), "car7");
	car8 = std::make_unique<Controller>(*this, nh, random_speed(), "car8");
}

TrafficManager::~TrafficManager() {}

double TrafficManager::random_speed() {
	// Return a random speed between 0.12 and 0.32
	static std::mt19937 rng{std::random_device{}()};
	static std::uniform_real_distribution<double> dist{0.12, 0.32};
	return dist(rng);
}

void TrafficManager::spawn_ego_car() {
	std::vector<VD> candidates;
	const auto &graph = planner->track.graph;
	for (auto [vi, vend] = vertices(graph); vi != vend; ++vi) {
		const auto &v = graph[*vi];
		if (v.x >= spawn_area[0] && v.x <= spawn_area[2] && v.y >= spawn_area[1] && v.y <= spawn_area[3] && v.attribute != ATTR::INTERSECTION) {
			candidates.push_back(*vi);
		}
	}
	if (candidates.empty()) {
		return;
	}
	static std::mt19937_64 rng{std::random_device{}()};
	std::uniform_int_distribution<size_t> distr(0, candidates.size() - 1);
	VD chosen = candidates[distr(rng)];
	const auto &wp = graph[chosen];
    double yaw = 0.0;
    auto [ei, eend] = out_edges(chosen, graph);
    if (ei != eend) {
        VD next = target(*ei, graph);
        const auto &wp2 = graph[next];
        yaw = std::atan2(wp2.y - wp.y, wp2.x - wp.x);
    }
    double card = std::round(yaw / (M_PI/2.0)) * (M_PI/2.0);
    move_car_to("car1", wp.x, wp.y, card);
}

void TrafficManager::move_car_to(const std::string &car_name, double x, double y, double yaw) {
	geometry_msgs::PoseStamped cmd;
	cmd.header.stamp = ros::Time::now();
	cmd.header.frame_id = "world";
	cmd.pose.position.x = x;
	cmd.pose.position.y = y;
	cmd.pose.position.z = 0;
	tf2::Quaternion q;
	q.setRPY(0, 0, yaw);
	cmd.pose.orientation = tf2::toMsg(q);
	teleport_pub.publish(cmd);
	set_car_position(car_name, x, y);
}

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
