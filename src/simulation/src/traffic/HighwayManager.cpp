#include "traffic/HighwayManager.hpp"
#include "PathPlanner.hpp"
#include "control/HighwayCar.hpp"
#include "control/Pedestrian.hpp"
#include <gazebo_msgs/SetModelState.h>
#include <geometry_msgs/PoseStamped.h>
#include <memory>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

HighwayManager::HighwayManager(ros::NodeHandle &nh, ros::ServiceClient &client) : nh(nh), client(client) {
	thread_pool.execute([this] { task_manager = std::make_shared<tbb::task_group>(); });
	planner = std::make_unique<PathPlanner>(0.32, 40, 0.1);
	/* spawn_ego_car(); */
	car1 = nh.subscribe("/gazebo/model_states", 3, &HighwayManager::ego_car_gps_callback, this);
	car2 = std::make_unique<HighwayCar>(*this, nh, random_speed(), "car_008", "runHighway502");
	car3 = std::make_unique<HighwayCar>(*this, nh, random_speed(), "car_014", "runHighway483");

	pedestrian = std::make_unique<Pedestrian>(*this, nh, "pedestrian_object");

	car2->start();
	car3->start();
}

HighwayManager::~HighwayManager() {
    task_manager->wait();
}

double HighwayManager::random_speed() {
	// Return a random speed between 0.12 and 0.32
	static std::mt19937 rng{std::random_device{}()};
	static std::uniform_real_distribution<double> dist{0.12, 0.32};
	return dist(rng);
}

void HighwayManager::spawn_ego_car() {
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
	double card = std::round(yaw / (M_PI / 2.0)) * (M_PI / 2.0);
	move_car_to("car1", wp.x, wp.y, card);
	ROS_INFO("spawn_ego_car: placed car1 at (%.2f,%.2f) yaw=%.2f°", wp.x, wp.y, card * 180.0 / M_PI);
}

void HighwayManager::move_car_to(const std::string &car_name, double x, double y, double yaw) {
	gazebo_msgs::SetModelState srv;
	srv.request.model_state.model_name = car_name;
	srv.request.model_state.pose.position.x = x;
	srv.request.model_state.pose.position.y = y;
	srv.request.model_state.pose.position.z = gazebo_z;
	srv.request.model_state.pose.orientation.w = cos(yaw / 2);
	srv.request.model_state.pose.orientation.z = sin(yaw / 2);
	srv.request.model_state.reference_frame = "world";
	client.call(srv);
	set_car_position(car_name, x, y);
}

void HighwayManager::get_car_position(const std::string &car_name, double &out_x, double &out_y) {
	tbb::spin_rw_mutex::scoped_lock lock(rw_mutex, /*write=*/false);
	tbb::concurrent_hash_map<std::string, std::pair<double, double>>::const_accessor a;
	if (car_positions.find(a, car_name)) {
		out_x = a->second.first;
		out_y = a->second.second;
	} else {
		out_x = 0.0;
		out_y = 0.0;
		ROS_WARN("HighwayManager: no position found for '%s'", car_name.c_str());
	}
}

void HighwayManager::set_car_position(const std::string &car_name, double x, double y) {
	tbb::spin_rw_mutex::scoped_lock lock(rw_mutex, true);
	tbb::concurrent_hash_map<std::string, std::pair<double, double>>::accessor a;
	car_positions.insert(a, car_name);
	a->second = {x, y};
}

bool HighwayManager::car_in_front(const std::string &ego_car, const std::function<bool(double, double)> &pred) const {
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

void HighwayManager::ego_car_gps_callback(const gazebo_msgs::ModelStates::ConstPtr &msg) {
	if (!car_idx.has_value()) {
		auto it = std::find(msg->name.begin(), msg->name.end(), "car1");
		if (it != msg->name.end()) {
			car_idx = std::distance(msg->name.begin(), it);
			std::cout << "automobile found: " << *car_idx << std::endl;
		} else {
			printf("automobile not found\n");
			return;
		}
	}
	double gps_x = msg->pose[*car_idx].position.x;
	double gps_y = msg->pose[*car_idx].position.y;
	set_car_position("car1", gps_x, gps_y);
}

void HighwayManager::stop_cars() {
	car2->stop();
	car3->stop();
}
