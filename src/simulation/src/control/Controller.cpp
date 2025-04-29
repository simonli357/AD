#include "Controller.hpp"
#include "map/Track.hpp"
#include <gazebo_msgs/SetModelState.h>
#include <geometry_msgs/PoseStamped.h>
#include <memory>
#include <ostream>
#include <ros/node_handle.h>
#include <ros/rate.h>
#include <ros/service_client.h>
#include <std_msgs/String.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <thread>

Controller::Controller(TrafficManager &traffic_manager, ros::NodeHandle &nh, double vref, std::string car_name) : traffic_manager(traffic_manager), nh(nh), gen(rd()) {
	this->car_name = car_name;
	planner = std::make_unique<PathPlanner>(vref * factor, N, T);
	setup();
	plan_path();

	Vertex start = path[0];

	move_car_to(start.x, start.y, start.tangent_angle);
	std::cout << car_name << " initialized." << std::endl;

	main = std::thread(&Controller::run, this);
}

Controller::~Controller() {
	if (main.joinable()) {
		main.join();
	}
}

void Controller::run() {
	ros::Rate rate(1.0 / T);
	size_t idx = 0;
	bool stopped = true;
	while (ros::ok() && alive) {
		if (path.empty()) {
			rate.sleep();
			continue;
		}
		const Vertex &v = path[idx];
		// Collision detection. If there is an obstacle in front of us, do not move
		if (can_move_car(v.x, v.y, idx)) {
			move_car_to(v.x, v.y, v.tangent_angle);
		} else {
            rate.sleep();
            continue;
        }
		// If we are at a stopline, stop for 3 seconds
		if (v.attribute == ATTR::STOPLINE && !stopped) {
			ros::Duration(3.0).sleep();
			stopped = true;
		}
		if (v.attribute != ATTR::STOPLINE && stopped) {
			stopped = false;
		}
		idx = (idx + 1) % path.size();
		rate.sleep();
	}
}

void Controller::stop() { alive = false; }

void Controller::setup() { teleport_pub = nh.advertise<geometry_msgs::PoseStamped>("/" + car_name + "/localisation/teleport", 1); }

bool Controller::is_near(double x1, double y1, double x2, double y2, double rad1, double rad2) {
	double dx = x2 - x1;
	double dy = y2 - y1;
	double rsum = rad1 + rad2;
	return (dx * dx + dy * dy) <= (rsum * rsum);
}

bool Controller::can_move_car(double x, double y, size_t idx) {
	if (path.empty())
		return false;
	for (size_t step = 1; step <= lookahead_wpts; ++step) {
		size_t i = (idx + step) % path.size();
		const auto &wp = path[i];

		const auto pred = [this, &wp](double cx, double cy) { return is_near(wp.x, wp.y, cx, cy, wp_radius, car_radius); };

		if (traffic_manager.car_in_front(car_name, pred)) {
			return false;
		}
	}
	return true;
}

void Controller::plan_path() {
	destinations.clear();
	path.clear();
	std::vector<VD> verts;
	for (auto [vi, vend] = vertices(planner->track.graph); vi != vend; ++vi) {
		verts.push_back(*vi);
	}
	while (destinations.size() < 150) {
		VD start = verts[gen() % verts.size()];
		find_random_cycle(planner->track.graph, start);
	}
	for (auto vd : destinations) {
		path.push_back(planner->track.graph[vd]);
	}
	std::vector<Vertex> general_path;
	general_path.push_back(path[0]);
	Vertex prev = path[0];
	for (const auto &v : path) {
		if (v.id == prev.id) {
			continue;
		}
		std::vector<Vertex> shortest_path = planner->track.dikstra(prev.id, v.id);
		general_path.insert(general_path.end(), shortest_path.begin() + 1, shortest_path.end());
		prev = v;
	}
	path = planner->spline_utils.interpolate_path(general_path, planner->density, planner->hw_density_factor, planner->cw_density_factor);
}

void Controller::move_car_to(double x, double y, double yaw) {
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
	traffic_manager.set_car_position(car_name, x, y);
}

void Controller::find_random_cycle(const Graph &graph, VD start) {
	std::vector<VD> temp;
	std::unordered_map<VD, size_t> visited;

	VD cur = start;
	while (true) {
		if (auto it = visited.find(cur); it != visited.end()) {
			size_t loop_start = it->second;
			for (size_t i = loop_start; i < temp.size(); ++i) {
				destinations.push_back(temp[i]);
			}
			return;
		}

		visited[cur] = temp.size();
		temp.push_back(cur);

		auto [oe, oend] = boost::out_edges(cur, graph);
		auto range = std::distance(oe, oend);
		if (range == 0) {
			return;
		}
		std::advance(oe, gen() % range);
		cur = boost::target(*oe, graph);
	}
}
