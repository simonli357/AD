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

Controller::Controller(ros::NodeHandle &nh, ros::ServiceClient &model_states_client, double vref, std::string car_name) : nh(nh), model_states_client(model_states_client), gen(rd()) {
	this->car_name = car_name;
	planner = std::make_unique<PathPlanner>(vref, N, T);
	setup();
	plan_path();

	Vertex start = path[0];

	set_pose(start.x, start.y, start.tangent_angle);
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
	while (ros::ok() && alive) {
		if (path.empty()) {
			rate.sleep();
			continue;
		}
		const Vertex &v = path[idx];
        // Collision detection. If there is an obstacle in front of us, do not move
        if (can_move_car(v.x, v.y)) {
            move_car_to(v.x, v.y, v.tangent_angle);
        }

        // If we are at a stopline, stop for 3 seconds
        if (v.attribute == ATTR::STOPLINE && path[(idx + 1) % path.size()].attribute != ATTR::STOPLINE) {
            ros::Duration(3.0).sleep();
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

bool Controller::can_move_car(double x, double y) {

}

void Controller::plan_path() {
	std::vector<VD> verts;
	for (auto [vi, vi_end] = vertices(planner->track.graph); vi != vi_end; ++vi) {
		verts.push_back(*vi);
	}
	do {
		VD start = verts[gen() % verts.size()];
		find_random_cycle(planner->track.graph, start);
	} while (destinations.size() < 150);
	for (const auto &v : destinations) {
		path.push_back(planner->track.graph[v]);
	}
	path = planner->spline_utils.interpolate_path(path, planner->density, planner->hw_density_factor, planner->cw_density_factor);
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
}

void Controller::set_pose(double x, double y, double yaw) {
	gazebo_msgs::SetModelState srv;
	srv.request.model_state.model_name = car_name;
	srv.request.model_state.pose.position.x = x;
	srv.request.model_state.pose.position.y = y;
	srv.request.model_state.pose.position.z = 0;
	srv.request.model_state.pose.orientation.w = cos(yaw / 2);
	srv.request.model_state.pose.orientation.z = sin(yaw / 2);
	srv.request.model_state.reference_frame = "world";
	model_states_client.call(srv);
}

void Controller::find_random_cycle(Graph &graph, VD start) {
	std::unordered_map<VD, int> visited;
	VD cur = start;

	while (true) {
		visited[cur] = destinations.size();
		destinations.push_back(cur);

		auto [out_i, out_end] = boost::out_edges(cur, graph);
		if (out_i == out_end) {
			break; // dead‑end
		}

		std::advance(out_i, gen() % std::distance(out_i, out_end));
		VD nxt = target(*out_i, graph);

		if (visited.count(nxt)) {
			// extract the loop portion
			int idx = visited[nxt];
			return;
		}
		cur = nxt;
	}
}
