#include "Controller.hpp"
#include "map/Track.hpp"
#include "utils/localisation.h"
#include <gazebo_msgs/SetModelState.h>
#include <memory>
#include <ros/node_handle.h>
#include <ros/rate.h>
#include <ros/service_client.h>
#include <std_msgs/String.h>
#include <thread>

Controller::Controller(ros::NodeHandle &nh, ros::ServiceClient &model_states_client, double vref, std::string car_name) : nh(nh), model_states_client(model_states_client), gen(rd()) {
	this->car_name = car_name;
	planner = std::make_unique<PathPlanner>(vref, N, T);
	setup();
    plan_path();

    VD start = path[0];
    const auto& graph = planner->track.graph;
    const Vertex& vp  = graph[start];
    double x0 = vp.x;
    double y0 = vp.y;
    double yaw0 = vp.tangent_angle;
    
    set_pose(x0, y0, yaw0);

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
    const double proximity_threshold = 0.1;

    while (ros::ok()) {
        if (!current_pose) {
            rate.sleep();
            continue;
        }
        if (path.empty()) {
            rate.sleep();
            continue;
        }
        double cx   = current_pose->posA;
        double cy   = current_pose->posB;
        double cyaw = current_pose->rotA;

        VD target_v = path[idx];
        const auto& graph = planner->track.graph;
        const Vertex& vp  = graph[target_v];
        double tx = vp.x;
        double ty = vp.y;

        double dx = tx - cx;
        double dy = ty - cy;
        double dist = std::hypot(dx, dy);
        double desired_yaw = std::atan2(dy, dx);
        double yaw_error = std::atan2(std::sin(desired_yaw - cyaw), std::cos(desired_yaw - cyaw));

        double speed = std::min(planner->vref, dist);

        publish_cmd(yaw_error, speed);

        if (dist < proximity_threshold) {
            idx = (idx + 1) % path.size();
        }

        rate.sleep();
    }
}

void Controller::setup() {
	nh.subscribe<utils::localisation>("/" + car_name + "/localisation", 1, &Controller::fetch_current_pose, this);
	cmd_vel_pub = nh.advertise<std_msgs::String>("/" + car_name + "/command", 8);
}

void Controller::fetch_current_pose(const Pose &msg) { current_pose = msg; }

void Controller::plan_path() {
	std::vector<VD> verts;
	for (auto [vi, vi_end] = vertices(planner->track.graph); vi != vi_end; ++vi) {
		verts.push_back(*vi);
	}
	do {
		VD start = verts[gen() & verts.size()];
		find_random_cycle(planner->track.graph, start);
	} while (path.size() < 150);
}

void Controller::publish_cmd(double angle, double speed) {
	rot.data = "{\"action\":\"2\",\"steerAngle\":" + std::to_string(angle) + "}";
	vel.data = "{\"action\":\"1\",\"speed\":" + std::to_string(speed) + "}";
	cmd_vel_pub.publish(rot);
	cmd_vel_pub.publish(vel);
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
		visited[cur] = path.size();
		path.push_back(cur);

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
