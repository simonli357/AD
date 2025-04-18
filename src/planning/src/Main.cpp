#include "PathPlanner.hpp"
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/exception.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/graph_utility.hpp>
#include <cmath>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/SetModelState.h>
#include <random>
#include <ros/duration.h>
#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/service.h>
#include <thread>

using Graph = PathPlanner::Graph;
using Vertex = PathPlanner::Vertex;
using Edge = PathPlanner::Edge;
using VD = boost::graph_traits<Graph>::vertex_descriptor;

void teleportCar(ros::ServiceClient &client, const Vertex &vtx, const std::string &model_name) {
	gazebo_msgs::SetModelState srv;
	srv.request.model_state.model_name = model_name;
	srv.request.model_state.pose.position.x = vtx.x;
	srv.request.model_state.pose.position.y = vtx.y;
	srv.request.model_state.pose.position.z = 0;
	srv.request.model_state.pose.orientation.w = cos(vtx.tangent_angle / 2);
	srv.request.model_state.pose.orientation.z = sin(vtx.tangent_angle / 2);
	srv.request.model_state.reference_frame = "world";
	client.call(srv);
}

std::vector<VD> findRandomCycle(const Graph &g, VD start, std::mt19937 &gen) {
	std::unordered_map<VD, int> visited;
	std::vector<VD> path;
	VD cur = start;

	while (true) {
		visited[cur] = path.size();
		path.push_back(cur);

		auto [out_i, out_end] = out_edges(cur, g);
		if (out_i == out_end)
			break; // dead‑end

		std::advance(out_i, gen() % std::distance(out_i, out_end));
		VD nxt = target(*out_i, g);

		if (visited.count(nxt)) {
			// extract the loop portion
			int idx = visited[nxt];
			return std::vector<VD>(path.begin() + idx, path.end());
		}
		cur = nxt;
	}
	return {}; // no cycle
}

void driveLoop(const Graph &map, const std::vector<VD> &cycle, const std::string &model_name, double hz) {
	ros::NodeHandle nh;
	auto tele = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
	tele.waitForExistence();

	ros::Rate rate(hz);
	double dt = 1.0 / hz;
	size_t idx = 0;

	while (ros::ok()) {
		// grab the next edge
		VD u = cycle[idx];
		VD v = cycle[(idx + 1) % cycle.size()];

		const auto &V0 = map[u];
		const auto &V1 = map[v];

		double dx = V1.x - V0.x;
		double dy = V1.y - V0.y;
		double dist = std::hypot(dx, dy);
		double vref = std::max(V0.vref, 1.0);
		double dur = dist / vref;
		int steps = std::max(1, int(dur / dt));

		for (int s = 0; s <= steps && ros::ok(); ++s) {
			double a = double(s) / steps;
			Vertex p;
			p.x = V0.x + a * dx;
			p.y = V0.y + a * dy;
			p.tangent_angle = std::atan2(dy, dx);
			teleportCar(tele, p, model_name);
			rate.sleep();
		}

		idx = (idx + 1) % cycle.size();
	}
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "planning_node");
	ros::NodeHandle nh;

	PathPlanner planner(32, 40, 0.1);
	Graph map = planner.track.graph;

	// build a list of all vertices
	std::vector<VD> verts;
	for (auto [vi, vi_end] = vertices(map); vi != vi_end; ++vi)
		verts.push_back(*vi);

	std::random_device rd;
	std::mt19937 gen(rd());

	// --- find one random cycle of length >= 150 ---
	std::vector<VD> cycle;
	do {
		VD start = verts[gen() % verts.size()];
		cycle = findRandomCycle(map, start, gen);
	} while (cycle.size() < 150);
	ROS_INFO("Using cycle of length %zu", cycle.size());

	// list of car model names
	std::vector<std::string> models = {"sign_130", "sign_122", "sign_123"};
	double hz = 250;

	// prepare one rotated copy of that cycle per car
	std::vector<std::vector<VD>> cycles;
	cycles.reserve(models.size());
	for (size_t i = 0; i < models.size(); ++i) {
		auto c = cycle;										 // copy
		size_t offset = gen() % c.size();					 // random start
		std::rotate(c.begin(), c.begin() + offset, c.end()); // rotate
		ROS_INFO("Car %s starts at index %zu", models[i].c_str(), offset);
		cycles.push_back(std::move(c));
	}

	// spawn one thread per car
	std::vector<std::thread> threads;
	for (size_t i = 0; i < models.size(); ++i) {
		threads.emplace_back(driveLoop, std::cref(map), std::cref(cycles[i]), models[i], hz);
	}

	// wait until shutdown
	for (auto &t : threads) {
		t.join();
	}
	return 0;
}
