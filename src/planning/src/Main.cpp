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

using Graph = PathPlanner::Graph;
using Vertex = PathPlanner::Vertex;
using Edge = PathPlanner::Edge;
using VD = boost::graph_traits<Graph>::vertex_descriptor;

void teleportCar(ros::ServiceClient &client, const Vertex &vtx) {
	gazebo_msgs::SetModelState srv;
	srv.request.model_state.model_name = "car1";
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

int main(int argc, char **argv) {
	ros::init(argc, argv, "planning_node");
	ros::NodeHandle nh;

	PathPlanner planner(32, 40, 0.1);
	Graph map = planner.condensed_graph;

	auto tele_client = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
	tele_client.waitForExistence();

	// build a list of all vertices
	std::vector<VD> verts;
	for (auto [vi, vi_end] = vertices(map); vi != vi_end; ++vi)
		verts.push_back(*vi);

	std::random_device rd;
	std::mt19937 gen(rd());

	// pick an arbitrary start
	VD start = verts[gen() % verts.size()];

	std::vector<VD> cycle;
    std::vector<VD> best;
    // try N random starts and keep the longest
    for (int i = 0; i < 200; ++i) {
        VD s = verts[gen() % verts.size()];
        auto c = findRandomCycle(map, s, gen);
        if (c.size() > best.size())
            best = std::move(c);
    }
    cycle = std::move(best);
    ROS_INFO("Longest random cycle has length %zu", cycle.size());

	if (cycle.empty()) {
		ROS_ERROR("No cycle found in the graph at all!");
		return 1;
	}

	// 3) Drive that cycle forever
	size_t idx = 0;
	ros::Rate rate(50); // 50Hz → 0.02s steps
	while (ros::ok()) {
		VD u = cycle[idx];
		VD v = cycle[(idx + 1) % cycle.size()];

		// straight‐line interpolation exactly as before
		const auto &V0 = map[u];
		const auto &V1 = map[v];
		double dx = V1.x - V0.x, dy = V1.y - V0.y;
		double dist = std::hypot(dx, dy);
		double vref = std::max(V0.vref, 1.0);
		double dur = dist / vref;
		int steps = std::max(1, int(dur / 0.02));

		for (int s = 0; s <= steps && ros::ok(); ++s) {
			double a = double(s) / steps;
			Vertex p;
			p.x = V0.x + a * dx;
			p.y = V0.y + a * dy;
			p.tangent_angle = std::atan2(dy, dx);
			teleportCar(tele_client, p);
			rate.sleep();
		}

		idx = (idx + 1) % cycle.size();
	}
	return 0;
}
