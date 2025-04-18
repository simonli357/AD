#include "PathPlanner.hpp"
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>
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

int main(int argc, char **argv) {
	ros::init(argc, argv, "planning_node");
	ros::NodeHandle nh;
	PathPlanner planner(32, 40, 0.1);
	Graph map = planner.condensed_graph;

	ros::ServiceClient teleport_client = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
	teleport_client.waitForExistence();

	std::random_device rd;
	std::mt19937 gen(rd());

	std::vector<VD> vertices_vec;
	for (auto vp = vertices(map); vp.first != vp.second; ++vp.first)
		vertices_vec.push_back(*vp.first);

	VD current = vertices_vec[gen() % vertices_vec.size()];

	std::vector<VD> path;
	// Traverse a path through the graph
	while (ros::ok()) {
		// Walk until we hit a junction or dead-end
		path.clear();
		path.push_back(current);
		while (true) {
			auto [out_begin, out_end] = out_edges(current, map);
			if (std::distance(out_begin, out_end) != 1)
				break;
			VD next = target(*out_begin, map);
			path.push_back(next);
			current = next;
		}

		// Walk the smoothed segment with interpolation
		for (size_t i = 0; i + 1 < path.size() && ros::ok(); ++i) {
			const Vertex &v0 = map[path[i]];
			const Vertex &v1 = map[path[i + 1]];

			double dx = v1.x - v0.x;
			double dy = v1.y - v0.y;
			double dist = std::sqrt(dx * dx + dy * dy);

			double vref = std::max(v0.vref, 1.0);
			double duration = dist / vref;

			int steps = std::max(1, static_cast<int>(duration / 0.02));
			for (int s = 0; s <= steps && ros::ok(); ++s) {
				double alpha = static_cast<double>(s) / steps;
				Vertex interp;
				interp.x = v0.x + alpha * dx;
				interp.y = v0.y + alpha * dy;
				interp.tangent_angle = std::atan2(dy, dx);
				teleportCar(teleport_client, interp);
				ros::Duration(0.02).sleep();
			}
		}

		// Choose a new random point after one chain finishes
		current = vertices_vec[gen() % vertices_vec.size()];
        teleportCar(teleport_client, map[current]);
        ros::Duration(1.0).sleep();
	}
}
