#include "PathPlanner.hpp"
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/SetModelState.h>
#include <random>
#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/service.h>

using Graph = PathPlanner::Graph;
using Vertex = PathPlanner::Vertex;
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

	// Random number generator for vertex selection
	std::random_device rd;
	std::mt19937 gen(rd());

	std::vector<VD> vertices_vec;
	for (auto vp = vertices(map); vp.first != vp.second; ++vp.first)
		vertices_vec.push_back(*vp.first);

	VD current = vertices_vec[gen() % vertices_vec.size()];

	ros::Rate rate(10); // 10 Hz teleport rate

	while (ros::ok()) {
		teleportCar(teleport_client, map[current]);
		ros::spinOnce();

		// Choose next vertex ensuring it has outgoing edges
		std::vector<VD> next_options;
		for (auto ep = out_edges(current, map); ep.first != ep.second; ++ep.first)
			next_options.push_back(target(*ep.first, map));

		if (next_options.empty()) {
			// If dead end, pick a new random vertex
			current = vertices_vec[gen() % vertices_vec.size()];
		} else {
			current = next_options[gen() % next_options.size()];
		}

		rate.sleep();
	}

	return 0;
}
