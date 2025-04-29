#pragma once

#include "PathPlanner.hpp"
#include "utils/localisation.h"
#include <memory>
#include <random>
#include <ros/node_handle.h>
#include <ros/service_client.h>
#include <std_msgs/String.h>
#include <string>
#include <thread>
#include <vector>

class Controller {
  public:
	Controller(ros::NodeHandle &nh, ros::ServiceClient &client, double vref, std::string car_name);
	Controller(Controller &&) = delete;
	Controller(const Controller &) = delete;
	Controller &operator=(Controller &&) = delete;
	Controller &operator=(const Controller &) = delete;
	~Controller();

	using Graph = PathPlanner::Graph;
	using Vertex = PathPlanner::Vertex;
	using Edge = PathPlanner::Edge;
	using VD = boost::graph_traits<Graph>::vertex_descriptor;
	using Pose = utils::localisationConstPtr;

  private:
	int N = 40;
	double T = 0.1;

	ros::NodeHandle &nh;
	ros::ServiceClient &model_states_client;
	ros::Publisher cmd_vel_pub;
	std::string car_name;

	std_msgs::String rot;
	std_msgs::String vel;

	std::random_device rd;
	std::mt19937 gen;

	std::thread main;
	std::unique_ptr<PathPlanner> planner;

	std::vector<VD> path;
	Pose current_pose;

	void run();
	void setup();
	void plan_path();
	void fetch_current_pose(const Pose &msg);
	void publish_cmd(double angle, double velocity);
	void set_pose(double x, double y, double yaw);
	void find_random_cycle(Graph &graph, VD start);
};
