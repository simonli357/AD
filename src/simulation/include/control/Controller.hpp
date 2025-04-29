#pragma once

#include "PathPlanner.hpp"
#include "TrafficManager.hpp"
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
	Controller(TrafficManager &traffic_manager, ros::NodeHandle &nh, double vref, std::string car_name);
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
	using ATTR = Track::ATTRIBUTE;

	void stop();

  private:
	int N = 40;
	double T = 0.1;
	bool alive = true;
	double car_radius = 0.15;
	double wp_radius = 0.02;
    size_t lookahead_wpts = 15;

	TrafficManager &traffic_manager;
	ros::NodeHandle &nh;
	ros::Publisher teleport_pub;
	std::string car_name;

	std::random_device rd;
	std::mt19937 gen;

	std::thread main;
	std::unique_ptr<PathPlanner> planner;

	std::vector<VD> destinations;
	std::vector<Vertex> path;

	void run();
	void setup();
	void plan_path();
	void move_car_to(double x, double y, double yaw);
	void find_random_cycle(Graph &graph, VD start);
	bool is_near(double x1, double y1, double x2, double y2, double rad1, double rad2);
	bool can_move_car(double x, double y, size_t idx);
};
