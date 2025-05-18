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
#include <vector>

class Car {
  public:
	Car(void *traffic_manager, ros::NodeHandle &nh, double vref, const std::string &car_name);
	Car(Car &&) = delete;
	Car(const Car &) = delete;
	Car &operator=(Car &&) = delete;
	Car &operator=(const Car &) = delete;
	virtual ~Car() = default;

	using Graph = PathPlanner::Graph;
	using Vertex = PathPlanner::Vertex;
	using Edge = PathPlanner::Edge;
	using VD = boost::graph_traits<Graph>::vertex_descriptor;
	using Pose = utils::localisationConstPtr;
	using ATTR = Track::ATTRIBUTE;

	virtual void start();

	void stop();

  protected:
	int N = 40;
	double factor = 1.0;
	double T = 0.1 * factor;
	double vref = 0.32;
	double gazebo_z = 0.032940;
	bool alive = true;
	double car_radius = 0.15;
	double wp_radius = 0.02;
	size_t lookahead_wpts = 20;

	TrafficManager *traffic_manager;
	ros::NodeHandle &nh;
	ros::Publisher teleport_pub;
	std::string car_name;

	std::random_device rd;
	std::mt19937 gen;

	std::unique_ptr<PathPlanner> planner;

	std::vector<VD> destinations;
	std::vector<Vertex> path;

	void run();
	void plan_path();
	void setup();
	void move_car_to(double x, double y, double yaw);
	void find_random_cycle(const Graph &graph, VD start);
	bool is_near(double x1, double y1, double x2, double y2, double rad1, double rad2);
	bool can_move_car(double x, double y, size_t idx);
};
