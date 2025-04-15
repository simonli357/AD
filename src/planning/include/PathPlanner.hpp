#pragma once

#include "map/Track.hpp"
#include "utils/FilterUtils.hpp"
#include "utils/SplineUtils.hpp"
#include <cmath>
#include <std_msgs/Float32MultiArray.h>

class PathPlanner {
  public:
	PathPlanner();
	PathPlanner(PathPlanner &&) = default;
	PathPlanner(const PathPlanner &) = default;
	PathPlanner &operator=(PathPlanner &&) = default;
	PathPlanner &operator=(const PathPlanner &) = default;
	~PathPlanner() = default;

	using Vertex = Track::Vertex;
	using Float32MultiArray = std_msgs::Float32MultiArray;

	Track track;
	SplineUtils spline_utils;
	FilterUtils filter_utils;

	double vref;
	int N;
	double T;
	double density;
	std::string name;
	std::vector<Vertex> path;
	std::vector<Vertex> condensed_path;

    double distance_threshold;
    double yaw_threshold = 60 * M_PI / 180;

	void set_constraints(double vref, int N, int T, double start_x, double start_y, std::vector<std::tuple<float, float>> destination_positions);
	void set_constraints(double vref, int N, int T, double start_x, double start_y, std::string name);
	void plan_path(Float32MultiArray &state_refs, Float32MultiArray &input_refs, Float32MultiArray &attributes, Float32MultiArray &normals);

  private:
	void precompute_path();
	void construct_path();
};
