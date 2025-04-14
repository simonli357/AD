#pragma once

#include "SplineUtils.hpp"
#include "Track.hpp"

class PathPlanner {
  public:
	PathPlanner();
	PathPlanner(PathPlanner &&) = default;
	PathPlanner(const PathPlanner &) = default;
	PathPlanner &operator=(PathPlanner &&) = default;
	PathPlanner &operator=(const PathPlanner &) = default;
	~PathPlanner() = default;

	using Vertex = Track::Vertex;

	Track track;
	SplineUtils spline_utils;

	double vref;
	int N;
	int T;
	std::string name;
	std::vector<Vertex> path;
	std::vector<Vertex> condensed_path;

	void set_constraints(double vref, int N, int T, std::string name, double start_x, double start_y, std::vector<std::tuple<float, float>> destination_positions);
	void plan_path(std::vector<double> &state_refs, std::vector<double> &input_refs, std::vector<double> &attributes, std::vector<double> &normals);

  private:
	void interpolate_path();
};
