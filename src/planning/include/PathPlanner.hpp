#pragma once

#include "map/Track.hpp"
#include "utils/PathUtils.hpp"
#include "utils/SplineUtils.hpp"
#include <cmath>
#include <std_msgs/Float32MultiArray.h>
#include <fstream>
#include <iomanip>
#include <stdexcept>
#include <string>

class PathPlanner {
  public:
	PathPlanner(double vref, int N, double T);
	PathPlanner(PathPlanner &&) = default;
	PathPlanner(const PathPlanner &) = default;
	PathPlanner &operator=(PathPlanner &&) = default;
	PathPlanner &operator=(const PathPlanner &) = default;
	~PathPlanner() = default;

	using Vertex = Track::Vertex;
	using Edge = Track::Edge;
	using Graph = Track::Graph;
	using Float32MultiArray = std_msgs::Float32MultiArray;

	Track track;
	SplineUtils spline_utils;
	PathUtils path_utils;

	double vref;
	int N;
	double T;
	std::string name;
	std::vector<Vertex> path;
	std::vector<Vertex> condensed_path;

	std::string serialized_graph = track.serialize_graph(track.graph);

	double density;
	double hw_density_factor = 4.0 / 3.0;
	double cw_density_factor = 3.0 / 2.0;
	double distance_threshold;
	double yaw_threshold = 60 * M_PI / 180;

	void set_constraints(double vref, int N, double T, double start_x, double start_y, std::vector<std::tuple<float, float>> destination_positions);
	void set_constraints(double vref, int N, double T, double start_x, double start_y, std::string name);
	void plan_path(Float32MultiArray &state_refs, Float32MultiArray &input_refs, Float32MultiArray &attributes, Float32MultiArray &normals);

  private:
	void precompute_path();
	void construct_path(bool has_first);
	static void saveTxt(const std_msgs::Float32MultiArray &msg, const std::string &filename, std::size_t elemsPerRow = 0, int precision = 6, const std::string &delim = " ") {
		const auto &data = msg.data;
		const std::size_t N = data.size();
		if (N == 0)
			throw std::runtime_error("io::saveTxt – message contains no data");

		// If the caller didn’t supply a row length, try to infer it from the layout.
		if (elemsPerRow == 0) {
			if (!msg.layout.dim.empty())
				elemsPerRow = msg.layout.dim.front().size;
			else
				elemsPerRow = N; // one giant row
		}
		if (elemsPerRow == 0 || N % elemsPerRow != 0)
			throw std::runtime_error("io::saveTxt – invalid elemsPerRow for data size");

		std::ofstream file(filename, std::ios::trunc);
		if (!file)
			throw std::runtime_error("io::saveTxt – cannot open " + filename);

		file << std::fixed << std::setprecision(precision);

		for (std::size_t i = 0; i < N; ++i) {
			file << data[i];
			bool endOfRow = ((i + 1) % elemsPerRow == 0);
			file << (endOfRow ? std::string("\n") : delim);
		}
		file.flush();
	}
};
