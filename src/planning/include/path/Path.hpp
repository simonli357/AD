#pragma once

#include "SplineUtils.hpp"
#include "Track.hpp"

class Path {
  public:
	Path();
	Path(Path &&) = default;
	Path(const Path &) = default;
	Path &operator=(Path &&) = default;
	Path &operator=(const Path &) = default;
	~Path() = default;

	using Vertex = Track::Vertex;

	Track track;
	SplineUtils spline_utils;

	double vref;
	int N;
	int T;
	std::string name;
	Vertex start;
	std::vector<Vertex> destinations;

	void set_constraints(double vref, int N, int T, std::string name, Vertex start, std::vector<Vertex> destinations);

	void set_N(double N) { this->N = N; }
	void set_T(double T) { this->T = T; }
	void set_vref(double vref) { this->vref = vref; }
	void set_name(std::string name) { this->name = name; }
	void set_start(Vertex start) { this->start = start; }
	void set_destinations(std::vector<Vertex> destinations) { this->destinations = destinations; }

	void plan_path(std::vector<double> &state_refs, std::vector<double> &input_refs, std::vector<double> &attributes, std::vector<double> &normals);

  private:
};
