#pragma once

#include "map/Track.hpp"

class SplineUtils {
  public:
	SplineUtils() = default;
	SplineUtils(SplineUtils &&) = default;
	SplineUtils(const SplineUtils &) = default;
	SplineUtils &operator=(SplineUtils &&) = default;
	SplineUtils &operator=(const SplineUtils &) = default;
	~SplineUtils() = default;

	using Vertex = Track::Vertex;

	std::vector<Vertex> interpolate_path(const std::vector<Vertex> &path, int density);
	void plot_path(const std::vector<Vertex> &original, const std::vector<Vertex> &smoothed);
};
