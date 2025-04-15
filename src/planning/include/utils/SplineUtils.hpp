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

	std::vector<Vertex> interpolate_path(const std::vector<Vertex> &path, double density, double smooth_factor=0.1);
};
