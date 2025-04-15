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
	using ATTRIBUTE = Track::ATTRIBUTE;

	std::vector<Vertex> interpolate_path(const std::vector<Vertex> &path, double density, double hw_density_factor, double cw_density_factor, double smooth_factor = 0.1);

  private:
	std::vector<Vertex> smooth_path(const std::vector<Vertex> &path, double density, ATTRIBUTE attribute, double smooth_factor);
};
