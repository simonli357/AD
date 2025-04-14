#include "Track.hpp"

class SplineUtils {
  public:
	SplineUtils() = default;
	SplineUtils(SplineUtils &&) = default;
	SplineUtils(const SplineUtils &) = default;
	SplineUtils &operator=(SplineUtils &&) = default;
	SplineUtils &operator=(const SplineUtils &) = default;
	~SplineUtils() = default;

	std::vector<Track::Vertex> interpolate_path(const std::vector<Track::Vertex> &path, int density);
	void plot_path(const std::vector<Track::Vertex> &original, const std::vector<Track::Vertex> &smoothed);
};
