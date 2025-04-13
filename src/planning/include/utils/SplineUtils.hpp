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

  private:
	std::vector<double> build_query_normalized(size_t numPoints, int density);
};
