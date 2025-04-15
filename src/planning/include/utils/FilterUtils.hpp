#include "map/Track.hpp"

class FilterUtils {
  public:
	FilterUtils() = default;
	FilterUtils(FilterUtils &&) = default;
	FilterUtils(const FilterUtils &) = default;
	FilterUtils &operator=(FilterUtils &&) = default;
	FilterUtils &operator=(const FilterUtils &) = default;
	~FilterUtils() = default;

	using Vertex = Track::Vertex;

	void yaw_filter(std::vector<Vertex> &path, double max_yaw_change);
	void distance_filter(std::vector<Vertex> &path, double thresh);
	double euclidean_distance(const Vertex &src, const Vertex &dest);
};
