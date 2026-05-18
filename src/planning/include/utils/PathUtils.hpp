#include "map/Track.hpp"

class PathUtils {
  public:
	PathUtils() = default;
	PathUtils(PathUtils &&) = default;
	PathUtils(const PathUtils &) = default;
	PathUtils &operator=(PathUtils &&) = default;
	PathUtils &operator=(const PathUtils &) = default;
	~PathUtils() = default;

	using Vertex = Track::Vertex;

	void normalize_yaw(std::vector<Vertex> &path, double max_yaw_change);
	void smooth_yaw(std::vector<Vertex> &path);
	void distance_filter(std::vector<Vertex> &path, double thresh, double hw_speed_ratio, double cw_density_factor);
	void compute_speeds(std::vector<Vertex> &path, double vref, double density, double hw_speed_ratio, double cw_density_factor);
	double euclidean_distance(const Vertex &src, const Vertex &dest);
};
