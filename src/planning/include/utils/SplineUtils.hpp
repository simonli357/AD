#include "Track.hpp"

class SplineUtils {
  public:
	SplineUtils() = default;
	SplineUtils(SplineUtils &&) = default;
	SplineUtils(const SplineUtils &) = default;
	SplineUtils &operator=(SplineUtils &&) = default;
	SplineUtils &operator=(const SplineUtils &) = default;
	~SplineUtils() = default;

	std::vector<Track::Vertex> interpolate_path(const std::vector<Track::Vertex> &path, int density, double rho = 0.1);
	void plot_path(const std::vector<Track::Vertex> &original, const std::vector<Track::Vertex> &smoothed);

  private:
	std::vector<double> compute_chord_parameters(const std::vector<Track::Vertex> &path);
};
