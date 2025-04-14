#include "SplineUtils.hpp"
#include "Track.hpp"
#include <Eigen/Core>
#include <matplot/matplot.h>
#include <unsupported/Eigen/Splines>

using namespace Eigen;
using Spline2d = Spline<double, 2>;

std::vector<Track::Vertex> SplineUtils::interpolate_path(const std::vector<Track::Vertex> &path, int density) {
	/*
	 * Given a list of vertices forming a path, use Eigen spline interpolation to increase the
	 * path density by computing additional vertices.
	 */
	if (path.size() <= 1 || density <= 0) {
		return path;
	}

	// Change the dimension: create a 2 x N matrix.
	Matrix<double, 2, Dynamic> points(2, static_cast<int>(path.size()));
	for (size_t i = 0; i < path.size(); i++) {
		points(0, i) = path[i].x;
		points(1, i) = path[i].y;
	}

	const int degree = 3;
	Spline2d spline = SplineFitting<Spline2d>::Interpolate(points, degree);

	const auto &knotVector = spline.knots();
	double t_min = knotVector(degree);
	double t_max = knotVector(knotVector.size() - degree - 1);

	std::vector<Track::Vertex> result;
	int totalSteps = (static_cast<int>(path.size()) - 1) * density;

	result.reserve(totalSteps + 1);

	for (int i = 0; i <= totalSteps; i++) {
		double alpha = static_cast<double>(i) / static_cast<double>(totalSteps);
		double t = t_min + alpha * (t_max - t_min);

		Eigen::Matrix<double, 2, 1> val = spline(t);

		Track::Vertex v;
		v.x = val(0);
		v.y = val(1);
		result.push_back(v);
	}

	return result;
}

void SplineUtils::plot_path(const std::vector<Track::Vertex> &original, const std::vector<Track::Vertex> &smoothed) {
	using namespace matplot;

	std::vector<double> ox, oy, sx, sy;
	for (const auto &v : original) {
		ox.push_back(v.x);
		oy.push_back(v.y);
	}
	for (const auto &v : smoothed) {
		sx.push_back(v.x);
		sy.push_back(v.y);
	}

	figure();
	auto p1 = scatter(ox, oy);
	p1->marker_face_color("red").marker_size(8);
	hold(on);
	auto p2 = plot(sx, sy)->line_width(1).color("blue");
	show();
}
