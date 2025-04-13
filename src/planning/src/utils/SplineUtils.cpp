#include "SplineUtils.hpp"
#include "Track.hpp"
#include <Eigen/Core>
#include <interpolation.h>
#include <matplot/matplot.h>

using namespace Eigen;
using namespace alglib;

std::vector<double> SplineUtils::compute_chord_parameters(const std::vector<Track::Vertex> &path) {
	std::vector<double> t(path.size(), 0.0);
	for (size_t i = 1; i < path.size(); ++i) {
		double dx = path[i].x - path[i - 1].x;
		double dy = path[i].y - path[i - 1].y;
		t[i] = t[i - 1] + std::hypot(dx, dy);
	}
	// Normalize to [0, 1]
	double total_length = t.back();
	if (total_length > 0) {
		for (auto &val : t)
			val /= total_length;
	}
	return t;
}

std::vector<Track::Vertex> SplineUtils::interpolate_path(const std::vector<Track::Vertex> &path, int density, double rho) {
	if (path.size() < 2)
		return {};

	// 1. Parameterize points using chord-length
	auto t_params = compute_chord_parameters(path);

	// 2. Prepare ALGLIB input arrays
	real_1d_array t, x, y;
	t.setlength(path.size());
	x.setlength(path.size());
	y.setlength(path.size());

	for (size_t i = 0; i < path.size(); ++i) {
		t[i] = t_params[i];
		x[i] = path[i].x;
		y[i] = path[i].y;
	}

	// 3. Fit smoothing splines for X and Y coordinates
	spline1dinterpolant spline_x, spline_y;
	ae_int_t info;
	spline1dfitreport rep;

	// Same smoothing factor for X and Y
	spline1dfit(t, x, path.size(), rho, info, spline_x, rep);
	spline1dfit(t, y, path.size(), rho, info, spline_y, rep);

	if (info != 1) { // ALGLIB error check
		std::cerr << "Spline fitting failed!" << std::endl;
		return {};
	}

	// 4. Generate dense query points
	std::vector<double> query_t;
	size_t num_segments = path.size() - 1;
	query_t.reserve(num_segments * density);
	for (size_t i = 0; i < num_segments; ++i) {
		double t_start = t_params[i];
		double t_end = t_params[i + 1];
		for (int j = 0; j < density; ++j) {
			double t = t_start + (t_end - t_start) * j / density;
			query_t.push_back(t);
		}
	}

	// 5. Evaluate splines at query points
	std::vector<Track::Vertex> result;
	for (double t : query_t) {
		Track::Vertex v;
		v.x = spline1dcalc(spline_x, t);
		v.y = spline1dcalc(spline_y, t);

		// Find original segment for attribute
		size_t seg = 0;
		while (seg < path.size() - 1 && t_params[seg + 1] < t)
			++seg;
		v.attribute = path[seg].attribute;

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
	auto p2 = plot(sx, sy)->line_width(2).color("blue");
	show();
}
