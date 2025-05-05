#include "utils/SplineUtils.hpp"
#include "interpolation.h"
#include "map/Track.hpp"
#include <cmath>
#include <vector>

using Vertex = Track::Vertex;
using Edge = Track::Edge;
using Graph = Track::Graph;
using ATTRIBUTE = Track::ATTRIBUTE;
using VD = boost::graph_traits<Graph>::vertex_descriptor;

std::vector<Vertex> SplineUtils::interpolate_path(const std::vector<Vertex> &path, double density, double hw_density_factor, double cw_density_factor, double smooth_factor) {
	if (path.size() <= 1 || density <= 0.0) {
		return path;
	}

	const int n = static_cast<int>(path.size());

	// 1) Compute cumulative "t" based on the original polyline lengths
	std::vector<double> t(n, 0.0);
	for (int i = 1; i < n; ++i) {
		double dx = path[i].x - path[i - 1].x;
		double dy = path[i].y - path[i - 1].y;
		t[i] = t[i - 1] + std::sqrt(dx * dx + dy * dy);
	}

	// 2) Build weights (for intersections, special IDs, etc.)
	std::vector<double> weights(n, 1.0);
	for (int i = 0; i < n; ++i) {
		switch (path[i].attribute) {
		case Track::INTERSECTION:
			weights[i] = 1.0;
			break;
		default:
			break;
		}
		switch (path[i].id) {
		case 206:
		case 208:
			weights[i] = 1.0;
			break;
		default:
			break;
		}
	}

	// 3) Copy into ALGLIB arrays and fit penalized smoothing splines
	alglib::real_1d_array t_arr, x_arr, y_arr, w_arr;
	t_arr.setlength(n);
	x_arr.setlength(n);
	y_arr.setlength(n);
	w_arr.setlength(n);
	for (int i = 0; i < n; ++i) {
		t_arr[i] = t[i];
		x_arr[i] = path[i].x;
		y_arr[i] = path[i].y;
		w_arr[i] = weights[i];
	}

	alglib::spline1dinterpolant spline_x, spline_y;
	alglib::ae_int_t info_x = 0, info_y = 0;
	alglib::spline1dfitreport rep_x, rep_y;
	alglib::spline1dfitpenalizedw(t_arr, x_arr, w_arr, n, smooth_factor, info_x, spline_x, rep_x);
	alglib::spline1dfitpenalizedw(t_arr, y_arr, w_arr, n, smooth_factor, info_y, spline_y, rep_y);

	std::vector<Vertex> result;
	const int INTEGRATION_STEPS = 64;

	// 4) Helper lambda: approximate ∫|r'(t)| dt via composite trapezoid rule
	auto arcLengthSegment = [&](double ta, double tb) {
		double h = (tb - ta) / INTEGRATION_STEPS;

		// Evaluate speed at ta
		double x0, dx0, d2x0, y0, dy0, d2y0;
		alglib::spline1ddiff(spline_x, ta, x0, dx0, d2x0);
		alglib::spline1ddiff(spline_y, ta, y0, dy0, d2y0);
		double v0 = std::hypot(dx0, dy0);

		double sum = 0.0;
		for (int j = 1; j <= INTEGRATION_STEPS; ++j) {
			double tj = ta + j * h;
			alglib::spline1ddiff(spline_x, tj, x0, dx0, d2x0);
			alglib::spline1ddiff(spline_y, tj, y0, dy0, d2y0);
			double v1 = std::hypot(dx0, dy0);
			sum += 0.5 * (v0 + v1) * h;
			v0 = v1;
		}
		return sum;
	};

	// 5) For each original segment, invert arc-length → t by bisection
	for (int i = 0; i < n - 1; ++i) {
		double t0 = t[i];
		double t1 = t[i + 1];
		double segLen = arcLengthSegment(t0, t1);

		// Compute effective density per-segment
		double eff_density = segLen * density * 0.9;
		switch (path[i].attribute) {
		case ATTRIBUTE::CROSSWALK:
			eff_density *= cw_density_factor;
			break;
		case ATTRIBUTE::HIGHWAY_LEFT:
		case ATTRIBUTE::HIGHWAY_RIGHT:
			eff_density /= hw_density_factor;
			break;
		default:
			break;
		}
		int localSteps = std::max(1, static_cast<int>(std::round(eff_density)));

		// Invert arc-length for each sample
		for (int step = 0; step < localSteps; ++step) {
			double s_target = (step * segLen) / localSteps;

			double a = t0, b = t1, m = 0.5 * (a + b);
			for (int iter = 0; iter < 20; ++iter) {
				m = 0.5 * (a + b);
				double s_mid = arcLengthSegment(t0, m);
				if (s_mid < s_target)
					a = m;
				else
					b = m;
			}
			double t_val = 0.5 * (a + b);

			// Evaluate spline & its derivatives at t_val
			double x_val, dx, d2x, y_val, dy, d2y;
			alglib::spline1ddiff(spline_x, t_val, x_val, dx, d2x);
			alglib::spline1ddiff(spline_y, t_val, y_val, dy, d2y);

			// Build the output vertex
			Vertex v;
			v.x = x_val;
			v.y = y_val;
			v.tangent_angle = std::atan2(dy, dx);
			v.normal_angle = v.tangent_angle + M_PI / 2.0;
			double speed = std::hypot(dx, dy);
			v.curvature = (speed > 1e-8) ? std::abs(dx * d2y - dy * d2x) / (speed * speed * speed) : 0.0;
			v.attribute = path[i].attribute;

			result.push_back(v);
		}
	}

	return result;
}
