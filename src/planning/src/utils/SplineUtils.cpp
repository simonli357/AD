#include "utils/SplineUtils.hpp"
#include "interpolation.h"
#include "map/Track.hpp"
#include <algorithm>
#include <cmath>
#include <vector>

using Vertex = Track::Vertex;
using ATTRIBUTE = Track::ATTRIBUTE;

std::vector<Vertex> SplineUtils::interpolate_path(const std::vector<Vertex> &path, int density, double smooth_factor) {
	if (path.size() <= 1 || density <= 0) {
		return path;
	}

	const int n = path.size();

	std::vector<double> t(n, 0.0);
	for (int i = 1; i < n; ++i) {
		double dx = path[i].x - path[i - 1].x;
		double dy = path[i].y - path[i - 1].y;
		double dist = std::sqrt(dx * dx + dy * dy);
		t[i] = t[i - 1] + dist;
	}

	alglib::real_1d_array t_arr, x_arr, y_arr;
	t_arr.setlength(n);
	x_arr.setlength(n);
	y_arr.setlength(n);
	for (int i = 0; i < n; ++i) {
		t_arr[i] = t[i];
		x_arr[i] = path[i].x;
		y_arr[i] = path[i].y;
	}

	alglib::spline1dinterpolant spline_x;
	alglib::spline1dinterpolant spline_y;
	alglib::ae_int_t info_x = 0;
	alglib::ae_int_t info_y = 0;
	alglib::spline1dfitreport rep_x;
	alglib::spline1dfitreport rep_y;
	alglib::spline1dfitpenalized(t_arr, x_arr, n, smooth_factor, info_x, spline_x, rep_x);
	alglib::spline1dfitpenalized(t_arr, y_arr, n, smooth_factor, info_y, spline_y, rep_y);

	int totalSteps = (n - 1) * density;
	std::vector<Vertex> result;
	result.reserve(totalSteps + 1);
	double t_min = t.front();
	double t_max = t.back();

	for (int step = 0; step <= totalSteps; ++step) {
		double alpha = static_cast<double>(step) / static_cast<double>(totalSteps);
		double t_val = t_min + alpha * (t_max - t_min);

		double x_val, dx, d2x;
		alglib::spline1ddiff(spline_x, t_val, x_val, dx, d2x);

		double y_val, dy, d2y;
		alglib::spline1ddiff(spline_y, t_val, y_val, dy, d2y);

		Vertex v;
		v.x = x_val;
		v.y = y_val;
		// Compute the tangent angle using the first derivatives.
		v.tangent_angle = std::atan2(dy, dx);
		v.normal_angle = v.tangent_angle + M_PI / 2.0;

		// curvature = |dx*d2y - dy*d2x| / (sqrt(dx^2 + dy^2)^3)
		double speed = std::sqrt(dx * dx + dy * dy);
		if (speed > 1e-8) {
			v.curvature = std::abs(dx * d2y - dy * d2x) / (speed * speed * speed);
		} else {
			v.curvature = 0.0;
		}

		// Determine the original segment this interpolated point belongs to.
		int seg_idx = 0;
		for (int i = 0; i < n - 1; ++i) {
			if (t_val >= t[i] && t_val <= t[i + 1]) {
				seg_idx = i;
				break;
			}
		}
		seg_idx = std::max(0, std::min(seg_idx, n - 1));
		v.attribute = path[seg_idx].attribute;

		result.push_back(v);
	}
	return result;
}
