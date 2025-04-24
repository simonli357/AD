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

/* std::vector<Vertex> SplineUtils::interpolate_path(const std::vector<Vertex> &path, double density, double hw_density_factor, double cw_density_factor, double smooth_factor) { */
/* 	if (path.size() <= 1 || density <= 0) { */
/* 		return path; */
/* 	} */

/* 	const int n = path.size(); */

/* 	// Compute cumulative arc-length for the whole path. */
/* 	std::vector<double> t(n, 0.0); */
/* 	for (int i = 1; i < n; ++i) { */
/* 		double dx = path[i].x - path[i - 1].x; */
/* 		double dy = path[i].y - path[i - 1].y; */
/* 		double dist = std::sqrt(dx * dx + dy * dy); */
/* 		t[i] = t[i - 1] + dist; */
/* 	} */

/* 	std::vector<double> weights(n, 1.0); */
/* 	for (int i = 0; i < n; ++i) { */
/* 		// Manually adjust some weights */
/* 		switch (path[i].attribute) { */
/* 		case Track::INTERSECTION: */
/* 			weights[i] = 0.0; */
/* 			break; */
/* 		default: */
/* 			break; */
/* 		} */
/* 		switch (path[i].id) { */
/* 		case 206: */
/* 		case 208: */
/* 			weights[i] = 1.0; */
/* 			break; */
/* 		default: */
/* 			break; */
/* 		} */
/* 	} */

/* 	// Prepare arrays for spline fitting including weights */
/* 	alglib::real_1d_array t_arr, x_arr, y_arr, w_arr; */
/* 	t_arr.setlength(n); */
/* 	x_arr.setlength(n); */
/* 	y_arr.setlength(n); */
/* 	w_arr.setlength(n); */
/* 	for (int i = 0; i < n; ++i) { */
/* 		t_arr[i] = t[i]; */
/* 		x_arr[i] = path[i].x; */
/* 		y_arr[i] = path[i].y; */
/* 		w_arr[i] = weights[i]; */
/* 	} */

/* 	// Fit penalized splines with weights */
/* 	alglib::spline1dinterpolant spline_x, spline_y; */
/* 	alglib::ae_int_t info_x = 0, info_y = 0; */
/* 	alglib::spline1dfitreport rep_x, rep_y; */
/* 	alglib::spline1dfitpenalizedw(t_arr, x_arr, w_arr, n, smooth_factor, info_x, spline_x, rep_x); */
/* 	alglib::spline1dfitpenalizedw(t_arr, y_arr, w_arr, n, smooth_factor, info_y, spline_y, rep_y); */

/* 	std::vector<Vertex> result; */

/* 	// Process each segment separately (existing code remains unchanged) */
/* 	for (int i = 0; i < n - 1; ++i) { */
/* 		double t_start = t[i]; */
/* 		double t_end = t[i + 1]; */
/* 		double seg_length = t_end - t_start; */
/* 		if (seg_length <= 0) { */
/* 			continue; */
/* 		} */

/* 		// Determine effective density (existing code remains unchanged) */
/* 		double effective_density = seg_length * density; */
/* 		switch (path[i].attribute) { */
/* 		case ATTRIBUTE::CROSSWALK: */
/* 			effective_density = seg_length * density * cw_density_factor; */
/* 			break; */
/* 		case ATTRIBUTE::HIGHWAY_LEFT: */
/* 		case ATTRIBUTE::HIGHWAY_RIGHT: */
/* 			effective_density = seg_length * density / hw_density_factor; */
/* 			break; */
/* 		default: */
/* 			break; */
/* 		} */

/* 		int localSteps = std::max(1, static_cast<int>(std::round(effective_density))); */

/* 		// Interpolate within the segment (existing code remains unchanged) */
/* 		for (int step = 0; step < localSteps; ++step) { */
/* 			double alpha = static_cast<double>(step) / localSteps; */
/* 			double t_val = t_start + alpha * (t_end - t_start); */

/* 			double x_val, dx, d2x; */
/* 			alglib::spline1ddiff(spline_x, t_val, x_val, dx, d2x); */

/* 			double y_val, dy, d2y; */
/* 			alglib::spline1ddiff(spline_y, t_val, y_val, dy, d2y); */

/* 			Vertex v; */
/* 			v.x = x_val; */
/* 			v.y = y_val; */
/* 			v.tangent_angle = std::atan2(dy, dx); */
/* 			v.normal_angle = v.tangent_angle + M_PI / 2.0; */

/* 			double speed = std::sqrt(dx * dx + dy * dy); */
/* 			v.curvature = (speed > 1e-8) ? std::abs(dx * d2y - dy * d2x) / (speed * speed * speed) : 0.0; */
/* 			v.attribute = path[i].attribute; */

/* 			result.push_back(v); */
/* 		} */
/* 	} */
/* 	return result; */
/* } */

std::vector<Vertex> resample_by_distance(const std::vector<Vertex> &in, double ds) {
	std::vector<Vertex> out;
	if (in.empty())
		return out;
	out.reserve(in.size());
	out.push_back(in.front());

	double acc = 0.0;
	for (size_t i = 1; i < in.size(); ++i) {
		const auto &P0 = in[i - 1];
		const auto &P1 = in[i];
		double dx = P1.x - P0.x;
		double dy = P1.y - P0.y;
		double seg = std::hypot(dx, dy);
		if (seg < 1e-8)
			continue;

		double ux = dx / seg;
		double uy = dy / seg;
		double remain = seg;

		// Step along this segment in increments of "ds"
		while (acc + remain >= ds) {
			double offset = ds - acc;
			Vertex V;
			// Start from last output point
			const Vertex &L = out.back();
			V.x = L.x + ux * offset;
			V.y = L.y + uy * offset;
			V.tangent_angle = std::atan2(uy, ux);
			V.normal_angle = V.tangent_angle + M_PI / 2.0;
			V.attribute = P0.attribute;
			// curvature and speed to be re-computed downstream
			out.push_back(V);
			remain -= offset;
			acc = 0.0;
		}
		acc += remain;
	}
	return out;
}

std::vector<Vertex> SplineUtils::interpolate_path(const std::vector<Vertex> &path, double density, double hw_density_factor, double cw_density_factor, double smooth_factor) {
	if (path.size() <= 1 || density <= 0) {
		return path;
	}

	const int n = (int)path.size();
	// 1) Build cumulative chord-length t
	std::vector<double> t(n, 0.0);
	for (int i = 1; i < n; ++i) {
		double dx = path[i].x - path[i - 1].x;
		double dy = path[i].y - path[i - 1].y;
		t[i] = t[i - 1] + std::hypot(dx, dy);
	}

	// 2) Compute weights (e.g. intersection = 0)
	std::vector<double> weights(n, 1.0);
	for (int i = 0; i < n; ++i) {
		if (path[i].attribute == ATTRIBUTE::INTERSECTION)
			weights[i] = 0.0;
		// override specific IDs if needed:
		if (path[i].id == 206 || path[i].id == 208)
			weights[i] = 1.0;
	}

	// 3) Prepare arrays for alglib
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

	// 4) Fit penalized splines
	alglib::spline1dinterpolant sx, sy;
	alglib::ae_int_t info_x, info_y;
	alglib::spline1dfitreport repx, repy;
	alglib::spline1dfitpenalizedw(t_arr, x_arr, w_arr, n, smooth_factor, info_x, sx, repx);
	alglib::spline1dfitpenalizedw(t_arr, y_arr, w_arr, n, smooth_factor, info_y, sy, repy);

	// 5) Parametric sampling (raw points)
	std::vector<Vertex> raw;
	raw.reserve((n - 1) * 10);
	for (int i = 0; i < n - 1; ++i) {
		double t0 = t[i], t1 = t[i + 1];
		double seg = t1 - t0;
		if (seg <= 0)
			continue;

		// choose # of steps via round instead of ceil
		double eff_den = seg * density;
		if (path[i].attribute == ATTRIBUTE::CROSSWALK)
			eff_den *= cw_density_factor;
		if (path[i].attribute == ATTRIBUTE::HIGHWAY_LEFT || path[i].attribute == ATTRIBUTE::HIGHWAY_RIGHT)
			eff_den /= hw_density_factor;
		int steps = std::max(1, (int)std::round(eff_den));

		for (int s = 0; s <= steps; ++s) {
			double alpha = double(s) / double(steps);
			double tv = t0 + alpha * seg;
			double x, dx, d2x;
			double y, dy, d2y;
			alglib::spline1ddiff(sx, tv, x, dx, d2x);
			alglib::spline1ddiff(sy, tv, y, dy, d2y);

			Vertex v;
			v.x = x;
			v.y = y;
			v.tangent_angle = std::atan2(dy, dx);
			v.normal_angle = v.tangent_angle + M_PI / 2.0;
			double speed = std::hypot(dx, dy);
			v.curvature = (speed > 1e-8 ? std::abs(dx * d2y - dy * d2x) / (speed * speed * speed) : 0.0);
			v.attribute = path[i].attribute;
			raw.push_back(v);
		}
	}

	// 6) Re-sample by true arc-length
	double desired_ds = 1.0 / density;
	auto final_pts = resample_by_distance(raw, desired_ds);

	return final_pts;
}
