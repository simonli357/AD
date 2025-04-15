#include "utils/SplineUtils.hpp"
#include "interpolation.h"
#include "map/Track.hpp"
#include <cmath>
#include <vector>

using Vertex = Track::Vertex;
using ATTRIBUTE = Track::ATTRIBUTE;

std::vector<Vertex> SplineUtils::interpolate_path(const std::vector<Vertex> &path, double density, double hw_density_factor, double cw_density_factor, double smooth_factor) {
	if (path.size() <= 1 || density <= 0) {
		return path;
	}

	const int n = path.size();

	// Compute cumulative arc-length for the whole path.
	std::vector<double> t(n, 0.0);
	for (int i = 1; i < n; ++i) {
		double dx = path[i].x - path[i - 1].x;
		double dy = path[i].y - path[i - 1].y;
		double dist = std::sqrt(dx * dx + dy * dy);
		t[i] = t[i - 1] + dist;
	}

	// Prepare arrays for spline fitting.
	alglib::real_1d_array t_arr, x_arr, y_arr;
	t_arr.setlength(n);
	x_arr.setlength(n);
	y_arr.setlength(n);
	for (int i = 0; i < n; ++i) {
		t_arr[i] = t[i];
		x_arr[i] = path[i].x;
		y_arr[i] = path[i].y;
	}

	// Fit penalized splines to x and y components.
	alglib::spline1dinterpolant spline_x, spline_y;
	alglib::ae_int_t info_x = 0, info_y = 0;
	alglib::spline1dfitreport rep_x, rep_y;
	alglib::spline1dfitpenalized(t_arr, x_arr, n, smooth_factor, info_x, spline_x, rep_x);
	alglib::spline1dfitpenalized(t_arr, y_arr, n, smooth_factor, info_y, spline_y, rep_y);

	std::vector<Vertex> result;

	// Process each segment separately.
	for (int i = 0; i < n - 1; ++i) {
		double t_start = t[i];
		double t_end = t[i + 1];
		double seg_length = t_end - t_start;
		if (seg_length <= 0)
			continue; // Avoid degenerate segments.

		// Determine effective density for this segment based on its attribute.
		double effective_density = seg_length * density;
        switch (path[i].attribute) {
            case ATTRIBUTE::CROSSWALK:
                effective_density = seg_length * density * cw_density_factor;
                break;
            case ATTRIBUTE::HIGHWAY_LEFT:
                effective_density = seg_length * density / hw_density_factor;
                break;
            case ATTRIBUTE::HIGHWAY_RIGHT:
                effective_density = seg_length * density / hw_density_factor;
                break;
            default:
                break;
        }

		if (path[i].attribute == ATTRIBUTE::HIGHWAY_LEFT || path[i].attribute == ATTRIBUTE::HIGHWAY_RIGHT) {
			effective_density /= hw_density_factor;
		} else if (path[i].attribute == ATTRIBUTE::CROSSWALK) {
			effective_density *= cw_density_factor;
		}

		// Compute number of interpolation steps for this segment.
		int localSteps = static_cast<int>(std::ceil(seg_length * effective_density));
		// Ensure at least one sample per segment.
		localSteps = std::max(1, localSteps);

		// Interpolate within the segment.
		for (int step = 0; step < localSteps; ++step) {
			double alpha = static_cast<double>(step) / localSteps;
			double t_val = t_start + alpha * (t_end - t_start);

			double x_val, dx, d2x;
			alglib::spline1ddiff(spline_x, t_val, x_val, dx, d2x);

			double y_val, dy, d2y;
			alglib::spline1ddiff(spline_y, t_val, y_val, dy, d2y);

			Vertex v;
			v.x = x_val;
			v.y = y_val;
			v.tangent_angle = std::atan2(dy, dx);
			v.normal_angle = v.tangent_angle + M_PI / 2.0;

			double speed = std::sqrt(dx * dx + dy * dy);
			v.curvature = (speed > 1e-8) ? std::abs(dx * d2y - dy * d2x) / (speed * speed * speed) : 0.0;

			// Assign the attribute for the interpolated point.
			// The attribute is taken from the segment's starting vertex.
			v.attribute = path[i].attribute;

			result.push_back(v);
		}
	}

	// Add the last vertex explicitly to ensure the full path is covered.
	result.push_back(path.back());
	return result;
}
