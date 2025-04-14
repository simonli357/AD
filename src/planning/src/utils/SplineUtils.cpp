#include "SplineUtils.hpp"
#include "Track.hpp"
#include <Eigen/Core>
#include <cmath>
#include <matplot/matplot.h>
#include <unsupported/Eigen/Splines>

using namespace Eigen;
using Vertex = Track::Vertex;
using ATTRIBUTE = Track::ATTRIBUTE;
using Spline2d = Spline<double, 2>;

std::vector<Track::Vertex> SplineUtils::interpolate_path(const std::vector<Track::Vertex> &path, int density) {
	if (path.size() <= 1 || density <= 0) {
		return path;
	}

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

	// Precompute the parameter `t` values for each original vertex
	std::vector<double> t_values;
	for (size_t i = 0; i < path.size(); ++i) {
		t_values.push_back(knotVector(degree + i));
	}

	std::vector<Track::Vertex> result;
	int totalSteps = (static_cast<int>(path.size()) - 1) * density;

	result.reserve(totalSteps + 1);

	for (int step = 0; step <= totalSteps; step++) {
		double alpha = static_cast<double>(step) / static_cast<double>(totalSteps);
		double t = t_min + alpha * (t_max - t_min);

		Eigen::Matrix<double, 2, 1> val = spline(t);

		auto derivatives = spline.derivatives(t, 1);
		double dx_dt = derivatives(0, 1);
		double dy_dt = derivatives(1, 1);

		Track::Vertex v;
		v.x = val(0);
		v.y = val(1);
		v.yaw = std::atan2(dy_dt, dx_dt);

		// Find the original segment this interpolated point belongs to
		auto it = std::upper_bound(t_values.begin(), t_values.end(), t);
		size_t segment_idx = std::distance(t_values.begin(), it) - 1;

		// Clamp to valid indices
		segment_idx = std::min(segment_idx, path.size() - 1);
		segment_idx = std::max(segment_idx, static_cast<size_t>(0));

		// Assign the key from the starting vertex of the segment
		v.attribute = path[segment_idx].attribute;

		result.push_back(v);
	}

	return result;
}

void SplineUtils::plot_path(const std::vector<Vertex> &original, const std::vector<Vertex> &smoothed) {
	using namespace matplot;

	// Extract original and smoothed coordinates
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

	if (!smoothed.empty()) {
		// Map each attribute to a distinct color
		auto get_color = [](ATTRIBUTE attr) {
			switch (attr) {
			case ATTRIBUTE::NORMAL:
				return "blue";
			case ATTRIBUTE::CROSSWALK:
				return "black";
			case ATTRIBUTE::INTERSECTION:
				return "green";
			case ATTRIBUTE::ONEWAY:
				return "orange";
			case ATTRIBUTE::HIGHWAY_LEFT:
				return "red";
			case ATTRIBUTE::HIGHWAY_RIGHT:
				return "purple";
			case ATTRIBUTE::ROUNDABOUT:
				return "cyan";
			case ATTRIBUTE::STOPLINE:
				return "magenta";
			case ATTRIBUTE::DOTTED:
				return "yellow";
			case ATTRIBUTE::DOTTED_CROSSWALK:
				return "gray";
			default:
				return "blue";
			}
		};

		// Plot path segments with color coding
		size_t segment_start = 0;
		ATTRIBUTE current_attr = smoothed[0].attribute;

		for (size_t i = 1; i < smoothed.size(); ++i) {
			if (smoothed[i].attribute != current_attr) {
				if (i - segment_start >= 1) {
					std::vector<double> seg_x(sx.begin() + segment_start, sx.begin() + i);
					std::vector<double> seg_y(sy.begin() + segment_start, sy.begin() + i);
					auto line = plot(seg_x, seg_y);
					line->color(get_color(current_attr)).line_width(1);
				}
				segment_start = i;
				current_attr = smoothed[i].attribute;
			}
		}

		// Plot last segment
		if (smoothed.size() - segment_start >= 1) {
			std::vector<double> seg_x(sx.begin() + segment_start, sx.end());
			std::vector<double> seg_y(sy.begin() + segment_start, sy.end());
			auto line = plot(seg_x, seg_y);
			line->color(get_color(current_attr)).line_width(1);
		}

		// Add yaw arrows for every 8th point in smoothed path
		const int arrow_step = 8;
		const double arrow_length = 0.6;
		for (size_t i = 0; i < smoothed.size(); i += arrow_step) {
			const auto &v = smoothed[i];
			double dx = arrow_length * cos(v.yaw);
			double dy = arrow_length * sin(v.yaw);

			// Use line plot with arrowhead
			auto arr = arrow(v.x, v.y, v.x + dx, v.y + dy);
			arr->color("crimson").line_width(0.5);
		}

		// Highlight start and end points
		std::vector<double> start_x = {smoothed[0].x};
		std::vector<double> start_y = {smoothed[0].y};
		auto start_marker = scatter(start_x, start_y);
		start_marker->marker_face_color("limegreen").marker_size(25);

		std::vector<double> end_x = {smoothed.back().x};
		std::vector<double> end_y = {smoothed.back().y};
		auto end_marker = scatter(end_x, end_y);
		end_marker->marker_face_color("orangered").marker_size(25);
	}

	title("Path Visualization with Yaw Directions");
	xlabel("X Coordinate");
	ylabel("Y Coordinate");
	grid(true);
	axis(equal);
	show();
}
