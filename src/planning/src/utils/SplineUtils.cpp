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

	std::vector<double> weights(n, 1.0);
	for (int i = 0; i < n; ++i) {
		// Manually adjust some weights
		switch (path[i].attribute) {
		case Track::INTERSECTION:
			weights[i] = 0.0;
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

	// Prepare arrays for spline fitting including weights
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

	// Fit penalized splines with weights
	alglib::spline1dinterpolant spline_x, spline_y;
	alglib::ae_int_t info_x = 0, info_y = 0;
	alglib::spline1dfitreport rep_x, rep_y;
	alglib::spline1dfitpenalizedw(t_arr, x_arr, w_arr, n, smooth_factor, info_x, spline_x, rep_x);
	alglib::spline1dfitpenalizedw(t_arr, y_arr, w_arr, n, smooth_factor, info_y, spline_y, rep_y);

	std::vector<Vertex> result;

	// Process each segment separately (existing code remains unchanged)
	for (int i = 0; i < n - 1; ++i) {
		double t_start = t[i];
		double t_end = t[i + 1];
		double seg_length = t_end - t_start;
		if (seg_length <= 0) {
			continue;
		}

		// Determine effective density (existing code remains unchanged)
		double effective_density = seg_length * density;
		switch (path[i].attribute) {
		case ATTRIBUTE::CROSSWALK:
			effective_density = seg_length * density * cw_density_factor;
			break;
		case ATTRIBUTE::HIGHWAY_LEFT:
		case ATTRIBUTE::HIGHWAY_RIGHT:
			effective_density = seg_length * density / hw_density_factor;
			break;
		default:
			break;
		}

		int localSteps = std::max(1, static_cast<int>(std::ceil(effective_density)));

		// Interpolate within the segment (existing code remains unchanged)
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
			v.attribute = path[i].attribute;

			result.push_back(v);
		}
	}
	return result;
}

Graph SplineUtils::interpolate_graph(const Graph &graph, double density, double hwDensityFactor, double cwDensityFactor, double smooth_factor) {
	using namespace boost;
	Graph result;
	if (num_vertices(graph) == 0)
		return result;
	std::vector<VD> old2new(num_vertices(graph));
	for (auto [vi, vi_end] = vertices(graph); vi != vi_end; ++vi) {
		old2new[*vi] = add_vertex(graph[*vi], result);
	}
	for (auto [ei, ei_end] = edges(graph); ei != ei_end; ++ei) {
		VD u = source(*ei, graph);
		VD v = target(*ei, graph);
		const auto &Vu = graph[u];
		const auto &Vv = graph[v];
		double dx = Vv.x - Vu.x;
		double dy = Vv.y - Vu.y;
		double length = std::hypot(dx, dy);
		if (length <= 0) {
			Edge eprop{0.0};
			add_edge(old2new[u], old2new[v], eprop, result);
			continue;
		}
		double effD = length * density;
		switch (Vu.attribute) {
		case ATTRIBUTE::CROSSWALK:
			effD *= cwDensityFactor;
			break;
		case ATTRIBUTE::HIGHWAY_LEFT:
		case ATTRIBUTE::HIGHWAY_RIGHT:
			effD /= hwDensityFactor;
			break;
		default:
			break;
		}
		int segments = std::max(1, static_cast<int>(std::ceil(effD)));
		if (segments == 1) {
			Edge eprop{length};
			add_edge(old2new[u], old2new[v], eprop, result);
			continue;
		}
		std::vector<VD> chain;
		chain.push_back(old2new[u]);
		for (int i = 1; i < segments; ++i) {
			double alpha = double(i) / segments;
			Vertex mid = Vu;
			mid.x = Vu.x + alpha * dx;
			mid.y = Vu.y + alpha * dy;
			mid.tangent_angle = std::atan2(dy, dx);
			mid.normal_angle = mid.tangent_angle + M_PI / 2.0;
			mid.curvature = 0.0;
			VD newV = add_vertex(mid, result);
			chain.push_back(newV);
		}
		chain.push_back(old2new[v]);
		for (size_t j = 0; j + 1 < chain.size(); ++j) {
			const auto &A = result[chain[j]];
			const auto &B = result[chain[j + 1]];
			double d = std::hypot(B.x - A.x, B.y - A.y);
			Edge eprop{d};
			add_edge(chain[j], chain[j + 1], eprop, result);
		}
	}
	return result;
}
