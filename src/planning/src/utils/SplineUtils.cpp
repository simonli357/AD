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

Graph SplineUtils::interpolate_graph(const Graph &graph, double density, double hwDensityFactor, double cwDensityFactor, double smoothFactor) {
	using namespace boost;
	Graph result;

	// Nothing to do on empty graph
	size_t N = num_vertices(graph);
	if (N == 0)
		return result;

	// 1) DFS to get a visit order covering all vertices (including disconnected)
	std::vector<bool> visited(N, false);
	std::vector<VD> visitOrder;
	visitOrder.reserve(N);

	std::function<void(VD)> dfs = [&](VD u) {
		visited[u] = true;
		visitOrder.push_back(u);
		for (auto [oe, oe_end] = out_edges(u, graph); oe != oe_end; ++oe) {
			VD v = target(*oe, graph);
			if (!visited[v])
				dfs(v);
		}
	};
	for (auto [vi, vi_end] = vertices(graph); vi != vi_end; ++vi) {
		if (!visited[*vi])
			dfs(*vi);
	}

	// 2) Extract raw vertex data in visit order
	std::vector<Vertex> raw;
	raw.reserve(visitOrder.size());
	for (VD u : visitOrder)
		raw.push_back(graph[u]);

	// 3) Fit entire path and sample via existing interpolate_path
	std::vector<Vertex> samples = interpolate_path(raw, density, hwDensityFactor, cwDensityFactor, smoothFactor);

	// 4) Map each original vertex index -> nearest sample index
	std::vector<size_t> sampleIndex(visitOrder.size());
	for (size_t i = 0; i < visitOrder.size(); ++i) {
		const auto &orig = raw[i];
		double bestDist2 = std::numeric_limits<double>::infinity();
		size_t bestIdx = 0;
		for (size_t j = 0; j < samples.size(); ++j) {
			double dx = samples[j].x - orig.x;
			double dy = samples[j].y - orig.y;
			double d2 = dx * dx + dy * dy;
			if (d2 < bestDist2) {
				bestDist2 = d2;
				bestIdx = j;
			}
		}
		sampleIndex[i] = bestIdx;
	}

	// 5) Copy original vertices into result graph
	std::vector<VD> old2new(N);
	for (auto [vi, vi_end] = vertices(graph); vi != vi_end; ++vi) {
		old2new[*vi] = add_vertex(graph[*vi], result);
	}

	// 6) For each original edge, subdivide along the sampled spline
	for (auto [ei, ei_end] = edges(graph); ei != ei_end; ++ei) {
		VD u = source(*ei, graph);
		VD v = target(*ei, graph);
		// find positions in visitOrder
		auto it_u = std::find(visitOrder.begin(), visitOrder.end(), u);
		auto it_v = std::find(visitOrder.begin(), visitOrder.end(), v);
		size_t idx_u = std::distance(visitOrder.begin(), it_u);
		size_t idx_v = std::distance(visitOrder.begin(), it_v);
		size_t s0 = sampleIndex[idx_u];
		size_t s1 = sampleIndex[idx_v];
		if (s1 < s0)
			std::swap(s0, s1);

		// If no subdivision needed, copy edge directly
		if (s0 == s1) {
			Edge eprop;
			const auto &A = result[old2new[u]];
			const auto &B = result[old2new[v]];
			eprop.distance = std::hypot(B.x - A.x, B.y - A.y);
			add_edge(old2new[u], old2new[v], eprop, result);
			continue;
		}

		// Otherwise, add intermediate vertices
		VD prev = old2new[u];
		for (size_t k = s0 + 1; k <= s1; ++k) {
			VD curr;
			if (k == s1) {
				curr = old2new[v];
			} else {
				curr = add_vertex(samples[k], result);
			}
			Edge eprop;
			const auto &A = result[prev];
			const auto &B = result[curr];
			eprop.distance = std::hypot(B.x - A.x, B.y - A.y);
			add_edge(prev, curr, eprop, result);
			prev = curr;
		}
	}

	return result;
}
