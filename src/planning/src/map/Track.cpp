#include "map/Track.hpp"

#include <algorithm>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/graphml.hpp>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <tinyxml2.h>

using Vertex = Track::Vertex;

std::istream &operator>>(std::istream &in, Track::ATTRIBUTE &attr) {
	int temp;
	in >> temp;
	attr = static_cast<Track::ATTRIBUTE>(temp);
	return in;
}

Track::Track() {
	read_graph();
	compute_edge_distances();
}

Track::Track(const std::string &filename) {
	graph_file = package_path + "/src/persistence/" + filename;
	read_graph();
	compute_edge_distances();
}

void Track::read_graph() {
	const std::string package_path = ros::package::getPath("planning");
	const std::string graph_file = package_path + "/src/persistence/track.graphml";

	tinyxml2::XMLDocument doc;
	if (doc.LoadFile(graph_file.c_str()) != tinyxml2::XML_SUCCESS) {
		std::cerr << "[Track] Error loading GraphML: " << graph_file << "\n";
		return;
	}

	tinyxml2::XMLElement *graphml = doc.FirstChildElement("graphml");
	if (!graphml) {
		std::cerr << "[Track] No <graphml> root\n";
		return;
	}
	tinyxml2::XMLElement *graphElem = graphml->FirstChildElement("graph");
	if (!graphElem) {
		std::cerr << "[Track] No <graph> element\n";
		return;
	}

	graph.clear();
	std::unordered_map<int, Graph::vertex_descriptor> idToVertex;

	// --- Parse nodes ---
	for (auto *nodeElem = graphElem->FirstChildElement("node"); nodeElem; nodeElem = nodeElem->NextSiblingElement("node")) {
		const char *idStr = nodeElem->Attribute("id");
		if (!idStr)
			continue;

		// strip leading 'n'
		int nodeId = 0;
		if (idStr[0] == 'n' || idStr[0] == 'N') {
			nodeId = std::stoi(idStr + 1);
		} else {
			nodeId = std::stoi(idStr);
		}

		double xVal = 0.0, yVal = 0.0;
		int attrVal = 0;

		for (auto *dataElem = nodeElem->FirstChildElement("data"); dataElem; dataElem = dataElem->NextSiblingElement("data")) {
			const char *key = dataElem->Attribute("key");
			const char *txt = dataElem->GetText();
			if (!key || !txt)
				continue;

			if (std::strcmp(key, "d0") == 0)
				xVal = std::stod(txt);
			else if (std::strcmp(key, "d1") == 0)
				yVal = std::stod(txt);
			else if (std::strcmp(key, "d2") == 0)
				attrVal = std::stoi(txt);
		}

		auto vd = boost::add_vertex(graph);
		graph[vd].id = nodeId;
		graph[vd].x = xVal;
		graph[vd].y = yVal;
		graph[vd].attribute = static_cast<ATTRIBUTE>(attrVal);

		idToVertex[nodeId] = vd;
	}

	// --- Parse edges (only source/target, no extra properties) ---
	for (auto *edgeElem = graphElem->FirstChildElement("edge"); edgeElem; edgeElem = edgeElem->NextSiblingElement("edge")) {
		const char *srcStr = edgeElem->Attribute("source");
		const char *tgtStr = edgeElem->Attribute("target");
		if (!srcStr || !tgtStr)
			continue;

		int srcId = (srcStr[0] == 'n' || srcStr[0] == 'N') ? std::stoi(srcStr + 1) : std::stoi(srcStr);
		int tgtId = (tgtStr[0] == 'n' || tgtStr[0] == 'N') ? std::stoi(tgtStr + 1) : std::stoi(tgtStr);

		auto sit = idToVertex.find(srcId);
		auto tit = idToVertex.find(tgtId);
		if (sit == idToVertex.end() || tit == idToVertex.end()) {
			std::cerr << "[Track] Edge references unknown node: " << srcId << " -> " << tgtId << "\n";
			continue;
		}

		boost::add_edge(sit->second, tit->second, graph);
	}
}

void Track::compute_edge_distances() {
	for (auto ep = boost::edges(graph); ep.first != ep.second; ++ep.first) {
		auto e = *ep.first;
		auto s = boost::source(e, graph);
		auto t = boost::target(e, graph);
		double sx = graph[s].x;
		double sy = graph[s].y;
		double tx = graph[t].x;
		double ty = graph[t].y;
		double dx = sx - tx;
		double dy = sy - ty;
		double dist = std::sqrt(dx * dx + dy * dy);
		graph[e].distance = dist;
	}
}

void Track::add_vertex(const Vertex &u, const Vertex &v) {
	using vd_t = Graph::vertex_descriptor;
	vd_t u_desc = boost::add_vertex(graph);
	graph[u_desc] = u;
	auto id_map = build_to_vertex_map();
	auto it = id_map.find(v.id);
	if (it == id_map.end()) {
		throw std::runtime_error("Track::add_vertex: no existing vertex with id " + std::to_string(v.id));
	}
	vd_t v_desc = it->second;
	Edge e_prop;
	e_prop.distance = sqrt_dist(graph[u_desc], graph[v_desc]);
	boost::add_edge(u_desc, v_desc, e_prop, graph);
}

void Track::remove_vertex(const Vertex &u) {
	using vd_t = Graph::vertex_descriptor;
	auto id_map = build_to_vertex_map();
	auto it = id_map.find(u.id);
	if (it == id_map.end()) {
		throw std::runtime_error("Track::remove_vertex: no vertex with id " + std::to_string(u.id));
	}
	vd_t u_desc = it->second;
	boost::clear_vertex(u_desc, graph);
	boost::remove_vertex(u_desc, graph);
}

Vertex Track::find_first_neighbor(const Vertex &u) {
	auto id_map = build_to_vertex_map();
	auto it = id_map.find(u.id);
	if (it == id_map.end()) {
		throw std::runtime_error("Track::find_neighbor: no vertex with id " + std::to_string(u.id));
	}
	auto u_desc = it->second;
	auto [ei, ei_end] = boost::out_edges(u_desc, graph);
	if (ei == ei_end) {
		return u;
	}
	auto neighbor_desc = boost::target(*ei, graph);
	return graph[neighbor_desc];
}

std::unordered_map<int, Track::Graph::vertex_descriptor> Track::build_to_vertex_map() {
	std::unordered_map<int, Graph::vertex_descriptor> idMap;
	for (auto vp = boost::vertices(graph); vp.first != vp.second; ++vp.first) {
		auto v = *vp.first;
		idMap[graph[v].id] = v;
	}
	return idMap;
}

std::vector<Vertex> Track::dikstra(int src, int tgt) {
	auto idMap = build_to_vertex_map();

	auto sIt = idMap.find(src);
	auto tIt = idMap.find(tgt);
	if (sIt == idMap.end() || tIt == idMap.end()) {
		std::cerr << "Dijkstra error: src or tgt ID not found in the graph: " << src << " " << tgt << "\n";
		return {};
	}
	Graph::vertex_descriptor srcV = sIt->second;
	Graph::vertex_descriptor tgtV = tIt->second;

	if (srcV == tgtV) {
		return {graph[srcV]};
	}

	const auto n = boost::num_vertices(graph);
	std::vector<Graph::vertex_descriptor> predecessor(n);
	std::vector<double> distance(n, (std::numeric_limits<double>::max)());

	auto indexMap = get(boost::vertex_index, graph);

	auto weightMap = boost::get(&Edge::distance, graph);

	boost::dijkstra_shortest_paths(
		graph, srcV,
		boost::predecessor_map(boost::make_iterator_property_map(predecessor.begin(), indexMap)).distance_map(boost::make_iterator_property_map(distance.begin(), indexMap)).weight_map(weightMap));

	auto tgtIndex = indexMap[tgtV];
	if (distance[tgtIndex] == (std::numeric_limits<double>::max)()) {
		std::cerr << "No path from " << src << " to " << tgt << " found.\n";
		return {};
	}

	std::vector<Graph::vertex_descriptor> pathVerts;
	for (auto v = tgtV; v != srcV; v = predecessor[indexMap[v]]) {
		pathVerts.push_back(v);
	}
	pathVerts.push_back(srcV);
	std::reverse(pathVerts.begin(), pathVerts.end());

	std::vector<Vertex> path;
	path.reserve(pathVerts.size());
	for (auto v : pathVerts) {
		path.push_back(graph[v]);
	}

	return path;
}

Vertex Track::find_closest_node(double pos_x, double pos_y) {
	double best_dist = std::numeric_limits<double>::max();
	Vertex best_node;
	for (auto vp = boost::vertices(graph); vp.first != vp.second; ++vp.first) {
		auto v = *vp.first;
		auto &vertex = graph[v];
		double dist = sqrt_dist(pos_x, pos_y, vertex);
		if (dist < best_dist) {
			best_node = vertex;
			best_dist = dist;
		}
	}
	return best_node;
}

Vertex Track::find_closest_node(double pos_x, double pos_y, double car_yaw) {
	constexpr double kYawThresholdDeg = 25.0; // Acceptable ± range

	double best_dist = std::numeric_limits<double>::max();
	Vertex best_node; // Will hold the closest orientation-matched node

	for (auto vp = boost::vertices(graph); vp.first != vp.second; ++vp.first) {
		auto v = *vp.first;
		auto &vertex = graph[v];

		double yaw = vertex.tangent_angle * 180.0 / M_PI;
        double max = car_yaw * 180.0 / M_PI + kYawThresholdDeg;
        double min = car_yaw * 180.0 / M_PI - kYawThresholdDeg;

        std::cout << car_yaw << std::endl;
        std::cout << yaw << std::endl;

        if (yaw > max || yaw < min) {
            continue;
        }

		double dist = sqrt_dist(pos_x, pos_y, vertex);
		if (dist < best_dist) {
			best_dist = dist;
			best_node = vertex;
		}
	}
	return best_node;
}

Vertex Track::find_node(int id) {
	for (auto vp = boost::vertices(graph); vp.first != vp.second; ++vp.first) {
		auto v = *vp.first;
		auto &vertex = graph[v];
		if (vertex.id == id) {
			return vertex;
		}
	}
	std::cerr << "Node: " << id << "not found" << std::endl;
	exit(1);
}

double Track::sqrt_dist(double x, double y, const Vertex &dest) {
	double dx = dest.x - x;
	double dy = dest.y - y;
	return std::sqrt(dx * dx + dy * dy);
}

double Track::sqrt_dist(const Vertex &src, const Vertex &dest) {
	double dx = dest.x - src.x;
	double dy = dest.y - src.y;
	return std::sqrt(dx * dx + dy * dy);
}

std::string Track::serialize_graph(Graph &graph, bool save_to_file) {
	boost::dynamic_properties dp;
	dp.property("id", get(&Vertex::id, graph));
	dp.property("x", get(&Vertex::x, graph));
	dp.property("y", get(&Vertex::y, graph));
	dp.property("tangent", get(&Vertex::tangent_angle, graph));
	dp.property("normal", get(&Vertex::normal_angle, graph));
	dp.property("curv", get(&Vertex::curvature, graph));
	dp.property("vref", get(&Vertex::vref, graph));
	dp.property("steer", get(&Vertex::steer_ref, graph));
	dp.property("attr", get(&Vertex::attribute, graph));
	dp.property("dist", get(&Edge::distance, graph));
	std::ostringstream out;
	write_graphml(out, graph, dp, true);
	if (save_to_file) {
		std::string graph_file = package_path + "/src/persistence/graph.graphml";
		std::ofstream file(graph_file);
		if (file.is_open()) {
			file << out.str();
			file.close();
		}
	}
	return out.str();
}

void Track::print_path(const std::vector<Vertex> &path) {
	if (path.empty()) {
		std::cout << "Empty path\n";
		return;
	}
	double total_dist = 0.0;
	for (size_t i = 0; i + 1 < path.size(); ++i) {
		double dx = path[i + 1].x - path[i].x;
		double dy = path[i + 1].y - path[i].y;
		total_dist += std::sqrt(dx * dx + dy * dy);
	}
	for (size_t i = 0; i < path.size(); ++i) {
		std::cout << path[i].id;
		if (i + 1 < path.size()) {
			std::cout << "->";
		}
	}
	std::cout << "\nTotal Distance: " << total_dist << std::endl;
}

void Track::print_graph() {
	std::cout << "Vertices:\n";
	for (auto vp = boost::vertices(graph); vp.first != vp.second; ++vp.first) {
		auto v = *vp.first;
		std::cout << "  Vertex id: " << graph[v].id << ", x: " << graph[v].x << ", y: " << graph[v].y << ", attribute: " << static_cast<int>(graph[v].attribute) << std::endl;
	}
	std::cout << "\nEdges:\n";
	for (auto ep = boost::edges(graph); ep.first != ep.second; ++ep.first) {
		auto e = *ep.first;
		auto src = boost::source(e, graph);
		auto tgt = boost::target(e, graph);
		std::cout << "  " << graph[src].id << " -> " << graph[tgt].id << " | distance=" << graph[e].distance << std::endl;
	}
}
