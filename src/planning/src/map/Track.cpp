#include "map/Track.hpp"

#include <algorithm>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/graphml.hpp>
#include <iostream>
#include <limits>
#include <ros/package.h>
#include <tinyxml2.h>

std::istream &operator>>(std::istream &in, Track::ATTRIBUTE &attr) {
	int temp;
	in >> temp;
	attr = static_cast<Track::ATTRIBUTE>(temp);
	return in;
}

Track::Track() {
	read_graph();
	adjust_graph();
	compute_edge_distances();
}

void Track::read_graph() {
	std::string package_path = ros::package::getPath("planning");
	std::string graph_file = package_path + "/src/persistence/track.graphml";

	tinyxml2::XMLDocument doc;
	tinyxml2::XMLError err = doc.LoadFile(graph_file.c_str());
	if (err != tinyxml2::XML_SUCCESS) {
		std::cerr << "Error opening/parsing file: " << graph_file << "\nTinyXML2 error code: " << err << std::endl;
		return;
	}

	tinyxml2::XMLElement *graphml = doc.FirstChildElement("graphml");
	if (!graphml) {
		std::cerr << "No <graphml> element found.\n";
		return;
	}

	tinyxml2::XMLElement *graphElem = graphml->FirstChildElement("graph");
	if (!graphElem) {
		std::cerr << "No <graph> element found.\n";
		return;
	}

	std::unordered_map<int, Graph::vertex_descriptor> idToVertex;

	for (tinyxml2::XMLElement *nodeElem = graphElem->FirstChildElement("node"); nodeElem; nodeElem = nodeElem->NextSiblingElement("node")) {
		const char *nodeIdStr = nodeElem->Attribute("id");
		if (!nodeIdStr) {
			std::cerr << "A <node> element has no id attribute.\n";
			continue;
		}
		int nodeId = std::stoi(nodeIdStr);

		double xVal = 0.0;
		double yVal = 0.0;
		int attrVal = 0;

		for (tinyxml2::XMLElement *dataElem = nodeElem->FirstChildElement("data"); dataElem; dataElem = dataElem->NextSiblingElement("data")) {
			const char *keyAttr = dataElem->Attribute("key");
			if (!keyAttr) {
				continue;
			}
			const char *textValue = dataElem->GetText();
			if (!textValue) {
				continue;
			}

			if (std::strcmp(keyAttr, "d0") == 0) {
				// e.g. <data key="d0">4.17</data>
				xVal = std::stod(textValue);
			} else if (std::strcmp(keyAttr, "d1") == 0) {
				yVal = std::stod(textValue);
			} else if (std::strcmp(keyAttr, "d2") == 0) {
				// e.g. <data key="d2">7</data>
				attrVal = std::stoi(textValue);
			}
		}

		Graph::vertex_descriptor v = boost::add_vertex(graph);

		graph[v].id = nodeId;
		graph[v].x = xVal;
		graph[v].y = yVal;
		graph[v].attribute = static_cast<ATTRIBUTE>(attrVal);

		idToVertex[nodeId] = v;
	}

	for (tinyxml2::XMLElement *edgeElem = graphElem->FirstChildElement("edge"); edgeElem; edgeElem = edgeElem->NextSiblingElement("edge")) {
		const char *sourceStr = edgeElem->Attribute("source");
		const char *targetStr = edgeElem->Attribute("target");
		if (!sourceStr || !targetStr) {
			std::cerr << "Edge missing source= or target=.\n";
			continue;
		}
		int sourceId = std::stoi(sourceStr);
		int targetId = std::stoi(targetStr);

		auto sIt = idToVertex.find(sourceId);
		auto tIt = idToVertex.find(targetId);
		if (sIt == idToVertex.end() || tIt == idToVertex.end()) {
			std::cerr << "Edge references unknown node: " << sourceId << " -> " << targetId << std::endl;
			continue;
		}
		boost::add_edge(sIt->second, tIt->second, graph);
	}
}

void Track::adjust_graph() {
	for (auto vp = boost::vertices(graph); vp.first != vp.second; ++vp.first) {
		auto v = *vp.first;
		auto &vertex = graph[v];

		double x = vertex.x;
		double y = vertex.y;

		if (vertex.id >= 502 && vertex.id <= 521) {
			y = y + hw_safety_offset;
		} else if (vertex.id >= 483 && vertex.id <= 502) {
			y = y - hw_safety_offset;
		}
		y = 13.786 - y;

		vertex.x = x;
		vertex.y = y;
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

std::unordered_map<int, Track::Graph::vertex_descriptor> Track::build_to_vertex_map() {
	std::unordered_map<int, Graph::vertex_descriptor> idMap;
	for (auto vp = boost::vertices(graph); vp.first != vp.second; ++vp.first) {
		auto v = *vp.first;
		idMap[graph[v].id] = v;
	}
	return idMap;
}

std::vector<Track::Vertex> Track::dikstra(int src, int tgt) {
	auto idMap = build_to_vertex_map();

	auto sIt = idMap.find(src);
	auto tIt = idMap.find(tgt);
	if (sIt == idMap.end() || tIt == idMap.end()) {
		std::cerr << "Dijkstra error: src or tgt ID not found in the graph.\n";
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

Track::Vertex Track::find_closest_node(double pos_x, double pos_y) {
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

Track::Vertex Track::find_node(int id) {
	for (auto vp = boost::vertices(graph); vp.first != vp.second; ++vp.first) {
		auto v = *vp.first;
		auto &vertex = graph[v];
		if (vertex.id == id) {
			return vertex;
		}
	}
	std::cerr << "Node not found" << std::endl;
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

std::string Track::serialize_graph(Graph &graph) {
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
