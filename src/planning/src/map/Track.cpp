#include "map/Track.hpp"

#include <boost/graph/graphml.hpp>
#include <iostream>
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
	print_graph();
}

void Track::read_graph() {
	std::string package_path = ros::package::getPath("planning");
	std::string graph_file = package_path + "/src/map/track.graphml";

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
		auto &props = graph[v];

		double x = props.x;
		double y = props.y;

		if (props.id >= 502 && props.id <= 521) {
			y = y + hw_safety_offset;
		} else if (props.id >= 483 && props.id <= 502) {
			y = y - hw_safety_offset;
		}
		y = 13.786 - y;

		props.x = x;
		props.y = y;
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
