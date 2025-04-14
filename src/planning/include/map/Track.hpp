#pragma once

#include <boost/graph/adjacency_list.hpp>
#include <boost/property_map/vector_property_map.hpp>

class Track {
  public:
	Track();
	~Track() = default;

	Track(Track &&) = default;
	Track(const Track &) = default;
	Track &operator=(Track &&) = default;
	Track &operator=(const Track &) = default;

	enum ATTRIBUTE {
		NORMAL = 0,
		CROSSWALK = 1,
		INTERSECTION = 2,
		ONEWAY = 3,
		HIGHWAY_LEFT = 4,
		HIGHWAY_RIGHT = 5,
		ROUNDABOUT = 6,
		STOPLINE = 7,
		DOTTED = 8,
		DOTTED_CROSSWALK = 9,
	};

	struct Vertex {
		int id = -1;
		double x;
		double y;
		double tangent_angle = 0;
		double normal_angle = 0;
		double curvature = 0;
		ATTRIBUTE attribute;
	};

	struct Edge {
		double distance;
	};

	using Graph = boost::adjacency_list<boost::vecS, boost::vecS, boost::directedS, Vertex, Edge>;

	std::vector<Vertex> dikstra(int src, int tgt);
	Vertex find_closest_node(double x_pos, double y_pos);
	double sqrt_dist(double x, double y, Vertex dest);
	double sqrt_dist(Vertex src, Vertex dest);

	void print_path(const std::vector<Vertex> &path);
	void print_graph();

  private:
	Graph graph;

	double hw_safety_offset = 0.05;

	std::unordered_map<int, Track::Graph::vertex_descriptor> build_to_vertex_map();

	void read_graph();
	void adjust_graph();
	void compute_edge_distances();

	friend std::istream &operator>>(std::istream &in, ATTRIBUTE &attr);
};
