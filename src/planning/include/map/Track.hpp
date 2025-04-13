#pragma once

#include <boost/graph/adjacency_list.hpp>
#include <boost/property_map/vector_property_map.hpp>
#include <string>

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

	struct VertexProperties {
		int id;
		double x;
		double y;
		ATTRIBUTE attribute;
	};

	using Graph = boost::adjacency_list<boost::vecS, boost::vecS, boost::directedS, VertexProperties>;

  private:
	Graph graph;

	void read_graph();
	void print_graph();

	friend std::istream &operator>>(std::istream &in, ATTRIBUTE &attr);
};
