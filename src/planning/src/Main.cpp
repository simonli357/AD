#include <iostream>
#include "map/Track.hpp"
#include <utils/SplineUtils.hpp>

int main(int argc, char **argv) {
    std::cout << "TEST NODE - Planning" << std::endl;
    Track graph;
    SplineUtils interpolator;
    graph.print_graph();
    graph.print_path(graph.dikstra(44, 37));
    graph.print_path(interpolator.interpolate_path(graph.dikstra(44, 37), 40));
	return 0;
}
