#include <iostream>
#include "map/Track.hpp"

int main(int argc, char **argv) {
    std::cout << "TEST NODE - Planning" << std::endl;
    Track graph;
    graph.print_graph();
    graph.print_path(graph.dikstra(44, 37));
	return 0;
}
