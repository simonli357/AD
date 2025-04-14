#include "path/Path.hpp"

Path::Path() : track(), spline_utils() {}

void Path::set_constraints(double vref, int N, int T, std::string name, double start_x, double start_y, std::vector<std::tuple<float, float>> destination_positions) {
    this->vref = vref;
    this->N = N;
    this->T = T;
    this->name = name;

    Vertex start = track.find_closest_node(start_x, start_y);
    path.push_back(start);
    for (const auto& dest : destination_positions) {
        double x = std::get<0>(dest);
        double y = std::get<1>(dest);
        Vertex node = track.find_closest_node(x, y);
        path.push_back(node);
    }
}

void Path::plan_path(std::vector<double> &state_refs, std::vector<double> &input_refs, std::vector<double> &attributes, std::vector<double> &normals) {
    if (path.empty()) {
        return;
    }
}
