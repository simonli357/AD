#include "planner/PathPlanner.hpp"

PathPlanner::PathPlanner() : track(), spline_utils() {}

void PathPlanner::set_constraints(double vref, int N, int T, std::string name, double start_x, double start_y, std::vector<std::tuple<float, float>> destination_positions) {
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

    interpolate_path();
}

void PathPlanner::interpolate_path() {
    std::vector<Vertex> general_path;
    general_path.push_back(path[0]);
    Vertex prev = path[0];
    for (const auto &v : path) {
        if (v.id == prev.id) {
            continue;
        }
        std::vector<Vertex> shortest_path = track.dikstra(prev.id, v.id);
        general_path.insert(general_path.end(), shortest_path.begin() + 1, shortest_path.end());
        prev = v;
    }
    condensed_path = spline_utils.interpolate_path(general_path, N);
}

void PathPlanner::plan_path(std::vector<double> &state_refs, std::vector<double> &input_refs, std::vector<double> &attributes, std::vector<double> &normals) {
    for (const auto &v : condensed_path) {
        // State refs
        state_refs.push_back(v.x);
        state_refs.push_back(v.y);
        state_refs.push_back(v.tangent_angle);
        
        // Input refs
        input_refs.push_back(vref);
        input_refs.push_back(v.curvature);

        // Attributes
        attributes.push_back(static_cast<double>(v.attribute));

        // Normals
        normals.push_back(std::cos(v.normal_angle));
        normals.push_back(std::sin(v.normal_angle));
    }
}
