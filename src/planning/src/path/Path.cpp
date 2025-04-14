#include "path/Path.hpp"

Path::Path() : track(), spline_utils() {}

void Path::set_constraints(double vref, int N, int T, std::string name, Vertex start, std::vector<Vertex> destinations) {
    this->vref = vref;
    this->N = N;
    this->T = T;
    this->name = name;
    this->start = start;
    this->destinations = destinations;
}

void Path::plan_path(std::vector<double> &state_refs, std::vector<double> &input_refs, std::vector<double> &attributes, std::vector<double> &normals) {

}
