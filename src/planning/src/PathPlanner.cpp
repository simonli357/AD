#include "PathPlanner.hpp"
#include "utils/helper.h"
#include <iostream>
#include <memory>
#include <ostream>
#include <ros/package.h>
#include <yaml-cpp/yaml.h>

using Vertex = PathPlanner::Vertex;

PathPlanner::PathPlanner(double vref, int N, double T) : track(), spline_utils(), path_utils(), vref(vref), N(N), T(T) {
    this->attribute_map = std::make_unique<Track>("attribute_map.graphml");
	this->T = 0.1;
	this->density = 1.0 / std::fabs(this->vref) / this->T;
}

void PathPlanner::set_constraints(double vref, int N, double T, double start_x, double start_y, std::vector<std::tuple<float, float>> destination_positions) {
	this->vref = vref;
	this->N = N;
	this->T = 0.1;
	this->density = 1.0 / std::fabs(this->vref) / this->T;
	this->distance_threshold = vref * this->T * 1.5;
	path.clear();
	Vertex start;
	start.id = -2;
	start.x = start_x;
	start.y = start_y;
	Vertex first = track.find_closest_node(start_x, start_y);
	track.add_vertex(start, first);
	path.push_back(start);
	path.push_back(first);
	for (const auto &dest : destination_positions) {
		double x = std::get<0>(dest);
		double y = std::get<1>(dest);
		Vertex node = track.find_closest_node(x, y);
		path.push_back(node);
	}
	construct_path(true);
	track.remove_vertex(start);
}

void PathPlanner::set_constraints(double vref, int N, double T, double start_x, double start_y, std::string name, bool use_gps) {
	this->vref = vref;
	this->N = N;
	this->T = 0.1;
	this->density = 1.0 / std::fabs(this->vref) / this->T;
	this->distance_threshold = vref * this->T * 1.5;
    this->name = name;
	path.clear();
    Vertex start;
    Vertex first = track.find_closest_node(start_x, start_y);
    std::cout << "PathPlanner::set_constraints: Path: " << name << std::endl;
    if (use_gps) {
        start.id = -2;
        start.x = start_x;
        start.y = start_y;
        track.add_vertex(start, first);
        path.push_back(start);
    }
	path.push_back(first);
    precompute_path();
    construct_path(use_gps);
    if (use_gps) {
        track.remove_vertex(start);
    }
}

void PathPlanner::set_constraints(double vref, int N, double T, std::string name) {
	this->vref = vref;
	this->N = N;
	this->T = 0.1;
	this->density = 1.0 / std::fabs(this->vref) / this->T;
	this->distance_threshold = vref * this->T * 1.5;
    this->name = name;
	path.clear();
    precompute_path();
    construct_path(false);
}

void PathPlanner::precompute_path() {
	std::string package_path = ros::package::getPath("planning");
	std::string run_file = package_path + "/src/persistence/runs.yaml";
	try {
		YAML::Node config = YAML::LoadFile(run_file);
		if (!config[name]) {
			std::cerr << "Run name '" << name << "' not found in file.\n";
			return;
		}
		for (const auto &node : config[name]) {
            int id = node.as<int>();
            bool exists = std::any_of(path.begin(), path.end(), [id](const Vertex &v) {
                return v.id == id;
            });
            if (!exists) {
                path.push_back(track.find_node(id));
            }
		}
	} catch (const std::exception &e) {
		std::cerr << "Error reading YAML file: " << e.what() << std::endl;
	}
}

void PathPlanner::construct_path(bool use_gps) {
	std::vector<Vertex> general_path;
    Vertex prev;
	general_path.push_back(path[0]);
    prev = path[0];
    if (use_gps) {
        general_path.push_back(path[1]);
        prev = path[1];
    }
	for (const auto &v : path) {
		if (v.id == prev.id || v.id == path[0].id || prev.id == -2) {
			continue;
		}
		std::vector<Vertex> shortest_path = track.dikstra(prev.id, v.id);
		if (shortest_path.empty()) {
			std::cerr << "No path found between " << prev.x << ", " << prev.y << " and " << v.x << ", " << v.y << "\n";
			exit(1);
		}
		general_path.insert(general_path.end(), shortest_path.begin() + 1, shortest_path.end());
		prev = v;
	}
	condensed_path = spline_utils.interpolate_path(general_path, density, hw_density_factor, cw_density_factor);
	path_utils.compute_speeds(condensed_path, vref, density, hw_density_factor, cw_density_factor);
}

void PathPlanner::plan_path(Float32MultiArray &out_state_refs, Float32MultiArray &out_input_refs, Float32MultiArray &out_attributes, Float32MultiArray &out_normals) {
	for (const auto &v : condensed_path) {
		// State refs
		out_state_refs.data.push_back(v.x);
		out_state_refs.data.push_back(v.y);
		out_state_refs.data.push_back(v.tangent_angle);

		// Input refs
		out_input_refs.data.push_back(v.vref);
		out_input_refs.data.push_back(v.steer_ref);

		// Attributes
		out_attributes.data.push_back(static_cast<double>(v.attribute));

		// Normals
		out_normals.data.push_back(std::cos(v.normal_angle));
		out_normals.data.push_back(std::sin(v.normal_angle));
	}
	std::string path = helper::getSourceDirectory();
	/* saveTxt(out_state_refs, path + "/state_refs.txt", 3); */
	/* saveTxt(out_input_refs, path + "/input_refs.txt", 2); */
	/* saveTxt(out_attributes, path + "/attributes.txt", 1); */
}

std::vector<Vertex> PathPlanner::plan_path() {
    return condensed_path;
}

void PathPlanner::print_path() {
    std::cout << "Path: ";
    for (size_t i = 0; i < path.size(); i++) {
        if (i == path.size() - 1) {
            std::cout << path[i].id;
        } else {
            std::cout << path[i].id << "->";
        }
    }
    std::cout << std::endl;
}
