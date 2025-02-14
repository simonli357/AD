#pragma once

#include "PathManager.hpp"
#include "ValueTypes.hpp"
#include "htn/World.hpp"
#include "utility.hpp"
#include <unordered_map>

class ObjectDetection {
public:
    ObjectDetection(World &world, std::unordered_map<PRIMITIVES, ValueType> &current_state);
    ObjectDetection(ObjectDetection &&) = default;
    ObjectDetection(const ObjectDetection &) = delete;
    ObjectDetection &operator=(ObjectDetection &&) = delete;
    ObjectDetection &operator=(const ObjectDetection &) = delete;
    ~ObjectDetection();

    World &world;
    Eigen::Vector3d &x_current;
    PathManager &path_manager;
    Utility &utils;
    std::unordered_map<PRIMITIVES, ValueType> &current_state;

    void detect_objects();

private:
    void detect_traffic_lights();
    void detect_signs();
    void detect_pedestrians();
    void detect_cars();
    void check_road_signs();
    bool check_highway_signs();
    void check_parking_signs();
    int park_sign_detected();
    bool sign_in_path(int sign_idx, double search_dist);
    bool intersection_reached();
    bool near_intersection();
};
