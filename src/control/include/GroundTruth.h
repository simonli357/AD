#pragma once

#include <vector>
#include <optional>
#include <Eigen/Dense>
#include "utils/constants.h"

namespace GroundTruth {

using namespace VehicleConstants;

struct Intersection;
struct Sign {
    Eigen::Vector3d pose;        // x, y, yaw
    OBJECT type;                 // Can be NONE initially and later updated
    Intersection* intersection = nullptr;  // back-reference

    Sign(const Eigen::Vector3d& pose_, OBJECT type_)
        : pose(pose_), type(type_) {}
    
    Sign(const std::vector<double>& v, OBJECT type_)
        : Sign(Eigen::Vector3d{v[0], v[1], v[2]}, type_) {}
};

struct Intersection {
    Eigen::Vector3d pose;                         // x, y, yaw
    std::shared_ptr<Sign> associated_sign = nullptr;
    std::string direction;                        // "north", "south", etc. (optional, useful for debugging/lookup)

    Intersection(const Eigen::Vector3d& pose_,
            std::shared_ptr<Sign> sign = nullptr,
            const std::string& dir = "")
        : pose(pose_), direction(dir) {
        if (sign) {
            double dist = (pose.head<2>() - sign->pose.head<2>()).norm();
            if (dist >= 1.0) {
                std::cerr << "[Intersection] Sign pose (" 
                        << sign->pose[0] << ", " << sign->pose[1] 
                        << ") is too far from intersection pose (" 
                        << pose[0] << ", " << pose[1] << ") — must be < 1.0m\n";
                assert(false);
            }
            sign->intersection = this;
            associated_sign = std::move(sign);
        }
    }

    Intersection(const std::vector<double>& v,
        std::shared_ptr<Sign> sign = nullptr,
        const std::string& dir = "")
    : Intersection(Eigen::Vector3d{v[0], v[1], v[2]}, sign, dir) {}
};

// All intersections with optional associated signs
inline std::vector<Intersection> intersections_all;
// Intersection signs (ie. stop signs, traffic lights, prios, crosswalks, roundabouts)
inline std::vector<std::shared_ptr<Sign>> intersection_signs;
// Standalone signs (ie. parking, highway entries/exits)
inline std::vector<std::shared_ptr<Sign>> standalone_signs;
inline std::vector<Eigen::Vector2d> PARKING_SPOTS;
// === Utility Functions ===
inline void clear_ground_truth() {
    intersections_all.clear();
    standalone_signs.clear();
    intersection_signs.clear();
}

inline void initialize_ground_truth() {
    clear_ground_truth();

    using namespace VehicleConstants;

    const auto& S = SOUTH_FACING_INTERSECTIONS;
    const auto& N = NORTH_FACING_INTERSECTIONS;
    const auto& W = WEST_FACING_INTERSECTIONS;
    const auto& E = EAST_FACING_INTERSECTIONS;

    std::cout << "[GroundTruth] Initializing ground truth...\n";
    // ==== Helper Lambda ====
    auto make_intersection = [](const std::vector<double>& inter_pose,
                                          const std::vector<double>& sign_pose,
                                          OBJECT type,
                                          std::vector<Intersection>& container,
                                          const std::string& dir) {
        
        assert(inter_pose.size() >= 3 && "Intersection pose must have at least 3 elements");
        assert(sign_pose.size() >= 3 && "Sign pose must have at least 3 elements");
        auto sign_ptr = std::make_shared<Sign>(sign_pose, type);
        intersection_signs.push_back(sign_ptr);
        container.emplace_back(inter_pose, sign_ptr, dir);
    };

    // ==== SOUTH ====
    make_intersection(S[0], ALL_SIGNS[0], OBJECT::NONE, intersections_all, "south");
    make_intersection(S[1], ALL_SIGNS[1], OBJECT::NONE, intersections_all, "south");
    make_intersection(S[2], ALL_SIGNS[2], OBJECT::NONE, intersections_all, "south");
    make_intersection(S[3], ALL_SIGNS[3], OBJECT::NONE, intersections_all, "south");
    make_intersection(S[4], ALL_SIGNS[4], OBJECT::NONE, intersections_all, "south");
    make_intersection(S[5], ALL_SIGNS[5], OBJECT::NONE, intersections_all, "south");
    make_intersection(S[6], ALL_SIGNS[6], OBJECT::NONE, intersections_all, "south");
    make_intersection(S[7], ALL_SIGNS[7], OBJECT::NONE, intersections_all, "south");
    make_intersection(S[8], ALL_LIGHTS[0], OBJECT::LIGHTS, intersections_all, "south");
    make_intersection(S[9], ALL_ROUNDABOUTS[0], OBJECT::ROUNDABOUT, intersections_all, "south");
    make_intersection(S[10], ALL_CROSSWALKS[0], OBJECT::CROSSWALK, intersections_all, "south");
    make_intersection(S[11], ALL_CROSSWALKS[1], OBJECT::CROSSWALK, intersections_all, "south");

    // ==== NORTH ====
    make_intersection(N[0], ALL_SIGNS[8], OBJECT::NONE, intersections_all, "north");
    make_intersection(N[1], ALL_SIGNS[9], OBJECT::NONE, intersections_all, "north");
    make_intersection(N[2], ALL_SIGNS[10], OBJECT::NONE, intersections_all, "north");
    make_intersection(N[3], ALL_SIGNS[11], OBJECT::NONE, intersections_all, "north");
    make_intersection(N[4], ALL_SIGNS[12], OBJECT::NONE, intersections_all, "north");
    make_intersection(N[5], ALL_SIGNS[13], OBJECT::NONE, intersections_all, "north");
    make_intersection(N[6], ALL_SIGNS[14], OBJECT::NONE, intersections_all, "north");
    make_intersection(N[7], ALL_LIGHTS[1], OBJECT::LIGHTS, intersections_all, "north");
    make_intersection(N[8], ALL_ROUNDABOUTS[1], OBJECT::ROUNDABOUT, intersections_all, "north");
    make_intersection(N[9], ALL_CROSSWALKS[2], OBJECT::CROSSWALK, intersections_all, "north");
    make_intersection(N[10], ALL_CROSSWALKS[3], OBJECT::CROSSWALK, intersections_all, "north");

    // ==== WEST ====
    make_intersection(W[0], ALL_SIGNS[15], OBJECT::NONE, intersections_all, "west");
    make_intersection(W[1], ALL_SIGNS[16], OBJECT::NONE, intersections_all, "west");
    make_intersection(W[2], ALL_SIGNS[17], OBJECT::NONE, intersections_all, "west");
    make_intersection(W[3], ALL_SIGNS[18], OBJECT::NONE, intersections_all, "west");
    make_intersection(W[4], ALL_SIGNS[19], OBJECT::NONE, intersections_all, "west");
    make_intersection(W[5], ALL_SIGNS[20], OBJECT::NONE, intersections_all, "west");
    make_intersection(W[6], ALL_SIGNS[21], OBJECT::NONE, intersections_all, "west");
    make_intersection(W[7], ALL_SIGNS[22], OBJECT::NONE, intersections_all, "west");
    make_intersection(W[8], ALL_LIGHTS[2], OBJECT::LIGHTS, intersections_all, "west");
    make_intersection(W[9], ALL_ROUNDABOUTS[2], OBJECT::ROUNDABOUT, intersections_all, "west");
    make_intersection(W[10], ALL_CROSSWALKS[4], OBJECT::CROSSWALK, intersections_all, "west");
    make_intersection(W[11], ALL_CROSSWALKS[5], OBJECT::CROSSWALK, intersections_all, "west");

    // ==== EAST ====
    make_intersection(E[0], ALL_SIGNS[23], OBJECT::NONE, intersections_all, "east");
    make_intersection(E[1], ALL_SIGNS[24], OBJECT::NONE, intersections_all, "east");
    make_intersection(E[2], ALL_SIGNS[25], OBJECT::NONE, intersections_all, "east");
    make_intersection(E[3], ALL_SIGNS[26], OBJECT::NONE, intersections_all, "east");
    make_intersection(E[4], ALL_SIGNS[27], OBJECT::NONE, intersections_all, "east");
    make_intersection(E[5], ALL_SIGNS[28], OBJECT::NONE, intersections_all, "east");
    make_intersection(E[6], ALL_LIGHTS[3], OBJECT::LIGHTS, intersections_all, "east");
    make_intersection(E[7], ALL_ROUNDABOUTS[3], OBJECT::ROUNDABOUT, intersections_all, "east");
    make_intersection(E[8], ALL_CROSSWALKS[6], OBJECT::CROSSWALK, intersections_all, "east");
    make_intersection(E[9], ALL_CROSSWALKS[7], OBJECT::CROSSWALK, intersections_all, "east");

    // ==== Standalone signs ====
    for (const auto& p : ALL_HIGHWAYENTRANCES) {
        standalone_signs.emplace_back(std::make_shared<Sign>(p, OBJECT::HIGHWAYENTRANCE));
    }
    for (const auto& p : ALL_HIGHWAYEXITS) {
        standalone_signs.emplace_back(std::make_shared<Sign>(p, OBJECT::HIGHWAYEXIT));
    }
    for (const auto& p : PARKING_SIGN_POSES1) {
        standalone_signs.emplace_back(std::make_shared<Sign>(p, OBJECT::PARK));
    }
    for (const auto& p : PARKING_SIGN_POSES2) {
        standalone_signs.emplace_back(std::make_shared<Sign>(p, OBJECT::PARK));
    }
    for (const auto& p : ALL_ONEWAYS) {
        standalone_signs.emplace_back(std::make_shared<Sign>(p, OBJECT::ONEWAY));
    }

    for(int i=0; i<5; i++) {
        Eigen::Vector2d spot_right = {PARKING_SPOT_RIGHT[0] + i*PARKING_SPOT_LENGTH, PARKING_SPOT_RIGHT[1]};
        Eigen::Vector2d spot_left = {PARKING_SPOT_LEFT[0] + i*PARKING_SPOT_LENGTH, PARKING_SPOT_LEFT[1]};
        PARKING_SPOTS.push_back(spot_right);
        PARKING_SPOTS.push_back(spot_left);
    }
    std::cout << "[GroundTruth] Done Initializing ground truth...\n";
}

}  // namespace GroundTruth
