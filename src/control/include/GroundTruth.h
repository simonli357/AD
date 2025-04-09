#pragma once

#include <vector>
#include <optional>
#include <Eigen/Dense>
#include "utils/constants.h"

namespace GroundTruth {

using namespace VehicleConstants;

struct Sign {
    Eigen::Vector3d pose;        // x, y, yaw
    OBJECT type;                 // Can be NONE initially and later updated

    Sign(const Eigen::Vector3d& pose_, OBJECT type_)
        : pose(pose_), type(type_) {}
    
    Sign(const std::vector<double>& v, OBJECT type_)
        : Sign(Eigen::Vector3d{v[0], v[1], v[2]}, type_) {}
};

struct Intersection {
    Eigen::Vector3d pose;                         // x, y, yaw
    std::optional<Sign> associated_sign;          // may be set later
    std::string direction;                        // "north", "south", etc. (optional, useful for debugging/lookup)

    Intersection(const Eigen::Vector3d& pose_,
            const std::optional<Sign>& sign = std::nullopt,
            const std::string& dir = "")
        : pose(pose_), associated_sign(sign), direction(dir) {
        if (sign.has_value()) {
            double dist = (pose.head<2>() - sign->pose.head<2>()).norm();
            if (dist >= 1.0) {
                std::cerr << "[Intersection] Sign pose (" 
                        << sign->pose[0] << ", " << sign->pose[1] 
                        << ") is too far from intersection pose (" 
                        << pose[0] << ", " << pose[1] << ") — must be < 1.0m\n";
                assert(false);
            }
        }
    }

    Intersection(const std::vector<double>& v,
        const std::optional<Sign>& sign = std::nullopt,
        const std::string& dir = "")
    : Intersection(Eigen::Vector3d{v[0], v[1], v[2]}, sign, dir) {}
};

// All intersections with optional associated signs
inline std::vector<Intersection> intersections_all;
inline std::vector<Intersection> intersections_south;
inline std::vector<Intersection> intersections_north;
inline std::vector<Intersection> intersections_west;
inline std::vector<Intersection> intersections_east;

// Standalone signs (ie. parking, highway entries/exits)
inline std::vector<Sign> standalone_signs;

// === Utility Functions ===
inline void clear_ground_truth() {
    intersections_south.clear();
    intersections_north.clear();
    intersections_west.clear();
    intersections_east.clear();
    intersections_all.clear();
    standalone_signs.clear();
}

inline void initialize_ground_truth() {
    clear_ground_truth();

    using namespace VehicleConstants;

    const auto& S = SOUTH_FACING_INTERSECTIONS;
    const auto& N = NORTH_FACING_INTERSECTIONS;
    const auto& W = WEST_FACING_INTERSECTIONS;
    const auto& E = EAST_FACING_INTERSECTIONS;

    // ==== SOUTH ====
    intersections_south.emplace_back(S[0], Sign{ALL_SIGNS[0], OBJECT::NONE}, "south");
    intersections_south.emplace_back(S[1], Sign{ALL_SIGNS[1], OBJECT::NONE}, "south");
    intersections_south.emplace_back(S[2], Sign{ALL_SIGNS[2], OBJECT::NONE}, "south");
    intersections_south.emplace_back(S[3], Sign{ALL_SIGNS[3], OBJECT::NONE}, "south");
    intersections_south.emplace_back(S[4], Sign{ALL_SIGNS[4], OBJECT::NONE}, "south");
    intersections_south.emplace_back(S[5], Sign{ALL_SIGNS[5], OBJECT::NONE}, "south");
    intersections_south.emplace_back(S[6], Sign{ALL_SIGNS[6], OBJECT::NONE}, "south");
    intersections_south.emplace_back(S[7], Sign{ALL_SIGNS[7], OBJECT::NONE}, "south");
    intersections_south.emplace_back(S[8], Sign{ALL_LIGHTS[0], OBJECT::LIGHTS}, "south");
    intersections_south.emplace_back(S[9], Sign{ALL_ROUNDABOUTS[0], OBJECT::ROUNDABOUT}, "south");
    intersections_south.emplace_back(S[10], Sign{ALL_CROSSWALKS[0], OBJECT::CROSSWALK}, "south");
    intersections_south.emplace_back(S[11], Sign{ALL_CROSSWALKS[1], OBJECT::CROSSWALK}, "south");

    // ==== NORTH ====
    intersections_north.emplace_back(N[0], Sign{ALL_SIGNS[8], OBJECT::NONE}, "north");
    intersections_north.emplace_back(N[1], Sign{ALL_SIGNS[9], OBJECT::NONE}, "north");
    intersections_north.emplace_back(N[2], Sign{ALL_SIGNS[10], OBJECT::NONE}, "north");
    intersections_north.emplace_back(N[3], Sign{ALL_SIGNS[11], OBJECT::NONE}, "north");
    intersections_north.emplace_back(N[4], Sign{ALL_SIGNS[12], OBJECT::NONE}, "north");
    intersections_north.emplace_back(N[5], Sign{ALL_SIGNS[13], OBJECT::NONE}, "north");
    intersections_north.emplace_back(N[6], Sign{ALL_SIGNS[14], OBJECT::NONE}, "north");
    intersections_north.emplace_back(N[7], Sign{ALL_LIGHTS[1], OBJECT::LIGHTS}, "north");
    intersections_north.emplace_back(N[8], Sign{ALL_ROUNDABOUTS[1], OBJECT::ROUNDABOUT}, "north");
    intersections_north.emplace_back(N[9], Sign{ALL_CROSSWALKS[2], OBJECT::CROSSWALK}, "north");
    intersections_north.emplace_back(N[10], Sign{ALL_CROSSWALKS[3], OBJECT::CROSSWALK}, "north");

    // ==== WEST ====
    intersections_west.emplace_back(W[0], Sign{ALL_SIGNS[15], OBJECT::NONE}, "west");
    intersections_west.emplace_back(W[1], Sign{ALL_SIGNS[16], OBJECT::NONE}, "west");
    intersections_west.emplace_back(W[2], Sign{ALL_SIGNS[17], OBJECT::NONE}, "west");
    intersections_west.emplace_back(W[3], Sign{ALL_SIGNS[18], OBJECT::NONE}, "west");
    intersections_west.emplace_back(W[4], Sign{ALL_SIGNS[19], OBJECT::NONE}, "west");
    intersections_west.emplace_back(W[5], Sign{ALL_SIGNS[20], OBJECT::NONE}, "west");
    intersections_west.emplace_back(W[6], Sign{ALL_SIGNS[21], OBJECT::NONE}, "west");
    intersections_west.emplace_back(W[7], Sign{ALL_SIGNS[22], OBJECT::NONE}, "west");
    intersections_west.emplace_back(W[8], Sign{ALL_LIGHTS[2], OBJECT::LIGHTS}, "west");
    intersections_west.emplace_back(W[9], Sign{ALL_ROUNDABOUTS[2], OBJECT::ROUNDABOUT}, "west");
    intersections_west.emplace_back(W[10], Sign{ALL_CROSSWALKS[4], OBJECT::CROSSWALK}, "west");
    intersections_west.emplace_back(W[11], Sign{ALL_CROSSWALKS[5], OBJECT::CROSSWALK}, "west");

    // ==== EAST ====
    intersections_east.emplace_back(E[0], Sign{ALL_SIGNS[23], OBJECT::NONE}, "east");
    intersections_east.emplace_back(E[1], Sign{ALL_SIGNS[24], OBJECT::NONE}, "east");
    intersections_east.emplace_back(E[2], Sign{ALL_SIGNS[25], OBJECT::NONE}, "east");
    intersections_east.emplace_back(E[3], Sign{ALL_SIGNS[26], OBJECT::NONE}, "east");
    intersections_east.emplace_back(E[4], Sign{ALL_SIGNS[27], OBJECT::NONE}, "east");
    intersections_east.emplace_back(E[5], Sign{ALL_SIGNS[28], OBJECT::NONE}, "east");
    intersections_east.emplace_back(E[6], Sign{ALL_LIGHTS[3], OBJECT::LIGHTS}, "east");
    intersections_east.emplace_back(E[7], Sign{ALL_ROUNDABOUTS[3], OBJECT::ROUNDABOUT}, "east");
    intersections_east.emplace_back(E[8], Sign{ALL_CROSSWALKS[6], OBJECT::CROSSWALK}, "east");
    intersections_east.emplace_back(E[9], Sign{ALL_CROSSWALKS[7], OBJECT::CROSSWALK}, "east");

    for (const auto& inter : intersections_south) {
        intersections_all.push_back(inter);
    }
    for (const auto& inter : intersections_north) {
        intersections_all.push_back(inter);
    }
    for (const auto& inter : intersections_west) {
        intersections_all.push_back(inter);
    }
    for (const auto& inter : intersections_east) {
        intersections_all.push_back(inter);
    }

    // ==== Standalone signs ====
    for (const auto& p : ALL_HIGHWAYENTRANCES) {
        standalone_signs.emplace_back(Sign{p, OBJECT::HIGHWAYENTRANCE});
    }
    for (const auto& p : ALL_HIGHWAYEXITS) {
        standalone_signs.emplace_back(Sign{p, OBJECT::HIGHWAYEXIT});
    }
    for (const auto& p : PARKING_SIGN_POSES1) {
        standalone_signs.emplace_back(Sign{p, OBJECT::PARK});
    }
    for (const auto& p : PARKING_SIGN_POSES2) {
        standalone_signs.emplace_back(Sign{p, OBJECT::PARK});
    }
}

}  // namespace GroundTruth
