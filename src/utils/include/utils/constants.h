#ifndef CONSTANTS_H
#define CONSTANTS_H

#include "Database.hpp"
#include <cmath>
#include <array>
#include <string>
#include <vector>
#include <tbb/task_arena.h>

namespace ThreadPools {
    inline tbb::task_arena communication{1}; // 1 Thread for communication tasks
}

namespace VehicleConstants {
    void init_params(Database &db);
    // vehicle constants
    static constexpr double WHEELBASE = 0.260;
    // static constexpr double L_R_SIM = 0.130;
    // static constexpr double L_F_SIM = 0.130;
    static constexpr double L_R_SIM = 0.108;
    static constexpr double L_F_SIM = 0.152;
    static constexpr double L_R_REAL = 0.108;
    static constexpr double L_F_REAL = 0.152;

    // static constexpr double CAR_LENGTH = 0.464; // sim
    static constexpr double CAR_LENGTH = 0.375;
    // static constexpr double CAR_WIDTH = 0.1885; //sim
    static constexpr double CAR_WIDTH = 0.195; // real
    static constexpr double CAR_HEIGHT = 0.1155;
    static constexpr double SAME_LANE_SAFETY_FACTOR = 0.0357;//0.0753;
    static constexpr double LANE_CENTER_TO_EDGE = 0.0777;
    static constexpr int IMAGE_HEIGHT = 480;
    static constexpr int IMAGE_WIDTH = 640;
    static constexpr double MAX_TAILING_DIST = 0.75;
    static constexpr double MIN_SIGN_DIST = 0.375;  // 0.6 - 0.21
    static constexpr double MAX_PARK_DIST = 1.;  // 1.0 - 0.21
    static constexpr double MAX_CROSSWALK_DIST = 0.79;  // 1.0 - 0.21
    static constexpr double PARKSIGN_TO_CAR = 0.51;
    static constexpr double PARK_OFFSET = 1.31;    // 1.1 + 0.21
    static constexpr double PARKING_SPOT_LENGTH = 0.723;
    static constexpr double PARKING_SPOT_WIDTH = 0.362;
    static constexpr double MAX_PARKING_Y_ERROR = 0.0775;
    static constexpr double CROSSWALK_LENGTH = 1.0;
    static constexpr double OVERTAKE_DIST = 2.0;
    static constexpr double LANE_OFFSET = 0.3935;
    static constexpr double INNER_LANE_OFFSET = 0.3465;
    static constexpr double LANE_WHITE = 0.0235;
    static constexpr double MIN_DIST_TO_CAR = 0.3;
    // static constexpr double MAX_CAR_DIST = 3.0;
    static constexpr double MIN_CAR_DIST = 1.0;
    static constexpr double SIGN_COOLDOWN = 1.0;
    static constexpr double TOLERANCE_SQUARED = 0.15 * 0.15;
    static constexpr double SOFT_MAX_STEERING = 0.3578 * 180 / M_PI;
    static constexpr double HARD_MAX_STEERING = 0.4363 * 180 / M_PI;
    static constexpr double INTERSECTION_DISTANCE_THRESHOLD = 0.72345; // minimum distance between two intersections
    static constexpr double INTERSECTION_TO_SIGN = 0.290; // distance from intersection to sign

    static constexpr double pole_size = 0.0514;

    // parking coordinates
    static constexpr std::array<double, 2> PARKING_SPOT_RIGHT = {9.40, 0.56};
    static constexpr std::array<double, 2> PARKING_SPOT_LEFT = {9.40, 1.30};

    static constexpr double ofs6 = INNER_LANE_OFFSET/2 - pole_size/2;
    static constexpr double hsw = pole_size/2;

    const std::array<std::string, 13> state_names = {
        "INIT", "MOVING", "APPROACHING_INTERSECTION", "WAITING_FOR_STOPSIGN",
        "WAITING_FOR_LIGHT", "PARKING", "PARKED", "EXITING_PARKING", "DONE", 
        "LANE_FOLLOWING", "INTERSECTION_MANEUVERING", "KEYBOARD", "TESTING"
    };

    enum STATE {
        INIT,
        MOVING,
        APPROACHING_INTERSECTION,
        WAITING_FOR_STOPSIGN,
        WAITING_FOR_LIGHT,
        PARKING,
        PARKED,
        EXITING_PARKING,
        DONE,
        LANE_FOLLOWING,
        INTERSECTION_MANEUVERING,
        KEYBOARD_CONTROL,
        TESTING
    };
    enum LightColor {
        RED,
        GREEN,
        YELLOW,
        UNDETERMINED
    };
    enum OBJECT {
        ONEWAY,
        HIGHWAYENTRANCE,
        STOPSIGN,
        ROUNDABOUT,
        PARK,
        CROSSWALK,
        NOENTRY,
        HIGHWAYEXIT,
        PRIORITY,
        LIGHTS,
        BLOCK,
        PEDESTRIAN,
        CAR,
        GREENLIGHT,
        YELLOWLIGHT,
        REDLIGHT,
        NONE,
        FOG,
        TUNNEL,
        RAMP
    };

    static constexpr std::array<OBJECT, 12> KNOWN_STATIC_OBJECTS = { 
        HIGHWAYENTRANCE, STOPSIGN, 
        ROUNDABOUT, PARK, CROSSWALK, 
        HIGHWAYEXIT, PRIORITY, LIGHTS, 
        GREENLIGHT, YELLOWLIGHT, REDLIGHT,
        ONEWAY
    };
    const std::array<std::string, 20> OBJECT_NAMES = { "Oneway", "Highway Entrance", "Stop Sign", "Roundabout", "Park", "Crosswalk", "No Entry", "Highway Exit", "Priority", "Lights", "Block", "Pedestrian", "Car", "Green Light", "Yellow Light", "Red Light", "None", "Fog", "Tunnel", "Ramp" };
    const std::array<double, 20> OBJECT_HEIGHTS = {
        0.1774, 0.1774, 0.1774, 0.1774, 0.1774, 0.1774, 0.1774, 0.1774, 0.1774,
        0.1944, 0.125, 0.1, 0.1, 0.1944, 0.1944, 0.1944, 0.1, 1.0, 1.0, 1.0};
    enum MANEUVER_DIRECTION {
        LEFT,
        STRAIGHT,
        RIGHT
    };
    enum DETECTED_CAR_STATE {
        SAME_LANE,
        ADJACENT_LANE,
        OPPOSITE_LANE,
        NOT_SURE
    };

    static const std::vector<double> NORTH_FACING_LANE_CENTERS = {0.579612+ofs6, 2.744851+ofs6, 4.9887+ofs6, 6.51+ofs6, 6.8507+ofs6, 16.954+ofs6, 15.532+ofs6, 16.1035+ofs6};
    static const std::vector<double> SOUTH_FACING_LANE_CENTERS = {0.50684-ofs6, 2.667-ofs6, 4.9156-ofs6, 15.165279+ofs6, 15.1632+ofs6, 15.7356+ofs6};
    static const std::vector<double> EAST_FACING_LANE_CENTERS = {12.904+ofs6, 10.5538-ofs6, 0.503891-ofs6, 1.072 - ofs6, 3.79216-ofs6, 6.6816-ofs6, 10.5538-ofs6, 11.6588+ofs6};
    static const std::vector<double> WEST_FACING_LANE_CENTERS = {13.314624+ofs6, 10.633+ofs6, 3.86375+ofs6, 0.58153+ofs6, 1.072+ofs6, 3.8661+ofs6, 6.753+ofs6, 13.278+ofs6, 12.032+ofs6};
    
    static constexpr double sign_ofs1 = 0.056;
    static constexpr double sign_ofs2 = 0.051;
    static const std::vector<std::vector<double>> ALL_SIGNS = {
        // SOUTH_FACING_SIGNS
        {{15.46-ofs6*2-pole_size-sign_ofs1, 4.58+sign_ofs2, -M_PI/2}},
        {{15.165376-pole_size-sign_ofs1, 1.4972+sign_ofs2, -M_PI/2}},
        {{4.9156-ofs6*2-pole_size-sign_ofs1, 4.581664+sign_ofs2, -M_PI/2}},
        {{4.9143-ofs6*2-pole_size-sign_ofs1, 1.3+sign_ofs2, -M_PI/2}},
        {{2.667-ofs6*2-pole_size-sign_ofs1, 1.3+sign_ofs2, -M_PI/2}},
        {{0.50684-ofs6*2-pole_size-sign_ofs1, 4.5849+sign_ofs2, -M_PI/2}},
        {{0.50684-ofs6*2-pole_size-sign_ofs1, 7.470675+sign_ofs2, -M_PI/2}},
        {{0.50684-ofs6*2-pole_size-sign_ofs1, 10.584+sign_ofs2, -M_PI/2}},

        // NORTH_FACING_SIGNS
        {{0.579612+ofs6*2+pole_size+sign_ofs1, 9.0727-sign_ofs2, M_PI/2}},
        {{0.579612+ofs6*2+pole_size+sign_ofs1, 5.96247-sign_ofs2, M_PI/2}},
        {{0.579612+ofs6*2+pole_size+sign_ofs1, 3.07145-sign_ofs2, M_PI/2}},
        {{2.744851+ofs6*2+pole_size+sign_ofs1, 5.9603-sign_ofs2, M_PI/2}},
        {{4.9887+ofs6*2+pole_size+sign_ofs1, 5.958-sign_ofs2, M_PI/2}},
        {{4.9887+ofs6*2+pole_size+sign_ofs1, 3.07-sign_ofs2, M_PI/2}},
        {{6.4836+ofs6*2+pole_size+sign_ofs1, 3.44261-sign_ofs2, M_PI/2}},

        // WEST_FACING_SIGNS
        {{1.296+sign_ofs2, 3.86375+ofs6*2+pole_size+sign_ofs1, M_PI}},
        {{5.71+sign_ofs2, 3.8661+ofs6*2+pole_size+sign_ofs1, M_PI}}, // remove temporarily
        {{5.708+sign_ofs2, 6.753+ofs6*2+pole_size+sign_ofs1, M_PI}},
        {{3.4547+sign_ofs2, 6.7545+ofs6*2+pole_size+sign_ofs1, M_PI}},
        {{1.296+sign_ofs2, 6.754754+ofs6*2+pole_size+sign_ofs1, M_PI}},
        {{7.568552+sign_ofs2, 3.8674+ofs6*2+pole_size+sign_ofs1, M_PI}},
        {{3.45624+sign_ofs2, 0.58153+ofs6*2+pole_size+sign_ofs1, M_PI}},
        {{16.2485+sign_ofs2, 3.8678+ofs6*2+pole_size+sign_ofs1, M_PI}}, // remove temporarily

        // EAST_FACING_SIGNS
        {{1.95075-sign_ofs2, 0.503891-ofs6*2-pole_size-sign_ofs1, 0}},
        {{1.95-sign_ofs2, 6.6816-ofs6*2-pole_size-sign_ofs1, 0}}, 
        {{4.19476-sign_ofs2, 6.681-ofs6*2-pole_size-sign_ofs1, 0}}, 
        {{4.19476-sign_ofs2, 3.79216-ofs6*2-pole_size-sign_ofs1, 0}}, 
        {{4.194644-sign_ofs2, 0.503836-ofs6*2-pole_size-sign_ofs1, 0}}, 
        {{14.7386-sign_ofs2, 1.07135-ofs6*2-pole_size-sign_ofs1, 0}}, 
    };

    // intersection coordinates
    static const std::vector<std::vector<double>> SOUTH_FACING_INTERSECTIONS = {
        {{15.46-ofs6, 4.58-hsw, -M_PI/2}}, // allsigns0
        {{15.165376+ofs6, 1.4972-hsw, -M_PI/2}}, // allsigns1
        {{4.9156-ofs6, 4.581664-hsw, -M_PI/2}}, // allsigns2
        {{4.9143-ofs6, 1.3-hsw, -M_PI/2}}, // allsigns3
        {{2.667-ofs6, 1.3-hsw, -M_PI/2}}, // allsigns4
        {{0.50684-ofs6, 4.5849-hsw, -M_PI/2}}, // allsigns5
        {{0.50684-ofs6, 7.470675-hsw, -M_PI/2}}, // allsigns6
        {{0.50684-ofs6, 10.584-hsw, -M_PI/2}}, // allsigns7
        {{2.67-ofs6, 4.584-hsw, -M_PI/2}}, // alllights0
        {{16.075438-ofs6, 11.684077-hsw, -M_PI/2}}, // allroundabouts0
        {{15.3, 3, -M_PI/2}}, // allcrosswalks0
        {{4.75, 7.92, -M_PI/2}}, // allcrosswalks1
    };
    static const std::vector<std::vector<double>> NORTH_FACING_INTERSECTIONS = {
        {{0.579612+ofs6, 9.0727+hsw, M_PI/2}}, // allsigns8
        {{0.579612+ofs6, 5.96247+hsw, M_PI/2}}, // allsigns9
        {{0.579612+ofs6, 3.07145+hsw, M_PI/2}}, // allsigns10
        {{2.744851+ofs6, 5.9603+hsw, M_PI/2}}, // allsigns11
        {{4.9887+ofs6, 5.958+hsw, M_PI/2}}, // allsigns12
        {{4.9887+ofs6, 3.07+hsw, M_PI/2}}, // allsigns13
        {{6.77784-ofs6, 3.44261+hsw, M_PI/2}}, // allsigns14
        {{2.744851+ofs6, 3.07145+hsw, M_PI/2}}, // alllights1
        {{16.104+ofs6, 9.5053+hsw, M_PI/2}}, // allroundabouts1
        {{17.11, 2.367, M_PI/2}}, // allcrosswalks2
        {{5.15, 7.38, M_PI/2}}, // allcrosswalks3
    };
    static const std::vector<std::vector<double>> WEST_FACING_INTERSECTIONS = {
        {{1.296-hsw, 3.86375+ofs6, M_PI}}, // allsigns15
        {{5.71-hsw, 3.8661+ofs6, M_PI}}, // allsigns16
        {{5.708-hsw, 6.753+ofs6, M_PI}}, // allsigns17
        {{3.4547-hsw, 6.7545+ofs6, M_PI}}, // allsigns18
        {{1.296-hsw, 6.754754+ofs6, M_PI}}, // allsigns19
        {{7.568552-hsw, 3.8674+ofs6, M_PI}}, // allsigns20
        {{3.45624-hsw, 0.58153+ofs6, M_PI}}, // allsigns21
        {{16.2485-hsw, 3.8678+ofs6, M_PI}}, // allsigns22
        {{3.4543-hsw, 3.865637+ofs6, M_PI}}, // alllights2
        {{17.1571-hsw, 10.633+ofs6, M_PI}}, // allroundabouts2
        {{9.5, 4, M_PI}}, // allcrosswalks4
        {{1.74, 10, M_PI}}, // allcrosswalks5
    };
    static const std::vector<std::vector<double>> EAST_FACING_INTERSECTIONS = {
        {{1.95075+hsw, 0.503891-ofs6, 0}}, // allsigns23
        {{1.95+hsw, 6.6816-ofs6, 0}}, // allsigns24
        {{4.19476+hsw, 6.681-ofs6, 0}}, // allsigns25
        {{4.19476+hsw, 3.79216-ofs6, 0}}, // allsigns26
        {{4.194644+hsw, 0.503836-ofs6, 0}}, // allsigns27
        {{14.7386+hsw, 1.07135-ofs6, 0}}, // allsigns28
        {{1.95075+hsw, 3.794-ofs6, 0}}, // alllights3
        {{14.983+hsw, 10.5538-ofs6, 0}}, //allroundabouts3
        {{8.2, 0.92, 0}}, // allcrosswalks6
        {{1.22, 9.62, 0}}, // allcrosswalks7
    };

    static const std::vector<std::vector<double>> EMPTY = {};

    static const std::vector<std::vector<double>> ALL_LIGHTS = {
        // {{2.67-ofs6*2-pole_size-sign_ofs1, 4.584+sign_ofs2, -M_PI/2}},
        // {{2.744851+ofs6*2+pole_size+sign_ofs1, 3.07145-sign_ofs2, M_PI/2}},
        // {{3.4543+sign_ofs2, 3.865637+ofs6*2+pole_size+sign_ofs1, M_PI}},
        // {{1.95075-sign_ofs2, 3.794-ofs6*2-pole_size-sign_ofs1, 0}},
        // Ferreol's measurements
        {{2.216, 4.601, -M_PI/2}},
        {{3.195, 3.031, M_PI/2}},
        {{3.541, 3.865637+ofs6*2+pole_size+sign_ofs1 + 0.05, M_PI}},
        {{1.906, 3.347, 0}},
    };
    static const std::vector<std::vector<double>> ALL_HIGHWAYENTRANCES = {
        {{14.362,11.028, M_PI}},
        {{6.332, 11.551, 0}},
    };
    static const std::vector<std::vector<double>> ALL_HIGHWAYEXITS = {
        {{6.488, 12.474, M_PI}},
        {{14.233, 10.066, 0}},
    };
    static const std::vector<std::vector<double>> ALL_ONEWAYS = {
        {{5.391, 0.104, 0}},
        {{15.634, 0.678, 0}},
        {{14.634, 4.268, M_PI}},
        {{15.049, 3.388, -M_PI/2}},
    };
    // PARKING SIGN COORDINATES
    static constexpr double park_ofs1_left = 0.009, park_ofs1_right = 0.016;
    static constexpr double park_ofs2 = 0.05325;
    static const std::vector<std::vector<double>> PARKING_SIGN_POSES1 = {{
        {{8.99-park_ofs2, 0.703367-park_ofs1_right, 0}}, 
        // {{8.99-park_ofs2, 1.1522+park_ofs1_left, 0}}
    }};
    static const std::vector<std::vector<double>> PARKING_SIGN_POSES2 = {{
        {{8.99-park_ofs2, 1.1522+park_ofs1_left, 0}}
    }};
    
    // ROUNDABOUT COORDINATES
    static constexpr double rdb_ofs1 = 0.107834;
    static constexpr double rdb_ofs2 = 0.05361;
    static const std::vector<std::vector<double>> ALL_ROUNDABOUTS = {
        {{15.737-rdb_ofs1, 11.690741+rdb_ofs2, -M_PI/2}},
        {{16.4+rdb_ofs1, 9.52-rdb_ofs2, M_PI/2}},
        {{17.164+rdb_ofs2, 10.928+rdb_ofs1, M_PI}},
        {{14.9777-rdb_ofs2, 10.263-rdb_ofs1, 0}}
    };

    // CROSSWALK COORDINATES
    static constexpr double cw_ofs2 = 0.025;
    static constexpr double cw_ofs1 = 0.028 + pole_size;
    static const std::vector<std::vector<double>> ALL_CROSSWALKS = {
        {{15.166-cw_ofs1, 3.01+cw_ofs2, -M_PI/2}},
        {{4.6255-cw_ofs1, 7.9375+cw_ofs2, -M_PI/2}},
        {{17.253+cw_ofs1, 2.313-cw_ofs2, M_PI/2}},
        {{5.371+cw_ofs1, 7.3775-cw_ofs2, M_PI/2}},
        {{9.521+cw_ofs2, 4.157+cw_ofs1, M_PI}},
        {{1.76+cw_ofs2, 10.16+cw_ofs1, M_PI}},
        {{8.1675-cw_ofs2, 0.7827-cw_ofs1, 0}},
        {{1.196-cw_ofs2, 9.427-cw_ofs1, 0}},
    };

    //utils
    static constexpr int NUM_VALUES_PER_OBJECT = 7;
    enum SignValues { x1, y1, x2, y2, distance, confidence, id};
    enum LOCALIZATION_SOURCE {
        ODOM,
        EKF
    };
    static constexpr std::array<double, 4> CAMERA_PARAMS = {
        607.40564, 
        607.40564, 
        320, 
        240
    };
    static constexpr std::array<double, 4> CAMERA_PARAMS_REAL = {
        607.40564,  // fx
        607.05829,  // fy
        322.97223,  // cx
        244.39398   // cy
    };
    static constexpr std::array<double, 6> REALSENSE_TF = {-0.090, -0.032, 0.260, -0.009774, 1.85*M_PI/180.0, 0.011}; // x, y, z, roll, pitch, yaw
    static constexpr std::array<double, 6> REALSENSE_TF_REAL = {-0.090, -0.032, 0.260, -0.009774, 1.85*M_PI/180.0, 0.011};
    static constexpr double CAM_TO_CAR_FRONT = 0.30;
}

#endif // VEHICLE_CONSTANTS_H
