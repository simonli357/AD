#include "RoadObject.hpp"

int RoadObject::OBJECT_COUNT = 0;

const std::array<std::string, 13> RoadObject::OBJECT_NAMES = {
    "Car",
    "Pedestrian",
    "Traffic_Light",
    "Stop_Sign",
    "Parking_Sign",
    "Crosswalk",
    "Roundabout",
    "Priority",
    "No_Entry",
    "One_Way",
    "Highway_Entrance",
    "Highway_Exit"
};

const std::array<std::array<double, 2>, 13> RoadObject::OBJECT_SIZE = { // width, length
    std::array<double, 2>{0.464, 0.1885}, // Car
    std::array<double, 2>{0.1,   0.1}, // Pedestrian
    std::array<double, 2>{0.12,  0.12}, // Traffic_Light
    std::array<double, 2>{0.1,   0.1}, // Stop_Sign
    std::array<double, 2>{0.1,   0.1}, // Parking_Sign
    std::array<double, 2>{0.1,   0.1}, // Crosswalk
    std::array<double, 2>{0.1,   0.1}, // Roundabout
    std::array<double, 2>{0.1,   0.1}, // Priority
    std::array<double, 2>{0.1,   0.1}, // No_Entry
    std::array<double, 2>{0.1,   0.1}, // One_Way
    std::array<double, 2>{0.1,   0.1}, // Highway_Entrance
    std::array<double, 2>{0.1,   0.1}  // Highway_Exit
};

RoadObject::~RoadObject() {
    OBJECT_COUNT--;
}

RoadObject::RoadObject(ObjectType type, double x, double y, double yaw, double speed, double confidence)
    : id(OBJECT_COUNT++), type(type), x(x), y(y), yaw(yaw), speed(speed), confidence(confidence), detection_count(1) {
    name = OBJECT_NAMES[type];
    if (type == ObjectType::CAR) {
        while (yaw > M_PI) yaw -= 2 * M_PI; // Normalize yaw to [-pi, pi]
    }
}

RoadObject::RoadObject() {
    RoadObject(ObjectType::CAR, 0, 0, 0, 0, 1.);
}
