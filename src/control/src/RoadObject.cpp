#include "RoadObject.hpp"

int RoadObject::OBJECT_COUNT = 0;

const std::array<std::string, 16> RoadObject::OBJECT_NAMES = {
    "Oneway", "Highway Entrance", "Stop Sign", "Roundabout", "Park", "Crosswalk", "No Entry", "Highway Exit", "Priority", "Lights", "Block", "Pedestrian", "Car", "Green Light", "Yellow Light", "Red Light",
};

const std::array<std::array<double, 2>, 16> RoadObject::OBJECT_SIZE = { // width, length
    std::array<double, 2>{0.1, 0.1}, 
    std::array<double, 2>{0.1,   0.1}, 
    std::array<double, 2>{0.1,  0.1}, 
    std::array<double, 2>{0.1,   0.1},
    std::array<double, 2>{0.1,   0.1}, 
    std::array<double, 2>{0.1,   0.1}, 
    std::array<double, 2>{0.1,   0.1}, 
    std::array<double, 2>{0.1,   0.1}, 
    std::array<double, 2>{0.1,   0.1},
    std::array<double, 2>{0.12,   0.12}, 
    std::array<double, 2>{0.1,   0.1}, 
    std::array<double, 2>{0.1,   0.1}, 
    std::array<double, 2>{0.464,   0.1885},
    std::array<double, 2>{0.12,   0.12}, 
    std::array<double, 2>{0.12,   0.12}, 
    std::array<double, 2>{0.12,   0.12} 
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
