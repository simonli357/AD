#include "RoadObject.hpp"

int RoadObject::OBJECT_COUNT = 0;

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

RoadObject::RoadObject(OBJECT type, double x, double y, double yaw, double speed, double confidence)
    : id(OBJECT_COUNT++), type(type), x(x), y(y), yaw(yaw), speed(speed), 
    confidence(confidence), detection_count(1), last_detection_time(ros::Time::now()) 
{
    name = OBJECT_NAMES[type];
    if (type == OBJECT::CAR) {
        while (yaw > M_PI) yaw -= 2 * M_PI; // Normalize yaw to [-pi, pi]
    }
}

RoadObject::RoadObject() {
    RoadObject(OBJECT::CAR, 0, 0, 0, 0, 1.);
}
