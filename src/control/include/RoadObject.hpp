#pragma once
#include <string>
#include <array>
#include <std_msgs/Float32MultiArray.h>
#include "utils/constants.h"

#include <cmath>
#include <algorithm>

using namespace VehicleConstants;
class RoadObject {
public:
    static inline int OBJECT_COUNT = 0;

    static inline const std::array<std::array<double, 2>, 16> OBJECT_SIZE = {
        std::array<double, 2>{0.1, 0.1}, 
        std::array<double, 2>{0.1, 0.1}, 
        std::array<double, 2>{0.1, 0.1}, 
        std::array<double, 2>{0.1, 0.1}, 
        std::array<double, 2>{0.1, 0.1}, 
        std::array<double, 2>{0.1, 0.1}, 
        std::array<double, 2>{0.1, 0.1}, 
        std::array<double, 2>{0.1, 0.1}, 
        std::array<double, 2>{0.1, 0.1}, 
        std::array<double, 2>{0.12, 0.12}, 
        std::array<double, 2>{0.1, 0.1}, 
        std::array<double, 2>{0.1, 0.1}, 
        std::array<double, 2>{0.464, 0.1885}, 
        std::array<double, 2>{0.12, 0.12}, 
        std::array<double, 2>{0.12, 0.12}, 
        std::array<double, 2>{0.12, 0.12}
    };

    RoadObject(OBJECT type, double x, double y, double yaw, double confidence, double speed)
        : id(OBJECT_COUNT++), type(type), x(x), y(y), yaw(yaw), speed(speed), 
        confidence(confidence), detection_count(1), last_detection_time(ros::Time::now()) 
    {
        name = OBJECT_NAMES[type];
        if (type == OBJECT::CAR) {
            while (yaw > M_PI) yaw -= 2 * M_PI; // Normalize yaw to [-pi, pi]
        }
    }

    RoadObject(OBJECT type, double x, double y, double yaw, double confidence)
        : RoadObject(type, x, y, yaw, confidence, 0) 
    {}

    RoadObject() {
        RoadObject(OBJECT::CAR, 0, 0, 0, 0, 1.);
    }

    RoadObject(int type, double x, double y, double yaw, double confidence, double speed)
    : RoadObject(static_cast<OBJECT>(type), x, y, yaw, confidence, speed) {}

    RoadObject(int type, double x, double y, double yaw, double confidence)
    : RoadObject(static_cast<OBJECT>(type), x, y, yaw, confidence) {}
    
    virtual ~RoadObject() { // virtual makes sure this class is polymorphic
        OBJECT_COUNT--;
    }

    OBJECT type;
    void printRoadObject() {
        std::cout << "RoadObject: " << name << " x: " << x << " y: " << y << " yaw: " << yaw << " speed: " << speed << " confidence: " << confidence << std::endl;
    }

    int id;
    double x;
    double y;
    double z = 0;
    double yaw;
    double speed;
    std::string name;
    int detection_count = 0;
    double confidence;
    ros::Time last_detection_time;
    double lifetime = 3600.0;  // 1 hour
    
    virtual bool is_same_object(double x, double y) {
        int type = static_cast<int>(this->type);
        if (type == OBJECT::CAR) {
            return (std::abs(this->x - x) < OBJECT_SIZE[type][0]*2 && std::abs(this->y - y) < OBJECT_SIZE[type][1]*2);
        } else {
            // compute squared distance
            double dx = this->x - x;
            double dy = this->y - y;
            return (dx * dx + dy * dy < 0.537 * 0.537);
        }
    }
    virtual void merge(double x, double y, double yaw, double confidence, double speed=0, double z = 0) {
        if(confidence >= 1.) {
            this->x = x;
            this->y = y;
            this->yaw = yaw;
            this->speed = 0;
            this->confidence = confidence;
            this->detection_count = 1;
            this->z = z;
            return;
        }
        this->x = (this->x * this->detection_count + x) / (this->detection_count + 1);
        this->y = (this->y * this->detection_count + y) / (this->detection_count + 1);
        this->yaw = (this->yaw * this->detection_count + yaw) / (this->detection_count + 1);
        this->z = (this->z * this->detection_count + z) / (this->detection_count + 1);
        // this->speed = (this->speed * this->detection_count + speed) / (this->detection_count + 1);
        this->speed = 0;
        this->confidence = (this->confidence * this->detection_count + confidence) / (this->detection_count + 1);
        this->detection_count++;
        this->last_detection_time = ros::Time::now();
    }
    void update(double x, double y, double yaw, double speed, double z) {
        this->x = x;
        this->y = y;
        this->yaw = yaw;
        this->speed = speed;
        this->z = z;
        this->last_detection_time = ros::Time::now();
    }
    static std_msgs::Float32MultiArray create_msg(const std::vector<std::shared_ptr<RoadObject>>& objects) {
        if (OBJECT_COUNT < 1) {
            return std_msgs::Float32MultiArray();
        }
        std_msgs::Float32MultiArray msg;
        msg.layout.data_offset = 0;
        for (const auto& obj : objects) {
            msg.data.push_back(obj->type);
            msg.data.push_back(obj->x);
            msg.data.push_back(obj->y);
            msg.data.push_back(obj->yaw);
            msg.data.push_back(obj->speed);
            msg.data.push_back(obj->confidence);
            msg.data.push_back(obj->z);
        }
        return msg;
    }
    static void cleanup_stale_objects(std::vector<std::shared_ptr<RoadObject>>& road_objects) {
        ros::Time current_time = ros::Time::now();
        // Remove objects that haven't been updated for over 5 seconds.
        road_objects.erase(
            std::remove_if(road_objects.begin(), road_objects.end(),
                [current_time](const std::shared_ptr<RoadObject>& obj) {
                    return (current_time - obj->last_detection_time).toSec() > obj->lifetime;
                }),
            road_objects.end());
    }
};
