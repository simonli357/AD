#pragma once
#include <string>
#include <array>
#include <std_msgs/Float32MultiArray.h>
#include "utils/constants.h"

#include <cmath>

using namespace VehicleConstants;
class RoadObject {
public:
    static int OBJECT_COUNT;
    static const std::array<std::array<double, 2>, 16> OBJECT_SIZE;

    RoadObject();
    ~RoadObject();

    RoadObject(OBJECT type, double x, double y, double yaw, double speed, double confidence);

    RoadObject(int type, double x, double y, double yaw, double speed, double confidence)
    : RoadObject(static_cast<OBJECT>(type), x, y, yaw, speed, confidence) {}
    
    OBJECT type;
    void printRoadObject();

    int id;
    double x;
    double y;
    double z;
    double yaw;
    double speed;
    std::string name;
    int detection_count = 0;
    double confidence;
    ros::Time last_detection_time;
    
    bool is_same_object(double x, double y) {
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
    void merge(double x, double y, double yaw, double speed, double confidence, double z = 0) {
        if(confidence >= 1.) {
            this->x = x;
            this->y = y;
            this->yaw = yaw;
            this->speed = speed;
            this->confidence = confidence;
            this->detection_count = 1;
            this->z = z;
            return;
        }
        this->x = (this->x * this->detection_count + x) / (this->detection_count + 1);
        this->y = (this->y * this->detection_count + y) / (this->detection_count + 1);
        this->yaw = (this->yaw * this->detection_count + yaw) / (this->detection_count + 1);
        this->z = (this->z * this->detection_count + z) / (this->detection_count + 1);
        this->speed = (this->speed * this->detection_count + speed) / (this->detection_count + 1);
        this->confidence = (this->confidence * this->detection_count + confidence) / (this->detection_count + 1);
        this->detection_count++;
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
};
