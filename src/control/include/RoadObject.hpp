#pragma once
#include <string>
#include <array>
#include <std_msgs/Float32MultiArray.h>

#include <cmath>
class RoadObject {
public:
    enum ObjectType {
        CAR,
        PEDESTRIAN,
        TRAFFIC_LIGHT,
        STOP_SIGN,
        PARKING_SIGN,
        CROSSWALK,
        ROUNDABOUT,
        PRIORITY,
        NO_ENTRY,
        ONE_WAY,
        HIGHWAY_ENTRANCE,
        HIGHWAY_EXIT
    };

    static int OBJECT_COUNT;
    static const std::array<std::string, 13> OBJECT_NAMES;
    static const std::array<std::array<double, 2>, 13> OBJECT_SIZE;

    RoadObject();
    ~RoadObject();

    RoadObject(ObjectType type, double x, double y, double yaw, double speed, double confidence);

    RoadObject(int type, double x, double y, double yaw, double speed, double confidence)
    : RoadObject(static_cast<ObjectType>(type), x, y, yaw, speed, confidence) {}
    
    ObjectType type;
    void printRoadObject();

    int id;
    double x;
    double y;
    double yaw;
    double speed;
    std::string name;
    int detection_count = 0;
    double confidence;
    
    bool is_same_object(double x, double y) {
        printf("std::abs(this->x - x): %.3f, std::abs(this->y - y): %.3f\n", std::abs(this->x - x), std::abs(this->y - y));
        return (std::abs(this->x - x) < OBJECT_SIZE[type][0]*2 && std::abs(this->y - y) < OBJECT_SIZE[type][1]*2);
    }
    void merge(double x, double y, double yaw, double speed, double confidence) {
        this->x = (this->x * this->detection_count + x) / (this->detection_count + 1);
        this->y = (this->y * this->detection_count + y) / (this->detection_count + 1);
        this->yaw = (this->yaw * this->detection_count + yaw) / (this->detection_count + 1);
        this->speed = (this->speed * this->detection_count + speed) / (this->detection_count + 1);
        this->confidence = (this->confidence * this->detection_count + confidence) / (this->detection_count + 1);
        this->detection_count++;
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
        }
        return msg;
    }
};
