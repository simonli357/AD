#pragma once
#include <string>
#include <array>
#include <vector>
#include <cmath>
#include <algorithm>
#include <deque>
#include <memory>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include "utils/constants.h"

using namespace VehicleConstants;

namespace Tracking {

struct TrackingParams {
    double association_radius;
    double max_speed;
    double base_lifetime;
};

static const std::array<TrackingParams, 17> OBJECT_TRACKING_PARAMS = {{
    {MIN_SIGN_DIST, 0.0, 3600},  // ONEWAY
    {MIN_SIGN_DIST, 0.0, 3600},  // HIGHWAYENTRANCE
    {MIN_SIGN_DIST, 0.0, 3600},  // STOPSIGN
    {MIN_SIGN_DIST, 0.0, 3600},  // ROUNDABOUT
    {MIN_SIGN_DIST, 0.0, 3600},  // PARK
    {MIN_SIGN_DIST, 0.0, 3600},  // CROSSWALK
    {MIN_SIGN_DIST, 0.0, 3600},  // NOENTRY
    {MIN_SIGN_DIST, 0.0, 3600},  // HIGHWAYEXIT
    {MIN_SIGN_DIST, 0.0, 3600},    // PRIORITY
    {MIN_SIGN_DIST, 0.0, 3600},    // LIGHTS
    {MIN_SIGN_DIST, 0.0, 3600},    // BLOCK
    {0.075, 0.2, 5},    // PEDESTRIAN
    {CAR_WIDTH, 0.5, 5},    // CAR
    {MIN_SIGN_DIST, 0.0, 10},    // GREENLIGHT
    {MIN_SIGN_DIST, 0.0, 10},    // YELLOWLIGHT
    {MIN_SIGN_DIST, 0.0, 10},    // REDLIGHT
    {MIN_SIGN_DIST, 0.0, 1}      // NONE
}};

inline int OBJECT_COUNT = 0;

class RoadObject {
public:
    int id;
    OBJECT type;
    std::string name;

    double x, y, z = 0;
    double yaw = 0.0;
    double confidence;
    double cumulative_confidence = 0;
    int detection_count = 0;
    double speed;

    double lifetime;
    ros::Time last_detection_time;

    std::deque<std::array<double, 3>> position_history;  // x, y, confidence
    static constexpr size_t HISTORY_SIZE = 5;

    RoadObject(OBJECT type, double x, double y, double confidence)
        : id(OBJECT_COUNT++), type(type), x(x), y(y), yaw(0), confidence(confidence), speed(0) {
        name = OBJECT_NAMES[type];
        detection_count = 1;
        cumulative_confidence = confidence;
        last_detection_time = ros::Time::now();
        lifetime = OBJECT_TRACKING_PARAMS[static_cast<int>(type)].base_lifetime;
        position_history.push_back({x, y, confidence});
    }

    virtual ~RoadObject() {
        OBJECT_COUNT--;
    }

    virtual bool is_same_object(double new_x, double new_y) const {
        double dx = x - new_x;
        double dy = y - new_y;
        double distance = std::hypot(dx, dy);
        return distance <= OBJECT_TRACKING_PARAMS[static_cast<int>(type)].association_radius;
    }

    virtual void update_yaw(double new_yaw) {
        yaw = new_yaw;
    }
    virtual void merge(double new_x, double new_y, double new_conf) {
        double alpha = new_conf / (confidence + new_conf);
        x = (1 - alpha) * x + alpha * new_x;
        y = (1 - alpha) * y + alpha * new_y;
    
        confidence = (1 - alpha) * confidence + alpha * new_conf;
        cumulative_confidence += new_conf;
        detection_count++;
    
        last_detection_time = ros::Time::now();
    
        lifetime = std::min(lifetime + (0.2 * new_conf), OBJECT_TRACKING_PARAMS[static_cast<int>(type)].base_lifetime);
    
        position_history.push_back({x, y, confidence});
        if (position_history.size() > HISTORY_SIZE) {
            position_history.pop_front();
        }
    }

    virtual void update(double x, double y) {
        this->x = x;
        this->y = y;
        last_detection_time = ros::Time::now();
    }
};

class SignObject : public RoadObject {
public:
    SignObject(OBJECT type, double x, double y, double confidence)
        : RoadObject(type, x, y, confidence) {
        speed = 0;
    }

    SignObject(OBJECT type, double x, double y, double confidence, double yaw)
        : SignObject(type, x, y, confidence) {
        this->yaw = yaw;
    }

    bool is_same_object(double new_x, double new_y) const override {
        return RoadObject::is_same_object(new_x, new_y);
    }

    void merge(double new_x, double new_y, double new_conf) override {
        RoadObject::merge(new_x, new_y, new_conf);
    }
};
    
class DynamicObject : public RoadObject {
public:
    double first_x, first_y;
    ros::Time first_detection_time;

    DynamicObject(OBJECT type, double x, double y, double confidence)
        : RoadObject(type, x, y, confidence),
            first_x(x), first_y(y), first_detection_time(ros::Time::now()) {
            
        if (type != OBJECT::CAR && type != OBJECT::PEDESTRIAN) {
            throw std::invalid_argument("DynamicObject must be either CAR or PEDESTRIAN");
        }
        speed = 0;
    }

    bool is_same_object(double new_x, double new_y) const override {
        double dt = (ros::Time::now() - last_detection_time).toSec();
        double pred_x = x + speed * std::cos(yaw) * dt;
        double pred_y = y + speed * std::sin(yaw) * dt;
        double distance = std::hypot(new_x - pred_x, new_y - pred_y);
        return distance <= OBJECT_TRACKING_PARAMS[static_cast<int>(type)].association_radius;
    }

    void merge(double new_x, double new_y, double new_conf) override {
        ros::Time current_time = ros::Time::now();
        double dt1 = (current_time - last_detection_time).toSec();
        double dt2 = (current_time - first_detection_time).toSec();
    
        // Estimate speed
        double inst_speed = (dt1 > 0) ? std::hypot(new_x - x, new_y - y) / dt1 : speed;
        double avg_speed = (dt2 > 0) ? std::hypot(new_x - first_x, new_y - first_y) / dt2 : inst_speed;
    
        double alpha = new_conf / (confidence + new_conf);
        double est_speed = (alpha * inst_speed + (1 - alpha) * avg_speed);
        speed = (1 - alpha) * speed + alpha * est_speed;
    
        // Yaw estimation from displacement
        double dx_inst = new_x - x;
        double dy_inst = new_y - y;
        double dx_avg = new_x - first_x;
        double dy_avg = new_y - first_y;
    
        bool valid_inst = std::hypot(dx_inst, dy_inst) > 1e-4;
        bool valid_avg = std::hypot(dx_avg, dy_avg) > 1e-4;
    
        if (valid_inst && valid_avg) {
            double yaw_inst = std::atan2(dy_inst, dx_inst);
            double yaw_avg = std::atan2(dy_avg, dx_avg);
    
            double sin_blend = (1 - alpha) * std::sin(yaw_avg) + alpha * std::sin(yaw_inst);
            double cos_blend = (1 - alpha) * std::cos(yaw_avg) + alpha * std::cos(yaw_inst);
            yaw = std::atan2(sin_blend, cos_blend);
        } else if (valid_inst) {
            yaw = std::atan2(dy_inst, dx_inst);
        } else if (valid_avg) {
            yaw = std::atan2(dy_avg, dx_avg);
        }
    
        // Position update (EMA)
        x = (1 - alpha) * x + alpha * new_x;
        y = (1 - alpha) * y + alpha * new_y;
    
        // Confidence
        confidence = (1 - alpha) * confidence + alpha * new_conf;
        cumulative_confidence += new_conf;
        detection_count++;
        last_detection_time = current_time;
    
        // Adaptive lifetime
        lifetime = std::min(lifetime + (0.2 * new_conf), OBJECT_TRACKING_PARAMS[static_cast<int>(type)].base_lifetime);
    
        // History
        position_history.push_back({x, y, confidence});
        if (position_history.size() > HISTORY_SIZE) {
            position_history.pop_front();
        }
    }
};

class EgoCarObject final: public DynamicObject {
public:
    double steer = 0;
    EgoCarObject(double x, double y, double yaw)
    : DynamicObject(OBJECT::CAR, x, y, 1.0) {
        this->z = 0;
        this->speed = 0;
        this->yaw = yaw;
        this->steer = 0;
    }
    
    void update(double x, double y, double yaw, double speed, double z, double steer) {
        this->x = x;
        this->y = y;
        this->yaw = yaw;
        this->speed = speed;
        this->z = z;
        this->steer = steer;
        last_detection_time = ros::Time::now();

        position_history.push_back({x, y, 1.0});
        if (position_history.size() > HISTORY_SIZE) {
            position_history.pop_front();
        }
    }
};

inline std::vector<std::shared_ptr<SignObject>> road_signs;
inline std::shared_ptr<EgoCarObject> ego_car;
inline std::vector<std::shared_ptr<EgoCarObject>> road_cars;
inline std::vector<std::shared_ptr<DynamicObject>> road_pedestrians;
inline std::shared_ptr<DynamicObject> create_ego_car(double x, double y, double yaw, double z, double confidence, double speed) {
    ego_car = std::make_shared<EgoCarObject>(x, y, yaw);
    return ego_car;
}
inline std::shared_ptr<RoadObject> create_object(OBJECT type, double x, double y, double yaw, double confidence) {
    switch (type) {
        case OBJECT::CAR:
            auto car = std::make_shared<DynamicObject>(type, x, y, confidence);
            road_cars.push_back(car);
            return car;
        case OBJECT::PEDESTRIAN:
            auto pedestrian = std::make_shared<DynamicObject>(type, x, y, confidence);
            road_pedestrians.push_back(pedestrian);
            return pedestrian;
        default:
            auto sign = std::make_shared<SignObject>(type, x, y, confidence, yaw);
            road_signs.push_back(sign);
            return sign;
    }
}

inline static std_msgs::Float32MultiArray ros_msg;

inline void reset_msg() {
    ros_msg.data.clear();
    ros_msg.layout.data_offset = 0;
}

inline void create_msg(const std::vector<std::shared_ptr<RoadObject>>& objects) {
    for (const auto& obj : objects) {
        ros_msg.data.push_back(static_cast<float>(obj->type));
        ros_msg.data.push_back(static_cast<float>(obj->x));
        ros_msg.data.push_back(static_cast<float>(obj->y));
        ros_msg.data.push_back(static_cast<float>(obj->yaw));
        ros_msg.data.push_back(static_cast<float>(obj->speed));
        ros_msg.data.push_back(static_cast<float>(obj->confidence));
        ros_msg.data.push_back(static_cast<float>(obj->z));
    }
}

inline const std_msgs::Float32MultiArray& get_msg() {
    return ros_msg;
}

inline void cleanup_stale_objects(std::vector<std::shared_ptr<RoadObject>>& road_objects) {
    ros::Time now = ros::Time::now();
    road_objects.erase(std::remove_if(road_objects.begin(), road_objects.end(),
        [now](const std::shared_ptr<RoadObject>& obj) {
            return (now - obj->last_detection_time).toSec() > obj->lifetime;
        }), road_objects.end());
}

}
        