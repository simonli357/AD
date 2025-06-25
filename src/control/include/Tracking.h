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
#include "Tunable.h"
#include "KalmanFilter.h"
#include <atomic>
#include <mutex>

using namespace VehicleConstants;

namespace Tracking {

struct TrackingParams {
    double association_radius;
    double max_speed;
    double base_lifetime;
};

static const std::array<TrackingParams, 20> OBJECT_TRACKING_PARAMS = {{
    {MIN_SIGN_DIST, 0.0, 3600},  // ONEWAY
    {MIN_SIGN_DIST, 0.0, 3600},  // HIGHWAYENTRANCE
    {MIN_SIGN_DIST, 0.0, 3600},  // STOPSIGN
    {MIN_SIGN_DIST, 0.0, 3600},  // ROUNDABOUT
    {MIN_SIGN_DIST, 0.0, 3600},  // PARK
    {MIN_SIGN_DIST, 0.0, 3600},  // CROSSWALK
    {MIN_SIGN_DIST, 0.0, 3600},  // NOENTRY
    {0.573, 0.0, 3600},  // HIGHWAYEXIT
    {MIN_SIGN_DIST, 0.0, 3600},    // PRIORITY
    {MIN_SIGN_DIST, 0.0, 3600},    // LIGHTS
    {MIN_SIGN_DIST, 0.0, 3600},    // BLOCK
    {0.075, 0.2, 2.5},    // PEDESTRIAN
    {CAR_WIDTH, 0.5, 5},    // CAR
    {MIN_SIGN_DIST, 0.0, 10},    // GREENLIGHT
    {MIN_SIGN_DIST, 0.0, 10},    // YELLOWLIGHT
    {MIN_SIGN_DIST, 0.0, 10},    // REDLIGHT
    {MIN_SIGN_DIST, 0.0, 1},      // NONE
    {MIN_SIGN_DIST, 0.0, 3600},      // FOG
    {MIN_SIGN_DIST, 0.0, 3600},      // TUNNEL
    {MIN_SIGN_DIST, 0.0, 3600}      // RAMP
}};

inline std::atomic<int> OBJECT_COUNT = 0;
inline bool is_known_static_object(OBJECT obj) {
    return std::find(KNOWN_STATIC_OBJECTS.begin(), KNOWN_STATIC_OBJECTS.end(), obj) != KNOWN_STATIC_OBJECTS.end();
}
inline bool is_known_static_object(int obj) {
    return is_known_static_object(static_cast<OBJECT>(obj));
}

struct PositionSample {
    double x;
    double y;
    double confidence;
    ros::Time stamp;
};

class RoadObject {
public:
    int id;
    OBJECT type;
    std::string name;
    mutable std::mutex mtx;
    double x, y = 0;
    double yaw = 0.0;
    double z = 0.0;
    double confidence;
    double cumulative_confidence = 0;
    double cumulative_confidence_threshold = 2.0;
    int detection_count = 0;
    double speed = 0;

    double lifetime;
    ros::Time last_detection_time;
    ros::Time first_detection_time;

    std::deque<PositionSample> position_history;
    static constexpr std::size_t BASE_HISTORY_SIZE = 5;

    RoadObject(OBJECT type, double x, double y, double yaw, double confidence)
        : id(OBJECT_COUNT.fetch_add(1)), type(type), x(x), y(y), yaw(yaw), confidence(confidence),
         speed(0), cumulative_confidence_threshold(Tunable::cumulative_confidence_thresholds[static_cast<int>(type)]) {
        name = OBJECT_NAMES[type];
        detection_count = 1;
        cumulative_confidence = confidence;
        last_detection_time = ros::Time::now();
        first_detection_time = last_detection_time;
        lifetime = OBJECT_TRACKING_PARAMS[static_cast<int>(type)].base_lifetime;
        position_history.push_back({x, y, confidence, last_detection_time});
    }

    virtual ~RoadObject() {
        /* OBJECT_COUNT.fetch_sub(1); */
    }

    virtual bool is_same_object(double new_x, double new_y) const {
        std::lock_guard<std::mutex> lock(mtx);
        double dx = x - new_x;
        double dy = y - new_y;
        double distance = std::hypot(dx, dy);
        return distance <= OBJECT_TRACKING_PARAMS[static_cast<int>(type)].association_radius;
    }
    virtual bool is_same_type(OBJECT new_type) const {
        return type == new_type;
    }

    virtual void merge(double new_x, double new_y, double new_yaw, double new_conf, OBJECT new_type) {
        std::lock_guard<std::mutex> lock(mtx);
        if (new_type != type) {
            throw std::invalid_argument("Cannot merge different types of objects");
        }
        double alpha = new_conf / (confidence + new_conf);
        x = (1 - alpha) * x + alpha * new_x;
        y = (1 - alpha) * y + alpha * new_y;
        yaw = (1 - alpha) * yaw + alpha * new_yaw;
        
        confidence = (1 - alpha) * confidence + alpha * new_conf;
        cumulative_confidence += new_conf;
        detection_count++;
    
        last_detection_time = ros::Time::now();
    
        lifetime = std::min(lifetime + (0.2 * new_conf), OBJECT_TRACKING_PARAMS[static_cast<int>(type)].base_lifetime);
    
         position_history.push_back({x, y, confidence, last_detection_time});
        if (position_history.size() > BASE_HISTORY_SIZE) {
            position_history.pop_front();
        }
    }

    virtual void update(double x, double y) {
        std::lock_guard<std::mutex> lock(mtx);
        this->x = x;
        this->y = y;
        cumulative_confidence += 1.0;
        detection_count++;
        last_detection_time = ros::Time::now();
        lifetime = std::min(lifetime + 0.2, OBJECT_TRACKING_PARAMS[static_cast<int>(type)].base_lifetime);
        position_history.push_back({x, y, 1.0});
    }
    virtual void update_yaw(double new_yaw) {
        std::lock_guard<std::mutex> lock(mtx);
        yaw = new_yaw;
    }
    virtual void populate_msg(std_msgs::Float32MultiArray& msg) const {
        std::lock_guard<std::mutex> lock(mtx);
        if (cumulative_confidence < Tunable::cumulative_confidence_thresholds[static_cast<int>(type)]) {
            return;
        }
        msg.data.push_back(static_cast<float>(type));
        msg.data.push_back(static_cast<float>(x));
        msg.data.push_back(static_cast<float>(y));
        msg.data.push_back(static_cast<float>(yaw));
        msg.data.push_back(static_cast<float>(speed));
        msg.data.push_back(static_cast<float>(cumulative_confidence));
        msg.data.push_back(static_cast<float>(z));
        msg.data.push_back(static_cast<float>(id));
    }
};

class KnownStaticObject : public RoadObject {
public:
    // std::vector<double> gt_pose;
    Eigen::Vector3d gt_pose;
    double cumulative_confidence_thresh = 2.5;
    KnownStaticObject(OBJECT type, double x, double y, double yaw, double confidence, const std::vector<double>& gt_pose)
        : RoadObject(type, x, y, yaw, confidence), gt_pose(gt_pose[0], gt_pose[1], gt_pose[2]), 
            cumulative_confidence_thresh(Tunable::cumulative_confidence_thresholds[static_cast<int>(type)]) {
        if (!is_known_static_object(type)) {
            throw std::invalid_argument("KnownStaticObject Constructor(): KnownStaticObject must be a known static object");
        }
        RoadObject::update_yaw(gt_pose[2]);
    }

    KnownStaticObject(OBJECT type, const std::vector<double>& gt_pose)
        : KnownStaticObject(type, gt_pose[0], gt_pose[1], gt_pose[2], 0.0, gt_pose) {}


    bool is_same_object(double new_x, double new_y) const override {
        return RoadObject::is_same_object(new_x, new_y);
    }

    void merge(double new_x, double new_y, double new_yaw, double new_conf, OBJECT new_type) override {
        RoadObject::merge(new_x, new_y, new_yaw, new_conf, new_type);
        // ROS_INFO("Merging KnownStaticObject: id=%d, type=%s, detection_count=%d, new_conf=%.2f, cumulative_confidence=%.2f, elapsed_time=%.2f, avg conf per second=%.2f",
        //          id, OBJECT_NAMES[type].c_str(), detection_count, new_conf, cumulative_confidence, (ros::Time::now() - first_detection_time).toSec(), cumulative_confidence / (ros::Time::now() - first_detection_time).toSec());
        RoadObject::update_yaw(gt_pose[2]); // keep the ground truth yaw
    }

    void reset() {
        std::lock_guard<std::mutex> lock(mtx);
        RoadObject::x = gt_pose[0];
        RoadObject::y = gt_pose[1];
        RoadObject::yaw = gt_pose[2];
        RoadObject::confidence = 0.0;
        RoadObject::detection_count = 0;
        cumulative_confidence_thresh += Tunable::cumulative_confidence_thresholds[static_cast<int>(type)];
    }
};

class LightObject : public KnownStaticObject {
public:
    LightColor current_color = LightColor::UNDETERMINED;

    std::deque<std::pair<LightColor, double>> color_history;
    static constexpr size_t HISTORY_SIZE = 10; // need tune based on detection frequency

    // Timestamp when the current state was entered (for forced transitions).
    ros::Time state_start_time;

    // When detections have not been updated for this many seconds, clear history.
    static constexpr double RESET_HISTORY_THRESHOLD = 15.0;

    LightObject(OBJECT type, double x, double y, double yaw, double confidence, const std::vector<double>& gt_pose)
        : KnownStaticObject(OBJECT::LIGHTS, x, y, yaw, confidence, gt_pose)
    {
        state_start_time = ros::Time::now();
        last_detection_time = state_start_time;
        LightColor initial_color = type == OBJECT::GREENLIGHT ? LightColor::GREEN :
                                   type == OBJECT::YELLOWLIGHT ? LightColor::YELLOW :
                                   type == OBJECT::REDLIGHT ? LightColor::RED : 
                                   type == OBJECT::LIGHTS ? LightColor::UNDETERMINED :
                                   LightColor::UNDETERMINED;
        current_color = initial_color;
        color_history.push_back({initial_color, confidence});
    }

    bool is_same_type(OBJECT new_type) const override {
        return new_type == OBJECT::LIGHTS ||
               new_type == OBJECT::GREENLIGHT ||
               new_type == OBJECT::YELLOWLIGHT ||
               new_type == OBJECT::REDLIGHT;
    }

    LightColor aggregateDetection() const {
        double weight_green = 0.0, weight_yellow = 0.0, weight_red = 0.0, weight_undetermined = 0.0;
        for (const auto& entry : color_history) {
            switch (entry.first) {
                case LightColor::GREEN:         weight_green += entry.second; break;
                case LightColor::YELLOW:        weight_yellow += entry.second; break;
                case LightColor::RED:           weight_red += entry.second; break;
                case LightColor::UNDETERMINED:  weight_undetermined += entry.second; break;
            }
        }
        if (weight_green >= weight_yellow && weight_green >= weight_red && weight_green >= weight_undetermined)
            return LightColor::GREEN;
        else if (weight_yellow >= weight_green && weight_yellow >= weight_red && weight_yellow >= weight_undetermined)
            return LightColor::YELLOW;
        else if (weight_red >= weight_green && weight_red >= weight_yellow && weight_red >= weight_undetermined)
            return LightColor::RED;
        else
            return LightColor::UNDETERMINED;
    }

    void update_light_detection(LightColor detected, double detection_confidence) {
        ros::Time now = ros::Time::now();
        
        // If enough time passed since the last update, clear the history.
        if ((now - last_detection_time).toSec() > RESET_HISTORY_THRESHOLD) {
            color_history.clear();
            current_color = LightColor::UNDETERMINED;
            state_start_time = now;
        }
        last_detection_time = now;
        
        color_history.push_back({detected, detection_confidence});
        if (color_history.size() > HISTORY_SIZE) {
            color_history.pop_front();
        }
        
        // Aggregate the detections.
        LightColor aggregated_detection = aggregateDetection();
        double time_in_state = (now - state_start_time).toSec();
        
        // Time-based forced transition: e.g., if red for over n seconds, force a transition to green.
        if (current_color == LightColor::RED && time_in_state > 10.0) {
            current_color = LightColor::GREEN;
            state_start_time = now;
            return;
        }
        
        // Enforce proper transition order.
        LightColor allowed_next;
        switch (current_color) {
            case LightColor::GREEN:
                allowed_next = LightColor::YELLOW;
                break;
            case LightColor::YELLOW:
                allowed_next = LightColor::RED;
                break;
            case LightColor::RED:
                allowed_next = LightColor::GREEN;
                break;
            case LightColor::UNDETERMINED:
                // For undetermined, adopt the aggregated detection immediately.
                allowed_next = aggregated_detection;
                break;
        }
        
        // Change state only if the aggregated detection indicates the allowed next state.
        if (aggregated_detection != current_color && aggregated_detection == allowed_next) {
            current_color = allowed_next;
            state_start_time = now;
        }
        // Otherwise, keep the current state.
    }

    void merge(double new_x, double new_y, double new_yaw, double new_conf, OBJECT new_type) override {
        KnownStaticObject::merge(new_x, new_y, new_yaw, new_conf, this->type);
        LightColor detected_color = new_type == OBJECT::GREENLIGHT ? LightColor::GREEN :
                                    new_type == OBJECT::YELLOWLIGHT ? LightColor::YELLOW :
                                    new_type == OBJECT::REDLIGHT ? LightColor::RED : 
                                    new_type == OBJECT::LIGHTS ? LightColor::UNDETERMINED : 
                                    current_color;
        update_light_detection(detected_color, new_conf);
    }

    void populate_msg(std_msgs::Float32MultiArray& msg) const override {
        std::lock_guard<std::mutex> lock(mtx);
        auto type = OBJECT::LIGHTS;
        if (current_color == LightColor::GREEN) {
            type = OBJECT::GREENLIGHT;
        } else if (current_color == LightColor::YELLOW) {
            type = OBJECT::YELLOWLIGHT;
        } else if (current_color == LightColor::RED) {
            type = OBJECT::REDLIGHT;
        }
        if (cumulative_confidence < Tunable::cumulative_confidence_thresholds[static_cast<int>(type)]) {
            return;
        }
        msg.data.push_back(static_cast<float>(type));
        msg.data.push_back(static_cast<float>(x));
        msg.data.push_back(static_cast<float>(y));
        msg.data.push_back(static_cast<float>(yaw));
        msg.data.push_back(static_cast<float>(speed));
        msg.data.push_back(static_cast<float>(cumulative_confidence));
        msg.data.push_back(static_cast<float>(z));
        msg.data.push_back(static_cast<float>(id));
    }
};

class DynamicObject : public RoadObject {
public:
    double first_x, first_y, first_yaw;
    ros::Time last_prediction_time;
    std::unique_ptr<KalmanFilter> kf;
    bool use_kf = true;
    bool parked = false;
    static constexpr std::size_t HISTORY_SIZE = 60;

    DynamicObject(OBJECT type, double x, double y, double yaw, double confidence)
        : RoadObject(type, x, y, yaw, confidence),
            first_x(x), first_y(y), first_yaw(yaw), use_kf(Tunable::use_kf),
          kf(std::make_unique<KalmanFilter>(x, y, yaw, 0.0)), last_prediction_time(ros::Time::now())  
    {
        if (type != OBJECT::CAR && type != OBJECT::PEDESTRIAN) {
            throw std::invalid_argument("DynamicObject Constructor(): DynamicObject must be either CAR or PEDESTRIAN");
        }
        speed = 0;
    }

    bool is_same_object(double new_x, double new_y) const override {
        std::lock_guard<std::mutex> lock(mtx);
        double dt = (ros::Time::now() - last_detection_time).toSec();
        double pred_x = x;// + speed * std::cos(yaw) * dt;
        double pred_y = y;// + speed * std::sin(yaw) * dt;
        double distance = std::hypot(new_x - pred_x, new_y - pred_y);
        return distance <= OBJECT_TRACKING_PARAMS[static_cast<int>(type)].association_radius;
    }

    void merge(double new_x, double new_y, double yaw, double new_conf, OBJECT new_type) override {
        std::lock_guard<std::mutex> lock(mtx);
        if (new_type != type) {
            throw std::invalid_argument("Cannot merge different types of objects");
        }
        if (use_kf && kf && parked) {
            kf->update(new_x, new_y);
            x = kf->x();
            y = kf->y();
            yaw = kf->yaw();
            speed = kf->speed();
        } else {
            const ros::Time current_time = ros::Time::now();
            const double alpha = new_conf / (confidence + new_conf); 
            while (!position_history.empty() &&
                   (current_time - position_history.front().stamp).toSec() > 2.0)
            {
                position_history.pop_front();
            }
            double est_speed = speed;          // fallback
            double est_yaw   = yaw;
            if (!position_history.empty()) {
                const auto& oldest = position_history.front();
                const double dt = (current_time - oldest.stamp).toSec();
                if (dt > 0.0) {
                    const double dx = new_x - oldest.x;
                    const double dy = new_y - oldest.y;
                    est_speed = std::hypot(dx, dy) / dt;
                    est_yaw   = std::atan2(dy, dx);
                }
            }
            speed = est_speed;

            double dt1 = (current_time - last_detection_time).toSec();
            double dt2 = (current_time - first_detection_time).toSec();
        
            if (speed < 0.0537) {
                speed = 0;
            }
            double dx_avg = new_x - first_x;
            double dy_avg = new_y - first_y;
        
            bool valid_avg = std::hypot(dx_avg, dy_avg) > 1e-4;
            
            if (valid_avg) {
                if (speed > 0.04 && cumulative_confidence > cumulative_confidence_threshold && (last_detection_time - first_detection_time).toSec() > 1.0) {
                    double yaw_avg = std::atan2(dy_avg, dx_avg);
            
                    double sin_blend = std::sin(yaw_avg);
                    double cos_blend = std::cos(yaw_avg);
                    double yaw_new = std::atan2(sin_blend, cos_blend);
                    this->yaw = (1 - alpha) * this->yaw + alpha * yaw_new;
                } else {
                    this->yaw = first_yaw;
                }
            }
            
            x = (1 - alpha) * x + alpha * new_x;
            y = (1 - alpha) * y + alpha * new_y;
            if (parked) {
                this->yaw = 0.0;
                this->speed = 0.0;
            }
        }
        double alpha = new_conf / (confidence + new_conf);
        confidence = (1 - alpha) * confidence + alpha * new_conf;
        cumulative_confidence += new_conf;
        // std::cout << "new_conf: " << new_conf << ", cum conf: " << cumulative_confidence << std::endl;
        detection_count++;
    
        last_detection_time = ros::Time::now();
        if (last_detection_time - first_detection_time > ros::Duration(4.0)) {
            first_x = x;
            first_y = y;
            first_detection_time = last_detection_time;
        }
    
        lifetime = std::min(lifetime + (0.2 * new_conf), OBJECT_TRACKING_PARAMS[static_cast<int>(type)].base_lifetime);
    
        position_history.push_back({x, y, confidence, last_detection_time});
        if (position_history.size() > HISTORY_SIZE) {
            position_history.pop_front();
        }
    
    }
    void predict() {
        std::lock_guard<std::mutex> lock(mtx);
        if (use_kf && kf && !parked) {
            double dt = (ros::Time::now() - last_prediction_time).toSec();
            last_prediction_time = ros::Time::now();
            kf->predict(dt);
            x = kf->x();
            y = kf->y();
            yaw = kf->yaw();
            speed = kf->speed();
        }
    }

    void populate_msg(std_msgs::Float32MultiArray& msg) const override {
        std::lock_guard<std::mutex> lock(mtx);
        if (cumulative_confidence < Tunable::cumulative_confidence_thresholds[static_cast<int>(type)]) {
            return;
        }
        // std::cout << "cumulative_confidence: " << cumulative_confidence << ", thresh: " << Tunable::cumulative_confidence_thresholds[static_cast<int>(type)] << std::endl;
        msg.data.push_back(static_cast<float>(type));
        msg.data.push_back(static_cast<float>(x));
        msg.data.push_back(static_cast<float>(y));
        msg.data.push_back(static_cast<float>(yaw));
        msg.data.push_back(static_cast<float>(speed));
        msg.data.push_back(static_cast<float>(cumulative_confidence));
        msg.data.push_back(static_cast<float>(z));
        msg.data.push_back(static_cast<float>(id));
    }
};

class PedestrianObject : public DynamicObject {
    public:
    bool on_crosswalk = false;
    PedestrianObject(OBJECT type, double x, double y, double yaw, double confidence)
        : DynamicObject(type, x, y, yaw, confidence), on_crosswalk(false) {
        if (type != OBJECT::PEDESTRIAN) {
            throw std::invalid_argument("PedestrianObject Constructor(): PedestrianObject must be of type PEDESTRIAN");
            for(auto& crosswalk: ALL_CROSSWALKS) {
                if (std::hypot(crosswalk[0] - x, crosswalk[1] - y) < 0.537) {
                    on_crosswalk = true;
                    break;
                }
            }
            parked = false;
        }
    }
};

class EgoCarObject final: public DynamicObject {
public:
    double steer = 0;
    EgoCarObject(double x, double y, double yaw)
    : DynamicObject(OBJECT::CAR, x, y, yaw, 1.0) {
        this->z = 0;
        this->speed = 0;
        this->steer = 0;
    }
    
    void update(double x, double y, double yaw, double speed, double z, double steer) {
        std::lock_guard<std::mutex> lock(mtx);
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

    void populate_msg(std_msgs::Float32MultiArray& msg) const override {
        std::lock_guard<std::mutex> lock(mtx);
        msg.data.push_back(static_cast<float>(type));
        msg.data.push_back(static_cast<float>(x));
        msg.data.push_back(static_cast<float>(y));
        msg.data.push_back(static_cast<float>(yaw));
        msg.data.push_back(static_cast<float>(speed));
        msg.data.push_back(static_cast<float>(cumulative_confidence));
        msg.data.push_back(static_cast<float>(z));
        msg.data.push_back(static_cast<float>(id));
    }
};

inline constexpr std::array<int, 8> KNOWN_STATIC_SIGNS = {
    static_cast<int>(OBJECT::HIGHWAYENTRANCE),
    static_cast<int>(OBJECT::STOPSIGN),
    static_cast<int>(OBJECT::ROUNDABOUT),
    static_cast<int>(OBJECT::PARK),
    static_cast<int>(OBJECT::CROSSWALK),
    static_cast<int>(OBJECT::HIGHWAYEXIT),
    static_cast<int>(OBJECT::PRIORITY),
    static_cast<int>(OBJECT::LIGHTS)
};

inline std::shared_ptr<EgoCarObject> ego_car;
inline std::shared_ptr<RoadObject> fog;
inline std::shared_ptr<RoadObject> tunnel;
inline std::shared_ptr<RoadObject> ramp;
inline std::vector<std::shared_ptr<RoadObject>> road_objects;
inline std::vector<std::shared_ptr<KnownStaticObject>> road_known_static_objects;
inline std::vector<std::shared_ptr<DynamicObject>> road_cars;
inline std::vector<std::shared_ptr<DynamicObject>> road_pedestrians;
inline std::mutex container_mutex;

inline void predict_dynamic_objects() {
    for (auto& obj : road_cars) {
        obj->predict();
    }
    for (auto& obj : road_pedestrians) {
        obj->predict();
    }
}

inline std::vector<std::shared_ptr<RoadObject>> get_road_objects() {
    return road_objects; // return a copy of the vector
}
inline std::vector<std::shared_ptr<DynamicObject>> get_road_cars() {
    std::lock_guard<std::mutex> lock(container_mutex);
    return road_cars; // return a copy of the vector
}
inline std::vector<std::shared_ptr<DynamicObject>> get_road_pedestrians() {
    std::lock_guard<std::mutex> lock(container_mutex);
    return road_pedestrians; // return a copy of the vector
}
inline std::vector<std::shared_ptr<KnownStaticObject>> get_road_known_static_objects() {
    std::lock_guard<std::mutex> lock(container_mutex);
    return road_known_static_objects; // return a copy of the vector
}

inline std::vector<std::shared_ptr<RoadObject>>* get_road_objects(OBJECT type) {
    if (is_known_static_object(type)) {
        return reinterpret_cast<std::vector<std::shared_ptr<RoadObject>>*>(&road_known_static_objects);
    }
    switch (type) {
        case OBJECT::CAR:
            return reinterpret_cast<std::vector<std::shared_ptr<RoadObject>>*>(&road_cars);
        case OBJECT::PEDESTRIAN:
            return reinterpret_cast<std::vector<std::shared_ptr<RoadObject>>*>(&road_pedestrians);
        default:
            return &road_objects;
    }
}
template<typename T>
inline std::shared_ptr<T> get_most_recent_object(const std::vector<std::shared_ptr<T>>& objects) {
    if (objects.empty()) {
        return nullptr;
    }
    auto most_recent_it = std::max_element(
        objects.begin(),
        objects.end(),
        [](const std::shared_ptr<T>& a, const std::shared_ptr<T>& b) {
            return a->last_detection_time < b->last_detection_time;
        }
    );
    return (most_recent_it != objects.end()) ? *most_recent_it : nullptr;
}
inline void create_ego_car(double x, double y, double yaw) {
    ego_car = std::make_shared<EgoCarObject>(x, y, yaw);
    return;
}
inline void initialize_tracking() {
    std::lock_guard<std::mutex> lock(container_mutex);
    road_objects.clear();
    road_known_static_objects.clear();
    road_cars.clear();
    road_pedestrians.clear();
    OBJECT_COUNT = 0;
    create_ego_car(0, 0, 0);
    fog = std::make_shared<RoadObject>(OBJECT::FOG, 0.55, 0.5, 0, 1.0);
    tunnel = std::make_shared<RoadObject>(OBJECT::TUNNEL, 4.0, 9.93, 0, 1.0);
    ramp = std::make_shared<RoadObject>(OBJECT::RAMP, 3.65, 11.96, 0, 1.0);
}
inline void create_object(OBJECT type, double x, double y, double yaw, double confidence, bool parked = false) {
    switch (type) {
        case OBJECT::CAR: {
            auto car = std::make_shared<DynamicObject>(type, x, y, yaw, confidence);
            car->parked = parked;
            road_cars.push_back(car);
            return;
        }
        case OBJECT::PEDESTRIAN: {
            auto pedestrian = std::make_shared<DynamicObject>(type, x, y, yaw, confidence);
            road_pedestrians.push_back(pedestrian);
            return;
        }
        default: {
            auto object = std::make_shared<RoadObject>(type, x, y, yaw, confidence);
            road_objects.push_back(object);
            return;
        }
    }
}
inline void create_known_static_object(OBJECT type, double x, double y, double yaw, double confidence, const std::vector<double> &gt_pose) {
    // check every object in road_known_static_objects, if gt is same as existing object, merge, else create new
    // for (const auto& obj : road_known_static_objects) {
    //     if(!obj->is_same_type(type)) continue;
    //     Eigen::Vector2d new_gt_pose(gt_pose[0], gt_pose[1]);
    //     double gt_distance = (obj->gt_pose.head<2>() - new_gt_pose).norm();
    //     if (gt_distance < 0.05) {
    //         obj->merge(x, y, yaw, confidence, type);
    //         return;
    //     }
    // }
    if (type == OBJECT::LIGHTS || type == OBJECT::GREENLIGHT || type == OBJECT::YELLOWLIGHT || type == OBJECT::REDLIGHT) {
		std::cout << "create_known_static_object(): LIGHT!!!!!!!!!!! type: " << OBJECT_NAMES[type] << ", x: " << x << ", y: " << y << ", yaw: " << yaw << ", confidence: " << confidence << std::endl;
		auto light_obj = std::make_shared<LightObject>(type, x, y, yaw, confidence, gt_pose);
		road_known_static_objects.push_back(std::static_pointer_cast<KnownStaticObject>(light_obj));
	} else {
		auto obj = std::make_shared<KnownStaticObject>(type, x, y, yaw, confidence, gt_pose);
		road_known_static_objects.push_back(obj);
	}
}

inline static std_msgs::Float32MultiArray ros_msg;

inline void reset_msg() {
    ros_msg.data.clear();
    ros_msg.layout.data_offset = 0;
    ros_msg.layout.dim.clear();
}

inline std_msgs::Float32MultiArray& create_all_msgs() {
    reset_msg();
    if (ego_car) {
        ego_car->populate_msg(ros_msg);
    }
    for (const auto& obj : road_objects) {
        obj->populate_msg(ros_msg);
    }
    for (const auto& obj : road_known_static_objects) {
        obj->populate_msg(ros_msg);
    }
    for (const auto& obj : road_cars) {
        obj->populate_msg(ros_msg);
    }
    for (const auto& obj : road_pedestrians) {
        obj->populate_msg(ros_msg);
    }
    return ros_msg;
}

inline const std_msgs::Float32MultiArray& get_msg() {
    return ros_msg;
}

inline void cleanup_stale_objects() {
    ros::Time now = ros::Time::now();
    road_objects.erase(std::remove_if(road_objects.begin(), road_objects.end(),
        [now](const std::shared_ptr<RoadObject>& obj) {
            return (now - obj->last_detection_time).toSec() > obj->lifetime;
        }), road_objects.end());
    road_known_static_objects.erase(
        std::remove_if(
            road_known_static_objects.begin(),
            road_known_static_objects.end(),
            [now](const std::shared_ptr<KnownStaticObject>& obj) {
                return ((now - obj->last_detection_time).toSec() > obj->lifetime 
                        || ((now - obj->last_detection_time).toSec() > 3.0 && obj->cumulative_confidence < 1.5));
            }),
        road_known_static_objects.end());
    road_cars.erase(std::remove_if(road_cars.begin(), road_cars.end(),
        [now](const std::shared_ptr<DynamicObject>& obj) {
            return (now - obj->last_detection_time).toSec() > obj->lifetime;
        }), road_cars.end());
    road_pedestrians.erase(std::remove_if(road_pedestrians.begin(), road_pedestrians.end(),
        [now](const std::shared_ptr<DynamicObject>& obj) {
            return (now - obj->last_detection_time).toSec() > obj->lifetime;
        }), road_pedestrians.end());
}

} // end of namespace Tracking
        
