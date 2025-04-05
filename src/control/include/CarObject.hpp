#pragma once
#include "RoadObject.hpp"
#include <cmath>
#include <ros/ros.h>
#include <algorithm>

class CarObject : public RoadObject {
public:
    // Additional members to track the initial detection for speed estimation.
    double first_x, first_y;
    ros::Time first_detection_time;

    // Default constructor: sets type to CAR and initializes first-detection values.
    CarObject()
    : RoadObject(),
      first_x(0.0), first_y(0.0), first_detection_time(ros::Time::now())
    {
      this->lifetime = 5.0;
    }

    CarObject(double x, double y, double yaw, double confidence, double speed)
    : RoadObject(OBJECT::CAR, x, y, yaw, confidence, 0.0),
      first_x(x), first_y(y), first_detection_time(ros::Time::now())
    {
        this->z = 0.0;
        this->speed = speed;
        this->lifetime = 5.0;
    }

    /**
     * @brief Checks whether a new detection at (new_x, new_y) corresponds to this car.
     * 
     * Uses a constant-velocity prediction to account for motion.
     */
    bool is_same_object(double x, double y) override {
        double predicted_x = this->x;
        double predicted_y = this->y;
        // std::cout << "predicted: (" << predicted_x << ", " << predicted_y << "), new: (" << x << ", " << y << ")" << std::endl;
        // Compute Euclidean distance from predicted position to new detection.
        double distance = std::hypot(x - predicted_x, y - predicted_y);
        double threshold = VehicleConstants::LANE_OFFSET * 0.75;
        return (distance < threshold);
    }

    /**
     * @brief Merges a new detection into the car object state.
     *
     * Instead of taking a new speed measurement, the updated speed is computed based on
     * the displacement from both the previous and the first detections.
     *
     * @param new_x The new x-coordinate.
     * @param new_y The new y-coordinate.
     * @param new_yaw The new yaw (heading) angle.
     * @param new_confidence The confidence of the new detection.
     * @param new_z The new z-coordinate (default is 0).
     */
    void merge(double new_x, double new_y, double new_yaw, double new_confidence, double new_speed, double new_z = 0.0) override {
        ros::Time current_time = ros::Time::now();
        
        double dt_since_last = (current_time - this->last_detection_time).toSec();
        double dt_since_first = (current_time - this->first_detection_time).toSec();

        // Compute instantaneous speed from the previous detection.
        double inst_speed = (dt_since_last > 0) ? std::hypot(new_x - this->x, new_y - this->y) / dt_since_last : this->speed;

        // Compute average speed from the first detection.
        double avg_speed = (dt_since_first > 0) ? std::hypot(new_x - this->first_x, new_y - this->first_y) / dt_since_first : inst_speed;

        // Blend the two estimates.
        // Here, we weight more heavily on the average speed when there have been few detections,
        // and gradually shift to the instantaneous measure.
        int effective_count = std::min(this->detection_count, 5);
        double new_speed_estimate = (effective_count * inst_speed + (5 - effective_count) * avg_speed) / 5.0;

        // Weight for exponential moving average based on confidence.
        double alpha = new_confidence / (this->confidence + new_confidence);

        // Update position with exponential moving average.
        this->x = (1 - alpha) * this->x + alpha * new_x;
        this->y = (1 - alpha) * this->y + alpha * new_y;

        // Update yaw using weighted sine/cosine averaging to handle wrap-around.
        double sin_avg = (1 - alpha) * std::sin(this->yaw) + alpha * std::sin(new_yaw);
        double cos_avg = (1 - alpha) * std::cos(this->yaw) + alpha * std::cos(new_yaw);
        this->yaw = std::atan2(sin_avg, cos_avg);
        // update yaw with direction of average speed
        this->yaw = std::atan2(new_y - first_y, new_x - first_x);

        // Update the speed using our computed estimate.
        this->speed = (1 - alpha) * this->speed + alpha * new_speed_estimate;

        // Update altitude and overall confidence.
        this->z = (1 - alpha) * this->z + alpha * new_z;
        this->confidence = (1 - alpha) * this->confidence + alpha * new_confidence;

        this->detection_count++;
        // Always update the last detection time.
        this->last_detection_time = current_time;
    }
};
