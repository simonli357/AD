#pragma once
#include <ros/ros.h>
#include <string>

namespace Tunable {
    inline double change_lane_yaw = 0.15;
    inline double cw_speed_ratio = 1.0;
    inline double hw_speed_ratio = 1.0;
    inline double sign_localization_threshold = 0.5;
    inline double lane_localization_orientation_threshold = 10;
    inline double pixel_center_offset = -30.0;
    inline double constant_distance_to_intersection_at_detection = 0.371;
    inline double intersection_localization_threshold = 0.5;
    inline double stop_duration = 3.0;
    inline double parking_base_yaw_target = 0.166;
    inline double parking_base_speed = -0.2;
    inline double parking_base_thresh = 0.1;
    inline double change_lane_speed = 0.2;
    inline double change_lane_thresh = 0.05;
    inline double sign_localization_orientation_threshold = 15;
    inline double intersection_localization_orientation_threshold = 15;
    inline double NORMAL_SPEED = 0.175;
    inline double FAST_SPEED = 0.4;
    inline double change_lane_offset_scaler = 1.2;
    
    inline bool use_stopline = true;
    inline bool lane_relocalize = false;
    inline bool sign_relocalize = true;
    inline bool intersection_relocalize = false;
    inline bool has_light = false;

    inline int pedestrian_count_thresh = 8;

    inline double sign_lon_offset;
    inline double sign_lon_offset_slope;
    inline double sign_lat_offset;
    inline double sign_latency;

    inline double max_light_dist = 0.9;
    inline double max_sign_dist = 1.5;
    inline double min_sign_dist = 0.5;
    inline double highway_cooldown = 3.0;
    inline double sign_cooldown = 3.0;
    inline std::vector<float> cumulative_confidence_thresholds = {2.5, 2.5, 2.5, 2.5, 2.5, 2.5, 2.5, 2.5, 2.5, 2.5, 2.5, 2.5, 2.5, 2.5, 2.5, 2.5};
    inline std::vector<float> recency_thresholds = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5};

    inline bool loadFromParams(ros::NodeHandle& nh, bool real) {
      bool success = true;
      std::string mode = real ? "/real" : "/sim";
      success &= nh.getParam(mode + "/change_lane_yaw", change_lane_yaw);
      success &= nh.getParam(mode + "/cw_speed_ratio", cw_speed_ratio);
      success &= nh.getParam(mode + "/hw_speed_ratio", hw_speed_ratio);
      success &= nh.getParam(mode + "/sign_localization_threshold", sign_localization_threshold);
      success &= nh.getParam(mode + "/lane_localization_orientation_threshold", lane_localization_orientation_threshold);
      success &= nh.getParam(mode + "/pixel_center_offset", pixel_center_offset);
      success &= nh.getParam(mode + "/constant_distance_to_intersection_at_detection", constant_distance_to_intersection_at_detection);
      success &= nh.getParam(mode + "/intersection_localization_threshold", intersection_localization_threshold);
      success &= nh.getParam(mode + "/stop_duration", stop_duration);
      success &= nh.getParam(mode + "/parking_base_yaw_target", parking_base_yaw_target);
      success &= nh.getParam(mode + "/parking_base_speed", parking_base_speed);
      success &= nh.getParam(mode + "/parking_base_thresh", parking_base_thresh);
      success &= nh.getParam(mode + "/change_lane_speed", change_lane_speed);
      success &= nh.getParam(mode + "/change_lane_thresh", change_lane_thresh);
      success &= nh.getParam(mode + "/sign_localization_orientation_threshold", sign_localization_orientation_threshold);
      success &= nh.getParam(mode + "/intersection_localization_orientation_threshold", intersection_localization_orientation_threshold);
      success &= nh.getParam(mode + "/NORMAL_SPEED", NORMAL_SPEED);
      success &= nh.getParam(mode + "/FAST_SPEED", FAST_SPEED);
      success &= nh.getParam(mode + "/use_stopline", use_stopline);
      success &= nh.getParam(mode + "/lane_relocalize", lane_relocalize);
      success &= nh.getParam(mode + "/sign_relocalize", sign_relocalize);
      success &= nh.getParam(mode + "/intersection_relocalize", intersection_relocalize);
      success &= nh.getParam(mode + "/has_light", has_light);
      success &= nh.getParam(mode + "/change_lane_offset_scaler", change_lane_offset_scaler);
      success &= nh.getParam(mode + "/pedestrian_count_thresh", pedestrian_count_thresh);
      success &= nh.getParam(mode + "/sign_lon_offset", sign_lon_offset);
      success &= nh.getParam(mode + "/sign_lon_offset_slope", sign_lon_offset_slope);
      success &= nh.getParam(mode + "/sign_lat_offset", sign_lat_offset);
      success &= nh.getParam(mode + "/sign_latency", sign_latency);
      success &= nh.getParam(mode + "/max_light_dist", max_light_dist);
      success &= nh.getParam(mode + "/max_sign_dist", max_sign_dist);
      success &= nh.getParam(mode + "/min_sign_dist", min_sign_dist);
      success &= nh.getParam(mode + "/highway_cooldown", highway_cooldown);
      success &= nh.getParam(mode + "/sign_cooldown", sign_cooldown);
      success &= nh.getParam(mode + "/cumulative_confidence_thresholds", cumulative_confidence_thresholds);
      success &= nh.getParam(mode + "/recency_thresholds", recency_thresholds);
      return success;
  }
}
