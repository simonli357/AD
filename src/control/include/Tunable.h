#pragma once
#include <ros/ros.h>
#include <string>

#define CHECK_PARAM(param_name, var) \
    if (!nh.getParam(param_name, var)) { \
        std::cout << "Missing param: " << param_name << std::endl; \
        exit(1); \
    }

namespace Tunable {
    inline bool initialized = false;

    // launch args
    inline bool sign, ekf, lane, real, dashboard, keyboardControl, pubWaypoints, hasGps, useGps;
    inline bool emergency = false;
    inline double T, v_ref, T_park;
    inline int N;
    inline bool use_beta, async;
    inline std::string robot_name;
    inline std::string path;
    inline double x0, y0, yaw0, vref;
    inline bool use_tcp, use_traffic_server;
    inline int gps_id;
    inline int gps_points;
    inline std::string traffic_server_ip, ip_address;
    inline bool testing;
    inline int debugLevel;
    inline bool camera;
    inline double steer_offset, speed_offset, steer_offset_minimum, steer_offset_maximum;
    inline bool subModel;
    inline double rateVal;
    inline bool realsense_imu;
    inline bool use_encoder = false;

    // tunables
    inline double rel_speed_thresh = 0.15;
    inline double cw_speed_ratio = 1.0;
    inline double hw_speed_ratio = 1.0;
    inline double sign_localization_threshold = 0.5;
    inline double lane_localization_orientation_threshold = 10;
    inline double lane_localization_threshold = 0.1;
    inline double pixel_center_offset = -30.0;
    inline double constant_distance_to_intersection_at_detection = 0.371;
    inline double intersection_localization_threshold = 0.5;
    inline double stop_duration = 3.0;
    inline double parking_base_yaw_target = 0.166;
    inline double parking_base_speed = -0.2;
    inline double parking_base_thresh = 0.1;
    inline double sign_localization_orientation_threshold = 15;
    inline double intersection_localization_orientation_threshold = 15;
    inline double NORMAL_SPEED = 0.175;
    inline double change_lane_offset_scaler = 1.2;
    inline double min_dist_to_car = 0.357;
    inline double min_dist_to_consider_car = 1.0;
    inline double min_tailing_dist = 0.3; 
    
    inline bool lane_relocalize = false;
    inline bool lane_relocalize2 = false;
    inline bool sign_relocalize = true;
    inline bool intersection_relocalize = false;
    inline bool lane_yaw_reset = false;
    inline double lane_yaw_reset_cooldown = 1000.0;
    inline bool has_light = false;

    inline double pedestrian_distance = 0.5;
    inline double pedestrian_threshold = 0.4;

    inline double sign_lon_offset;
    inline double sign_lon_offset_slope;
    inline double sign_lat_offset;
    inline double sign_lat_offset_slope;

    inline double max_light_dist = 0.9;
    inline double max_sign_dist = 1.5;
    inline double min_sign_dist = 0.5;
    inline double highway_cooldown = 3.0;
    inline double lane_localization_cooldown = 3.0;
    inline double sign_cooldown = 3.0;
    inline std::vector<float> cumulative_confidence_thresholds = {2.5, 2.5, 2.5, 2.5, 2.5, 2.5, 2.5, 2.5, 2.5, 2.5, 2.5, 2.5, 2.5, 2.5, 2.5, 2.5, 10, 10, 10, 10};
    inline std::vector<float> recency_thresholds = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.1, 0.1, 0.1, 0.1};
    inline bool use_kf; // use Kalman filter for dynamic object tracking, else use EMA
    inline double odom_rate;
    inline double rs_roll = 0.0;
    inline double rs_pitch = 0.0;
    inline double rs_yaw = 0.0;
    inline bool orientation_follow = false;
    inline double lane_yaw_reset_thresh1 = 2.0; // threshold 1 for lane yaw reset
    inline double lane_yaw_reset_thresh2 = 1.5; // threshold 2 for lane yaw reset

    inline bool loadFromParams(ros::NodeHandle& nh) {
  
      // Launch args
      CHECK_PARAM("/emergency", emergency);
      CHECK_PARAM("/pub_wpts", pubWaypoints);
      CHECK_PARAM("/kb", keyboardControl);
      CHECK_PARAM("/dashboard", dashboard);
      CHECK_PARAM("/gps", hasGps);
      CHECK_PARAM("/use_gps", useGps);
      CHECK_PARAM("/use_encoder", use_encoder);
      CHECK_PARAM("/gps_id", gps_id);
      CHECK_PARAM("/gps_points", gps_points);
  
      std::string nodeName = ros::this_node::getName();
      std::cout << "LoadFromParams: nodeName: " << nodeName << std::endl;
  
      CHECK_PARAM(nodeName + "/lane", lane);
      CHECK_PARAM(nodeName + "/ekf", ekf);
      CHECK_PARAM(nodeName + "/sign", sign);
      CHECK_PARAM("T", T);
      CHECK_PARAM("N", N);
      CHECK_PARAM("constraints/v_ref", v_ref);
      CHECK_PARAM(nodeName + "/name", robot_name);
      CHECK_PARAM("/path", path);
      CHECK_PARAM(nodeName + "/vref", vref);
      CHECK_PARAM("/x0", x0);
      CHECK_PARAM("/y0", y0);
      CHECK_PARAM("/yaw0", yaw0);
      CHECK_PARAM("/T_park", T_park);
      CHECK_PARAM(nodeName + "/real", real);
      CHECK_PARAM("/use_beta", use_beta);
      CHECK_PARAM("/async", async);
      CHECK_PARAM("/use_tcp", use_tcp);
      CHECK_PARAM("/use_traffic_server", use_traffic_server);
      CHECK_PARAM("/traffic_server_ip", traffic_server_ip);
      CHECK_PARAM("/ip", ip_address);
      // CHECK_PARAM("/test", testing);
      testing = false;
      CHECK_PARAM("/debug_level", debugLevel);
      CHECK_PARAM("/camera", camera);
      CHECK_PARAM("/steer_offset", steer_offset);
      CHECK_PARAM("/speed_offset", speed_offset);
      CHECK_PARAM("/steer_offset_minimum", steer_offset_minimum);
      CHECK_PARAM("/steer_offset_maximum", steer_offset_maximum);
      CHECK_PARAM(nodeName+"/subModel", subModel);
      CHECK_PARAM(nodeName+"/rate", rateVal);
      CHECK_PARAM("/realsense_imu", realsense_imu);
  
      printf("LoadFromParams: emergency: %d, pubWaypoints: %d, kb: %d, dashboard: %d, gps: %d, lane: %d, ekf: %d, sign: %d, T: %.3f, N: %d, v_ref: %.3f, robot_name: %s, vref: %.3f, x0: %.3f, y0: %.3f, yaw0: %.3f, T_park: %.3f, real: %d, use_beta: %d, async: %d, use_tcp: %d, test: %d\n",
          emergency, pubWaypoints, keyboardControl, dashboard, hasGps, lane, ekf, sign, T, N, v_ref,
          robot_name.c_str(), vref, x0, y0, yaw0, T_park, real, use_beta, async, use_tcp, testing);
      printf("LoadFromParams2: debug_level: %d, camera: %d, steer_offset: %.3f, speed_offset: %.3f, steer_offset_minimum: %.3f, steer_offset_maximum: %.3f, subModel: %d, rateVal: %.3f, realsense_imu: %d\n",
          debugLevel, camera, steer_offset, speed_offset, steer_offset_minimum, steer_offset_maximum, subModel, rateVal, realsense_imu);
      // Tunables
    //   std::string mode = real ? "/real" : "/sim";
      std::string mode = "real";
  
      CHECK_PARAM(mode + "/rel_speed_thresh", rel_speed_thresh);
      CHECK_PARAM(mode + "/cw_speed_ratio", cw_speed_ratio);
      CHECK_PARAM(mode + "/hw_speed_ratio", hw_speed_ratio);
      CHECK_PARAM(mode + "/sign_localization_threshold", sign_localization_threshold);
      CHECK_PARAM(mode + "/lane_localization_orientation_threshold", lane_localization_orientation_threshold);
      CHECK_PARAM(mode + "/lane_localization_threshold", lane_localization_threshold);
      CHECK_PARAM(mode + "/pixel_center_offset", pixel_center_offset);
      CHECK_PARAM(mode + "/constant_distance_to_intersection_at_detection", constant_distance_to_intersection_at_detection);
      CHECK_PARAM(mode + "/intersection_localization_threshold", intersection_localization_threshold);
      CHECK_PARAM(mode + "/stop_duration", stop_duration);
      CHECK_PARAM(mode + "/parking_base_yaw_target", parking_base_yaw_target);
      CHECK_PARAM(mode + "/parking_base_speed", parking_base_speed);
      CHECK_PARAM(mode + "/parking_base_thresh", parking_base_thresh);
      CHECK_PARAM(mode + "/sign_localization_orientation_threshold", sign_localization_orientation_threshold);
      CHECK_PARAM(mode + "/intersection_localization_orientation_threshold", intersection_localization_orientation_threshold);
      CHECK_PARAM(mode + "/NORMAL_SPEED", NORMAL_SPEED);
      CHECK_PARAM(mode + "/lane_relocalize", lane_relocalize);
      CHECK_PARAM(mode + "/lane_relocalize2", lane_relocalize2);
      CHECK_PARAM(mode + "/sign_relocalize", sign_relocalize);
      CHECK_PARAM(mode + "/intersection_relocalize", intersection_relocalize);
      CHECK_PARAM(mode + "/lane_yaw_reset", lane_yaw_reset);
      CHECK_PARAM(mode + "/lane_yaw_reset_cooldown", lane_yaw_reset_cooldown);
      CHECK_PARAM(mode + "/has_light", has_light);
      CHECK_PARAM(mode + "/change_lane_offset_scaler", change_lane_offset_scaler);
      CHECK_PARAM(mode + "/min_dist_to_car", min_dist_to_car);
      CHECK_PARAM(mode + "/min_dist_to_consider_car", min_dist_to_consider_car);
      CHECK_PARAM(mode + "/min_tailing_dist", min_tailing_dist);
      CHECK_PARAM(mode + "/pedestrian_distance", pedestrian_distance);
      CHECK_PARAM(mode + "/pedestrian_threshold", pedestrian_threshold);
      CHECK_PARAM(mode + "/sign_lon_offset", sign_lon_offset);
      CHECK_PARAM(mode + "/sign_lon_offset_slope", sign_lon_offset_slope);
      CHECK_PARAM(mode + "/sign_lat_offset", sign_lat_offset);
        CHECK_PARAM(mode + "/sign_lat_offset_slope", sign_lat_offset_slope);
      CHECK_PARAM(mode + "/max_light_dist", max_light_dist);
      CHECK_PARAM(mode + "/max_sign_dist", max_sign_dist);
      CHECK_PARAM(mode + "/min_sign_dist", min_sign_dist);
      CHECK_PARAM(mode + "/highway_cooldown", highway_cooldown);
      CHECK_PARAM(mode + "/lane_localization_cooldown", lane_localization_cooldown);
      CHECK_PARAM(mode + "/sign_cooldown", sign_cooldown);
      CHECK_PARAM(mode + "/cumulative_confidence_thresholds", cumulative_confidence_thresholds);
      CHECK_PARAM(mode + "/recency_thresholds", recency_thresholds);
      CHECK_PARAM(mode + "/use_kf", use_kf);
      CHECK_PARAM(mode + "/odom_rate", odom_rate);
      CHECK_PARAM(mode + "/rs_roll", rs_roll);
      CHECK_PARAM(mode + "/rs_pitch", rs_pitch);
      CHECK_PARAM(mode + "/rs_yaw", rs_yaw);
      CHECK_PARAM(mode + "/orientation_follow", orientation_follow);
      CHECK_PARAM(mode + "/lane_yaw_reset_thresh1", lane_yaw_reset_thresh1);
      CHECK_PARAM(mode + "/lane_yaw_reset_thresh2", lane_yaw_reset_thresh2);
        
      initialized = true;
      return true;
  }
}
