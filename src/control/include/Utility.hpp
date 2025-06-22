#pragma once

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/String.h>
#include <std_msgs/Header.h>
#include <gazebo_msgs/ModelStates.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_msgs/TFMessage.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <vector>
#include <array>
#include <eigen3/Eigen/Dense>
#include "TcpClient.hpp"
#include "TrafficClient.hpp"
#include "utils/Lane3.h"
#include <std_srvs/Trigger.h>
#include <mutex>
#include <cmath>
#include <boost/asio.hpp>
#include "utils/constants.h"
#include "Tracking.h"
#include "Tunable.h"
#include <algorithm>
#include <deque>
#include "ros/ros.h"
#include "sensor_msgs/image_encodings.h"
#include "std_msgs/Header.h"
#include <chrono>
#include <librealsense2/rs.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>
#include <utils/Sign.h>
#include <utils/encoder.h>
#include "utils/helper.h"
#include "Sensing.h"
#include "Perception.hpp"
#include <numeric>

using namespace VehicleConstants;
using namespace Tunable;

class Utility {
public:
    
    Utility(ros::NodeHandle& nh_, bool pubOdom = true);
    ~Utility();
    void callTriggerService();
// private:
    ros::NodeHandle& nh;
    ros::ServiceClient triggerServiceClient;
    
    bool emergency = false;
    int num_obj = 0;
    std::mutex general_mutex;
    bool pubOdom;
    std_msgs::String debug_msg;
    ros::Rate* rate;

    double l_r, l_f, odomRatio, maxspeed;

    // lane stuff
    double center, lane_center_offset, image_center, p, d, last;
    double stopline_dist, stopline_angle;
    Eigen::MatrixXd lane_waypoints;

    double height=0, velocity, steer_command, velocity_command, x_speed, y_speed;
    std::deque<double> velocity_command_queue; 
    void add_velocity_command(double new_command) {
        velocity_command_queue.push_back(new_command);
        if (velocity_command_queue.size() > 5) {
            velocity_command_queue.pop_front();  // Remove oldest
        }
    }
    double filter_encoder(double encoder_speed) {
        if (velocity_command_queue.size() < 5) {
            return encoder_speed;  // Not enough data to filter
        }
        double filtered_speed = encoder_speed;
        // calculate the difference between encoder_speed and every element in the queue.
        // if minimum different smaller than 0.1, don't do anything, else set encoder_speed to velocity_command
        double min_diff = std::abs(encoder_speed - velocity_command_queue[0]);
        for (size_t i = 1; i < velocity_command_queue.size(); ++i) {
            double diff = std::abs(encoder_speed - velocity_command_queue[i]);
            if (diff < min_diff) {
                min_diff = diff;
            }
        }
        if (min_diff > 0.15) {
            // printf("filter_encoder(): Encoder speed: %.2f, Velocity command: %.2f, min diff: %.2f. FILTERED!!!\n", encoder_speed, velocity_command, min_diff);
            // debug("filter_encoder(): Encoder speed: " + helper::d2str(encoder_speed) + 
            //       ", Velocity command: " + helper::d2str(velocity_command) + 
            //       ", min diff: " + helper::d2str(min_diff) + ". FILTERED!!!", 1);
            filtered_speed = velocity_command;
        } else {
            // printf("filter_encoder(): Encoder speed: %.2f, Velocity command: %.2f, min diff: %.2f\n", encoder_speed, velocity_command, min_diff);
        }

        // if encoder speed is less than 0.03, set it to 0
        if (std::abs(encoder_speed) < 0.03) {
            filtered_speed = 0.0;
        }
        if (std::abs(velocity_command) < 0.01 && min_diff > 0.07) {
            filtered_speed = velocity_command;
        }
        return filtered_speed;
    }
    double odomX, odomY, odomYaw, dx, dy, dheight, dyaw, ekf_x, ekf_y, ekf_yaw, gps_x, gps_y;
    double filtered_encoder_speed = 0.0;
    double x0 = 1.55, y0 = 6.906, yaw0 = 0;
    std::string pathName = Tunable::path;
    double gps_state[3];
    double ekf_state[3];
    std::optional<size_t> car_idx;

    ros::Time timerodom;
    std::optional<ros::Time> timerpid;
    std::optional<ros::Time> initializationTimer;
    ros::Time general_timer;

    double covariance_value;

    bool initializationFlag = false;

    tf2_ros::StaticTransformBroadcaster static_broadcaster;
    tf2_ros::TransformBroadcaster broadcaster;
    tf2_ros::Buffer tfBuffer;

    // Client
    std::shared_ptr<TcpClient> tcp_client;
    std::unique_ptr<TrafficClient> traffic_client;

    // publishers
    ros::Publisher odom_pub;
    ros::Publisher cmd_vel_pub;
    ros::Publisher car_pose_pub;
    ros::Publisher road_object_pub;
    ros::Publisher message_pub;
    ros::Publisher pose_pub;
    ros::Publisher waypoints_pub;
    ros::Publisher state_offset_pub;

    // messages
    nav_msgs::Odometry odom_msg;
    // nav_msgs::Odometry odom1_msg;
    nav_msgs::Odometry ekf_msg;
    std_msgs::String msg;
    std_msgs::String msg2;
    std_msgs::Float32MultiArray state_offset_msg;

    gazebo_msgs::ModelStates model;
    std_msgs::Float32MultiArray sign;
    utils::Lane3 lane;
    tf2::Matrix3x3 m_chassis;
    tf2::Quaternion tf2_quat;
    tf2::Quaternion q_transform;
    tf2::Quaternion q_chassis;

    // subscribers
    ros::Subscriber lane_sub;
    ros::Subscriber lane_center_offset_sub;
    ros::Subscriber sign_sub;
    ros::Subscriber encoder_sub;
    ros::Subscriber waypoints_sub;
    std::vector<float> detected_objects;
    ros::Subscriber model_sub;
    ros::Subscriber ekf_sub;
    ros::Subscriber tf_sub;

    // Initialization
    void initialize();
    void initialize_tcp_client();
    void fetch_run_params();

    ros::Timer odom_pub_timer;
    void odom_pub_timer_callback(const ros::TimerEvent&);
    ros::Timer ekf_update_timer;
    void ekf_update_timer_callback(const ros::TimerEvent&) {
        update_odom_with_ekf();
    }

    // Callbacks
    void lane_callback(const utils::Lane3::ConstPtr& msg);
    void process_lane_data(const utils::Lane3& msg);
    ros::Timer lane_timer;
    void lane_timer_callback(const ros::TimerEvent& event) {
        auto optional_lane_msg = Perception::get_lane_msg();
        if (optional_lane_msg) {
            process_lane_data(*optional_lane_msg);
        }
    }
    void sign_callback(const utils::Sign::ConstPtr& msg);
    void encoder_callback(const utils::encoder::ConstPtr& msg);
    void process_sign_data(const utils::Sign& msg);
    ros::Timer sign_timer;
    void sign_timer_callback(const ros::TimerEvent& event) {
        auto optional_sign_msg = Perception::get_sign_msg();
        if (optional_sign_msg) {
            process_sign_data(*optional_sign_msg);
        }
    }
    void model_callback(const gazebo_msgs::ModelStates::ConstPtr& msg);
    void ekf_callback(const nav_msgs::Odometry::ConstPtr& msg);
    void tf_callback(const tf2_msgs::TFMessage::ConstPtr& msg);
    void spin();
    // Methods
    void stop_car();
    void publish_static_transforms();
    void publish_odom();
    int object_index(int obj_id);
    std::vector<int> object_indices(int obj_id);
    double object_distance(int index);
    std::array<double, 4> object_box(int index);
    void object_box(int index, std::array<double, 4>& oBox);
    ros::Time object_detection_time;
    void set_initial_pose(double x, double y, double yaw);
    void reset_odom();
    int update_states_rk4(double velocity, double steer, double dt=-1);
    geometry_msgs::TransformStamped add_static_link(double x, double y, double z, double roll, double pitch, double yaw, std::string parent, std::string child);
    void publish_cmd_vel(double steering_angle, double velocity = -3.57, bool clip = true);
    void lane_follow();
    void idle();
    double get_steering_angle(double offset=-20);
    double get_current_orientation();
    std::array<double, 3> get_real_states() const;

    double get_yaw() {
        return Sensing::yaw;
    }
    int set_states(double x, double y) {
        x0 = x;
        y0 = y;
        odomX = 0;
        odomY = 0;
        return 0;
    }
    int get_states(double &x_, double &y_, double &yaw_) {
        // std::lock_guard<std::mutex> lock(general_mutex);
        if (subModel) {
            x_ = gps_x;
            y_ = gps_y;
        } else {
            x_ = odomX + x0;
            y_ = odomY + y0;
        }
        yaw_ = Sensing::yaw;
        return 0;
    }
    void update_states(Eigen::Vector3d& o_state) {
        // std::lock_guard<std::mutex> lock(general_mutex);
        if (subModel) {
            o_state << gps_x, gps_y, Sensing::yaw;
        } else {
            o_state << odomX + x0, odomY + y0, Sensing::yaw;
        }
    }
    int recalibrate_states(double x_offset, double y_offset) {
        if(Tunable::ekf) {
            if (hasGps) {
                x0 += x_offset;
                y0 += y_offset;
            } else {
                x0 += x_offset;
                y0 += y_offset;
            }
        } else {
            x0 += x_offset;
            y0 += y_offset;
        }
        return 1;
    }
    int get_mean_ekf(double &x_, double &y_, int n = 10) {
        auto ekf_states = Eigen::MatrixXd (2, n);
        for (int i = 0; i < n; i++) {
            ekf_states(0, i) = ekf_x;
            ekf_states(1, i) = ekf_y;
            ros::Duration(0.2).sleep();
        }
        // take the average of the last n states
        x_ = ekf_states.row(0).mean();
        y_ = ekf_states.row(1).mean();
        return 1;
    }
    int reinitialize_states() {
        if(Tunable::ekf) {
            std::cout << "waiting for ekf message" << std::endl;
            ros::topic::waitForMessage<nav_msgs::Odometry>("/odometry/filtered");
            std::cout << "received message from ekf" << std::endl;
            double x, y;
            get_mean_ekf(x, y);
            x0 = x;
            y0 = y;
        } else if(subModel) {
            std::cout << "waiting for model message" << std::endl;
            ros::topic::waitForMessage<gazebo_msgs::ModelStates>("/gazebo/model_states");
            std::cout << "received message from model" << std::endl;
            x0 = gps_x;
            y0 = gps_y;
        } else {
            x0 = odomX;
            y0 = odomY;
        }
        return 1;
    }
    void get_gps_states(double &x_, double &y_) {
        x_ = gps_x;
        y_ = gps_y;
    }
    void get_ekf_states(double &x_, double &y_) {
        x_ = ekf_x;
        y_ = ekf_y;
    }
    
    int update_odom_with_ekf() {
        ROS_INFO("DEBUG: update_odom_with_ekf(), ekf_x: %.3f, ekf_y: %.3f, odomX: %.3f, odomY: %.3f", ekf_x, ekf_y, odomX + x0, odomY + y0);
        x0 = ekf_x - odomX;
        y0 = ekf_y - odomY;
        return 1;
    }

    Eigen::Vector2d estimate_object_pose2d(double x, double y, double yaw,
                                       double x1, double y1, double x2, double y2,
                                       double object_distance, 
                                       bool is_car = false)
    {
        // std::cout << "estimate_object_pose2d(): x: " << x << ", y: " << y << ", yaw: " << yaw << ", object_distance: " << object_distance << ", is_car: " << is_car << std::endl;
        static double parallel_w2h_ratio = 1.30;
        static double perpendicular_w2h_ratio = 2.70;

        // std::cout << "object_distance1: " << object_distance << std::endl;
        if (is_car) {
            double car_pixel_w2h_ratio = std::abs((x2 - x1) / (y2 - y1));
            // std::cout << "car_pixel_w2h_ratio: " << car_pixel_w2h_ratio << std::endl;

            // Normalize the ratio to a scale of 0 (parallel) to 1 (perpendicular)
            double normalized_ratio_parallel = std::max((car_pixel_w2h_ratio / parallel_w2h_ratio), 1.0);
            double normalized_ratio_perpendicular = std::min(car_pixel_w2h_ratio / perpendicular_w2h_ratio, 1.0);
            // std::cout << "normalized_ratio_parallel: " << normalized_ratio_parallel << std::endl;
            // std::cout << "normalized_ratio_perpendicular: " << normalized_ratio_perpendicular << std::endl;

            double parallel_diff = std::abs(normalized_ratio_parallel - 1);
            double perpendicular_diff = std::abs(normalized_ratio_perpendicular - 1);
            double dist;
            if (car_pixel_w2h_ratio < 2.0 || parallel_diff < perpendicular_diff) { // Parallel to the camera
                // std::cout << "Parallel to the camera" << std::endl;
                dist = CAR_LENGTH / 2 / normalized_ratio_parallel;
            } else { // Perpendicular to the camera
                dist = CAR_WIDTH / 2 / normalized_ratio_perpendicular;
            }
            
            // std::cout << "dist: " << dist << std::endl;
            object_distance += dist;
        }
        // std::cout << "object_distance2: " << object_distance << std::endl;

        // Extract camera parameters
        double fx = CAMERA_PARAMS[0];
        double fy = CAMERA_PARAMS[1];
        double cx = CAMERA_PARAMS[2];
        double cy = CAMERA_PARAMS[3];
        if (Tunable::real) {
            fx = CAMERA_PARAMS_REAL[0];
            fy = CAMERA_PARAMS_REAL[1];
            cx = CAMERA_PARAMS_REAL[2];
            cy = CAMERA_PARAMS_REAL[3];
        }

        // Compute bounding box center in image coordinates
        double bbox_center_x = (x1 + x2) / 2;
        double bbox_center_y = (y1 + y2) / 2;

        // Convert image coordinates to normalized coordinates
        double x_norm = (bbox_center_x - cx) / fx;
        double y_norm = (bbox_center_y - cy) / fy;

        // Estimate 3D coordinates in the camera frame
        // [forward = Z_c, right = –X_c]
        double X_c = x_norm * object_distance;
        // double Y_c = y_norm * object_distance;
        double Z_c = object_distance;

        auto const& tf      = Tunable::real ? REALSENSE_TF_REAL : REALSENSE_TF;
        double tx    = tf[0], ty = tf[1], cam_yaw = tf[5];
        double cam_roll = tf[3];
        double cam_pitch = tf[4];

        // flat ground‐plane ray in camera coords (forward, right)
        Eigen::Vector2d P_cam_flat(Z_c, -X_c);

        // rotate by the camera’s yaw mount offset, then translate
        Eigen::Rotation2Dd R_cam_yaw(cam_yaw);
        Eigen::Vector2d P_v2d = R_cam_yaw * P_cam_flat + Eigen::Vector2d(tx, ty);

        // latency & motion compensation
        double latency = (ros::Time::now() - object_detection_time).toSec();
        double speed   = Tunable::use_encoder ? filtered_encoder_speed : velocity_command;
        P_v2d.x()    -= latency * speed;
        P_v2d.x()    += sign_lon_offset_slope * P_v2d.x() + sign_lon_offset;
        P_v2d.y()    += sign_lat_offset;

        // rotate into world frame and translate by vehicle pose
        Eigen::Matrix2d R_vw;
        R_vw << cos(yaw), -sin(yaw),
                sin(yaw),  cos(yaw);
        // std::cout << "object_distance4: " << P_v2d[0] << std::endl;

        static std::vector<std::vector<double>> history;
        double avg_x = std::accumulate(history.begin(), history.end(), 0.0, [](double sum, const std::vector<double>& vec) {
            return sum + vec[0];
        }) / history.size();
        double avg_y = std::accumulate(history.begin(), history.end(), 0.0, [](double sum, const std::vector<double>& vec) {
            return sum + vec[1];
        }) / history.size();
        history.push_back({P_v2d[0], P_v2d[1]});
        if (history.size() > 3) {
            history.erase(history.begin());
            printf("avg relative position: %.3f, %.3f\n", avg_x, avg_y);
        }

        return Eigen::Vector2d(x, y) + R_vw * P_v2d;
    }
    Eigen::Vector2d estimate_object_pose2d(double x, double y, double yaw, 
            const std::array<double, 4>& bounding_box, double object_distance, 
            bool is_car = false) {
        double x1 = bounding_box[0];
        double y1 = bounding_box[1];
        double x2 = bounding_box[2];
        double y2 = bounding_box[3];
        return estimate_object_pose2d(x, y, yaw, x1, y1, x2, y2, object_distance, is_car);
    }

    void send_speed_and_steer(float f_velocity, float f_angle) {
        // ROS_INFO("speed:%.3f, angle:%.3f, yaw:%.3f, odomX:%.2f, odomY:%.2f, ekfx:%.2f, ekfy:%.2f", f_velocity, f_angle, yaw * 180 / M_PI, odomX, odomY, ekf_x-x0, ekf_y-y0);
        static bool first = true;
        static bool use_pid = false;
        if (Sensing::serial == nullptr) {
            // debug("send_speed_and_steer(): Serial is null", 4);
            return;
        }

        if (first && use_pid) {
            first = false;
        
            float f_active = 1.0;
            float f_proportional = 1.00;
            float f_integral = 0.0;
            float f_derivative = 0.0;
            
            std::stringstream pid_str;
            char pid_buff[100];
            snprintf(pid_buff, sizeof(pid_buff), "%.4f:%.4f:%.4f:%.4f;;\r\n", f_active, f_proportional, f_integral, f_derivative);
            pid_str << "#" << "12" << ":" << pid_buff;
            std::cout << pid_str.str() << std::endl;
            
            boost::asio::write(*Sensing::serial, boost::asio::buffer(pid_str.str()));
        }

        // if(f_angle > 3.0) f_angle+=4.0;
        std::stringstream strs;
        char buff[100];
        snprintf(buff, sizeof(buff), "%.2f:%.2f;;\r\n", f_velocity * 100, f_angle);
        std::string number = use_pid ? "13" : "11";
        strs << "#" << number << ":" << buff;
        boost::asio::write(*Sensing::serial, boost::asio::buffer(strs.str()));
        // std::cout << strs.str() << std::endl;
    }

    Eigen::MatrixXd waypoints_to_world(const Eigen::MatrixXd& bodyWaypoints,
        const Eigen::Vector3d& vehiclePose)
    {
        double car_x   = vehiclePose(0);
        double car_y   = vehiclePose(1);
        double car_yaw = vehiclePose(2);

        // Construct the 2x2 rotation matrix based on the vehicle's yaw.
        Eigen::Matrix2d R;
        R << std::cos(car_yaw), -std::sin(car_yaw),
        std::sin(car_yaw),  std::cos(car_yaw);

        // Extract the position columns (n x 2) from the input.
        Eigen::MatrixXd positions = bodyWaypoints.leftCols(2);
        // Apply rotation: each row is rotated by R.
        // Note: multiplying by R.transpose() applies R to each row.
        Eigen::MatrixXd world_positions = (positions * R.transpose())
                    .rowwise() + Eigen::RowVector2d(car_x, car_y);

        // Compute the world-frame yaw for each waypoint.
        // Add the vehicle's yaw to each waypoint's yaw.
        Eigen::VectorXd world_yaw = bodyWaypoints.col(2).array() + car_yaw;

        // Combine the rotated positions and yaw into one (n x 3) matrix.
        Eigen::MatrixXd worldWaypoints(bodyWaypoints.rows(), 3);
        worldWaypoints.block(0, 0, bodyWaypoints.rows(), 2) = world_positions;
        worldWaypoints.col(2) = world_yaw;

        return worldWaypoints;
    }

    std::array<double, 3> object_to_world(double object_x, double object_y, double object_yaw, 
                                double vehicle_x, double vehicle_y, double vehicle_yaw)
    {
        double world_x = vehicle_x + (std::cos(vehicle_yaw) * object_x - std::sin(vehicle_yaw) * object_y);
        double world_y = vehicle_y + (std::sin(vehicle_yaw) * object_x + std::cos(vehicle_yaw) * object_y);
        double world_yaw = object_yaw + vehicle_yaw;
        return {world_x, world_y, world_yaw};
    }

    void debug(const std::string& message, int level) {
        if (debugLevel >= level) {
            debug_msg.data = message;
            message_pub.publish(debug_msg);
            if (tcp_client != nullptr) tcp_client->send_message(debug_msg);
            ROS_INFO("%s", message.c_str());
        }
    }
    
    const std::vector<std::vector<double>>& get_relevant_signs(int type, std::string& o_string) {
        OBJECT obj = static_cast<OBJECT>(type);
        if (obj == OBJECT::ROUNDABOUT) {
            o_string = "ALL ROUNDABOUTS";
            return ALL_ROUNDABOUTS;
        } else if (obj == OBJECT::STOPSIGN || obj == OBJECT::PRIORITY) {
            o_string = (obj == OBJECT::STOPSIGN) ? "STOPSIGN" :
                        (obj == OBJECT::PRIORITY) ? "PRIORITY" :
                        "UNKNOWN";
            return ALL_SIGNS;
        } else if (obj == OBJECT::CROSSWALK) {
            o_string = "ALL CROSSWALKS";
            return ALL_CROSSWALKS;
        } else if (obj == OBJECT::LIGHTS || obj == OBJECT::GREENLIGHT || obj == OBJECT::REDLIGHT || obj == OBJECT::YELLOWLIGHT) { 
            o_string = "ALL LIGHTS";
            return ALL_LIGHTS;
        } else if (obj == OBJECT::HIGHWAYENTRANCE) {
            o_string = "ALL_HIGHWAYENTRANCES";
            return ALL_HIGHWAYENTRANCES;
        } else if (obj == OBJECT::HIGHWAYEXIT) {
            o_string = "ALL_HIGHWAYEXITS";
            return ALL_HIGHWAYEXITS;
        } else if (obj == OBJECT::PARK) {
            o_string = "PARKING SIGNS";
            return PARKING_SIGN_POSES1;
        }
        o_string = "UNKNOWN";
        return EMPTY;
    }
};
