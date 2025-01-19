#pragma once

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/String.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/Imu.h>
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
#include "utils/Lane2.h"
#include <std_srvs/Trigger.h>
#include <mutex>
#include <cmath>
#include <boost/asio.hpp>
#include "constants.h"
#include "RoadObject.hpp"

using namespace VehicleConstants;

class Utility {
public:
    
    Utility(ros::NodeHandle& nh_, bool real, double x0, double y0, double yaw0, bool subSign = true, bool useEkf = false, bool subLane = false,  std::string robot_name = "car1", bool subModel = false, bool subImu = true, bool pubOdom = true);
    ~Utility();
    void callTriggerService();
// private:

    std::vector<std::shared_ptr<RoadObject>> road_objects;
    //tunables
    double gps_offset_x, gps_offset_y;

    typedef double (Utility::*TrajectoryFunction)(double x);
    TrajectoryFunction trajectoryFunction;
    int intersectionDecision;
    ros::NodeHandle& nh;
    ros::ServiceClient triggerServiceClient;
    
    std::string robot_name;
    // std::vector<std::array<double, 2>> detected_cars;
    std::vector<Eigen::Vector2d> detected_cars;
    std::vector<int> detected_cars_counter;
    std::list<int> recent_car_indices;
    void print_detected_cars() {
        for (size_t i = 0; i < detected_cars.size(); i++) {
            std::cout << "Car " << i << ": " << detected_cars[i][0] << ", " << detected_cars[i][1] << std::endl;
        }
    }
    struct CameraPose {
        double x;
        double y;
        double z;
    } const CAMERA_POSE = {0.095, 0, 0.165};

    bool emergency = false;
    int num_obj = 0;
    std::mutex lock;
    bool pubOdom, useIMU, subLane, subSign, subModel, subImu, useEkf, hasGps;
    int debugLevel = 5;
    std_msgs::String debug_msg;
    bool real;
    double rateVal;
    ros::Rate* rate;

    double wheelbase, odomRatio, maxspeed, center, image_center, p, d, last;
    // bool stopline = false;
    int stopline = -1;
    double yaw, pitch = 0, height=0, velocity, steer_command, velocity_command, x_speed, y_speed;
    double odomX, odomY, odomYaw, dx, dy, dheight, dyaw, ekf_x, ekf_y, ekf_yaw, gps_x, gps_y;
    double initial_yaw = 0;
    double x_offset, y_offset;
    double x0 = -1, y0 = -1, yaw0 = 0;
    double gps_state[3];
    double ekf_state[3];
    std::optional<size_t> car_idx;

    ros::Time timerodom;
    std::optional<ros::Time> timerpid;
    std::optional<ros::Time> initializationTimer;
    ros::Time general_timer;

    double covariance_value;

    bool initializationFlag, imuInitialized = false;

    tf2_ros::StaticTransformBroadcaster static_broadcaster;
    tf2_ros::TransformBroadcaster broadcaster;
    tf2_ros::Buffer tfBuffer;

    // Client
    std::unique_ptr<TcpClient> tcp_client;

    // publishers
    ros::Publisher odom_pub;
    ros::Publisher cmd_vel_pub;
    ros::Publisher car_pose_pub;
    ros::Publisher road_object_pub;
    ros::Publisher message_pub;
    ros::Publisher pose_pub;
    ros::Publisher waypoints_pub;
    ros::Publisher detected_cars_pub;
    ros::Publisher state_offset_pub;

    // messages
    nav_msgs::Odometry odom_msg;
    // nav_msgs::Odometry odom1_msg;
    nav_msgs::Odometry ekf_msg;
    std_msgs::String msg;
    std_msgs::String msg2;
    std_msgs::Float32MultiArray car_pose_msg;
    std_msgs::Float32MultiArray state_offset_msg;

    gazebo_msgs::ModelStates model;
    std_msgs::Float32MultiArray sign;
    utils::Lane2 lane;
    sensor_msgs::Imu imu_msg;
    tf2::Quaternion q_imu;
    tf2::Matrix3x3 m_chassis;
    tf2::Quaternion tf2_quat;
    tf2::Quaternion q_transform;
    tf2::Quaternion q_chassis;

    // subscribers
    ros::Subscriber lane_sub;
    ros::Subscriber sign_sub;
    std::vector<float> detected_objects;
    ros::Subscriber model_sub;
    ros::Subscriber imu_sub;
    ros::Subscriber ekf_sub;
    ros::Subscriber tf_sub;

    ros::Timer odom_pub_timer;
    void odom_pub_timer_callback(const ros::TimerEvent&);
    ros::Timer imu_pub_timer;
    void imu_pub_timer_callback(const ros::TimerEvent&);
    ros::Timer ekf_update_timer;
    void ekf_update_timer_callback(const ros::TimerEvent&) {
        update_odom_with_ekf();
    }
    ros::Publisher imu_pub;

    // Callbacks
    void lane_callback(const utils::Lane2::ConstPtr& msg);
    void sign_callback(const std_msgs::Float32MultiArray::ConstPtr& msg);
    void model_callback(const gazebo_msgs::ModelStates::ConstPtr& msg);
    void imu_callback(const sensor_msgs::Imu::ConstPtr& msg);
    void ekf_callback(const nav_msgs::Odometry::ConstPtr& msg);
    void tf_callback(const tf2_msgs::TFMessage::ConstPtr& msg);
    void spin();
    // Methods
    void stop_car();
    void publish_static_transforms();
    void set_pose_using_service(double x, double y, double yaw);
    void publish_odom();
    int object_index(int obj_id);
    std::vector<int> object_indices(int obj_id);
    double object_distance(int index);
    std::array<double, 4> object_box(int index);
    void object_box(int index, std::array<double, 4>& oBox);
    void set_initial_pose(double x, double y, double yaw);
    void reset_odom();
    int update_states_rk4(double velocity, double steer, double dt=-1);
    geometry_msgs::TransformStamped add_static_link(double x, double y, double z, double roll, double pitch, double yaw, std::string parent, std::string child);
    void publish_cmd_vel(double steering_angle, double velocity = -3.57, bool clip = true);
    void lane_follow();
    void idle();
    double get_steering_angle(double offset=-20);
    void set_rate(double rateVal);
    double get_current_orientation();
    std::array<double, 3> get_real_states() const;
    boost::asio::io_service io;
    // boost::asio::serial_port serial;
    std::unique_ptr<boost::asio::serial_port> serial;
    double get_yaw() {
        return yaw;
    }
    int set_states(double x, double y) {
        x0 = x;
        y0 = y;
        odomX = 0;
        odomY = 0;
        return 0;
    }
    int get_states(double &x_, double &y_, double &yaw_) {
        if (subModel) {
            x_ = gps_x;
            y_ = gps_y;
        } else {
            x_ = odomX + x0;
            y_ = odomY + y0;
        }
        yaw_ = yaw;
        // if(useEkf) {
        //     // ROS_INFO("Using ekf: %.3f, %.3f", ekf_x, ekf_y);
        //     if (hasGps) {
        //         x_ = ekf_x;
        //         y_ = ekf_y;
        //     } else {
        //         ROS_INFO("Using ekf without gps: %.3f, %.3f", ekf_x, ekf_y);
        //         x_ = ekf_x + x0;
        //         y_ = ekf_y + y0;
        //     }
        // } else if(subModel) {
        //     // ROS_INFO("Using gps: %.3f, %.3f", gps_x, gps_y);
        //     x_ = gps_x;
        //     y_ = gps_y;
        // } else {
        //     // ROS_INFO("Using odom: %.3f, %.3f", odomX, odomY);
        //     x_ = odomX + x0;
        //     y_ = odomY + y0;
        // }
        return 0;
    }
    void update_states(Eigen::Vector3d& o_state) {
        if (subModel) {
            o_state << gps_x, gps_y, yaw;
        } else {
            o_state << odomX + x0, odomY + y0, yaw;
        }
    }
    int recalibrate_states(double x_offset, double y_offset) {
        if(useEkf) {
            if (hasGps) {
                x0 += x_offset;
                y0 += y_offset;
                // ekf_x += x_offset;
                // ekf_y += y_offset;
                // set_pose_using_service(ekf_x, ekf_y, yaw);
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
        if(useEkf) {
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

    // Eigen::Vector2d estimate_object_pose2d(double x, double y, double yaw, double x1, double y1, double x2, double y2, double object_distance, const std::array<double, 4>& camera_params, bool is_car = false) {
    Eigen::Vector2d estimate_object_pose2d(double x, double y, double yaw,
                                       double x1, double y1, double x2, double y2,
                                       double object_distance,
                                       const std::array<double, 4>& camera_params,
                                       bool is_car = false)
    {
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
        double fx = camera_params[0];
        double fy = camera_params[1];
        double cx = camera_params[2];
        double cy = camera_params[3];

        // Compute bounding box center in image coordinates
        double bbox_center_x = (x1 + x2) / 2;
        double bbox_center_y = (y1 + y2) / 2;

        // Convert image coordinates to normalized coordinates
        double x_norm = (bbox_center_x - cx) / fx;
        double y_norm = (bbox_center_y - cy) / fy;

        // Add distance from camera to robot center
        object_distance += CAMERA_POSE.x;
        // std::cout << "object_distance3: " << object_distance << std::endl;
        object_distance -= (0.07 + 0.05);
        if (is_car) object_distance += 0.05;

        // Estimate 3D coordinates in the camera frame
        double X_c = x_norm * object_distance;
        double Y_c = y_norm * object_distance;
        double Z_c = object_distance;

        // 3D point in the camera frame
        Eigen::Vector3d P_c(X_c, Y_c, Z_c);

        // Convert to vehicle coordinates (vehicle's x-axis is forward, y-axis is left/right)
        Eigen::Vector3d P_v(Z_c, -X_c, 0);

        // Rotation matrix from vehicle to world coordinates
        Eigen::Matrix2d R_vw;
        R_vw << std::cos(yaw), -std::sin(yaw),
                std::sin(yaw), std::cos(yaw);

        // Translate to world coordinates
        Eigen::Vector2d vehicle_pos(x, y);
        Eigen::Vector2d P_v_2d(P_v[0], P_v[1]);
        // std::cout << "object_distance4: " << P_v_2d[0] << std::endl;
        // std::cout << "relative position: " << P_v_2d[0] << ", " << P_v_2d[1] << ", yaw:" << yaw<< std::endl;
        Eigen::Vector2d world_coordinates = vehicle_pos + R_vw * P_v_2d;

        return world_coordinates;
    }
    Eigen::Vector2d estimate_object_pose2d(double x, double y, double yaw, const std::array<double, 4>& bounding_box, double object_distance, const std::array<double, 4>& camera_params, bool is_car = false) {
        double x1 = bounding_box[0];
        double y1 = bounding_box[1];
        double x2 = bounding_box[2];
        double y2 = bounding_box[3];
        return estimate_object_pose2d(x, y, yaw, x1, y1, x2, y2, object_distance, camera_params, is_car);
    }

    Eigen::Matrix3d euler2rot(double roll, double pitch, double yaw)
    {
        Eigen::AngleAxisd rollAngle(roll,   Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd yawAngle(yaw,     Eigen::Vector3d::UnitZ());

        // Depending on your convention, verify the rotation order:
        // For example, if you want R = Rz * Ry * Rx:
        Eigen::Matrix3d R = yawAngle.toRotationMatrix() *
                            pitchAngle.toRotationMatrix() *
                            rollAngle.toRotationMatrix();
        return R;
    }

    Eigen::Affine3d buildCameraToVehicleTF(const std::array<double, 6>& tf)
    {
        // tf = {tx, ty, tz, roll, pitch, yaw}
        double tx    = tf[0];
        double ty    = tf[1];
        double tz    = tf[2];
        double roll  = tf[3];
        double pitch = tf[4];
        double yaw   = tf[5];

        Eigen::Matrix3d R_cv = euler2rot(roll, pitch, yaw);
        Eigen::Vector3d t_cv(tx, ty, tz);

        // Construct an Affine3d
        Eigen::Affine3d T_cv = Eigen::Affine3d::Identity();
        T_cv.linear() = R_cv;
        T_cv.translation() = t_cv;

        return T_cv;
    }
    Eigen::Vector2d estimate_object_pose2d_new(double x, double y, double yaw,
                                       double x1, double y1, double x2, double y2,
                                       double object_distance,
                                       const std::array<double, 4>& camera_params,
                                       bool is_car = false)
    {
        static double parallel_w2h_ratio = 1.30;
        static double perpendicular_w2h_ratio = 2.70;

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
            
            object_distance += dist;
        }

        // Extract camera parameters
        double fx = camera_params[0];
        double fy = camera_params[1];
        double cx = camera_params[2];
        double cy = camera_params[3];

        // Compute bounding box center in image coordinates
        double bbox_center_x = (x1 + x2) / 2;
        double bbox_center_y = (y1 + y2) / 2;

        // Convert image coordinates to normalized coordinates
        double x_norm = (bbox_center_x - cx) / fx;
        double y_norm = (bbox_center_y - cy) / fy;

        // Estimate 3D coordinates in the camera frame
        double X_c = x_norm * object_distance;
        double Y_c = y_norm * object_distance;
        double Z_c = object_distance;
        
        Eigen::Vector3d P_c(X_c, Y_c, Z_c);
        // 3) Transform from camera frame to vehicle frame using REALSENSE_TF
        static const auto T_cv = buildCameraToVehicleTF(REALSENSE_TF);
        Eigen::Vector3d P_v = T_cv.linear() * P_c + T_cv.translation();

        // 4) Convert from vehicle frame to world frame (2D)
        //    We only keep (x, y) for a ground-plane assumption.
        //    (P_v[0], P_v[1]) is the vehicle x-y plane in your chosen forward/left coordinate system
        Eigen::Matrix2d R_vw;
        R_vw << std::cos(yaw), -std::sin(yaw),
                std::sin(yaw),  std::cos(yaw);

        Eigen::Vector2d vehicle_pos(x, y);
        Eigen::Vector2d P_v_2d(P_v[0], P_v[1]);

        Eigen::Vector2d world_coordinates = vehicle_pos + R_vw * P_v_2d;
        return world_coordinates;
    }
    Eigen::Vector2d estimate_object_pose2d_new(double x, double y, double yaw, const std::array<double, 4>& bounding_box, double object_distance, const std::array<double, 4>& camera_params, bool is_car = false) {
        double x1 = bounding_box[0];
        double y1 = bounding_box[1];
        double x2 = bounding_box[2];
        double y2 = bounding_box[3];
        return estimate_object_pose2d_new(x, y, yaw, x1, y1, x2, y2, object_distance, camera_params, is_car);
    }
    
    void send_speed(float f_velocity) {
        if (serial == nullptr) {
            debug("send_speed: Serial is null", 2);
            return;
        }
        std::stringstream strs;
        char buff[100];
        snprintf(buff, sizeof(buff), "%.2f;;\r\n", f_velocity * 100);
        strs << "#" << "1" << ":" << buff;
        boost::asio::write(*serial, boost::asio::buffer(strs.str()));
    }

    void send_steer(float f_angle) {
        if (serial == nullptr) {
            debug("send_steer: Serial is null", 2);
            return;
        }
        std::stringstream strs;
        char buff[100];
        snprintf(buff, sizeof(buff), "%.2f;;\r\n", f_angle);
        strs << "#" << "2" << ":" << buff;
        boost::asio::write(*serial, boost::asio::buffer(strs.str()));
    }

    void send_speed_and_steer(float f_velocity, float f_angle) {
        // ROS_INFO("speed:%.3f, angle:%.3f, yaw:%.3f, odomX:%.2f, odomY:%.2f, ekfx:%.2f, ekfy:%.2f", f_velocity, f_angle, yaw * 180 / M_PI, odomX, odomY, ekf_x-x0, ekf_y-y0);
        if (serial == nullptr) {
            debug("send_speed_and_steer(): Serial is null", 4);
            return;
        }
        if(f_angle > 3.0) f_angle+=4.0;
        std::stringstream strs;
        char buff[100];
        snprintf(buff, sizeof(buff), "%.2f:%.2f;;\r\n", f_velocity * 100, f_angle);
        strs << "#" << "8" << ":" << buff;
        boost::asio::write(*serial, boost::asio::buffer(strs.str()));
        // std::cout << strs.str() << std::endl;
    }

    std::array<double, 3> object_to_world(double object_x, double object_y, double object_yaw, 
                                double vehicle_x, double vehicle_y, double vehicle_yaw)
    {
        double world_x = vehicle_x + (std::cos(vehicle_yaw) * object_x - std::sin(vehicle_yaw) * object_y);
        double world_y = vehicle_y + (std::sin(vehicle_yaw) * object_x + std::cos(vehicle_yaw) * object_y);
        double world_yaw = object_yaw + vehicle_yaw;
        return {world_x, world_y, world_yaw};
    }

    void reset_yaw() {
        initial_yaw = yaw;
    }
    static double nearest_direction(double yaw) {
        yaw = yaw_mod(yaw, M_PI);

        static const double directions[5] = {0, M_PI / 2, M_PI, 3 * M_PI / 2, 2 * M_PI};

        double minDifference = std::abs(yaw - directions[0]);
        double nearestDirection = directions[0];

        for (int i = 1; i < 5; ++i) {
            double difference = std::abs(yaw - directions[i]);
            if (difference < minDifference) {
                minDifference = difference;
                nearestDirection = directions[i];
            }
        }
        while (nearestDirection - yaw > M_PI) {
            nearestDirection -= 2 * M_PI;
        }
        while (nearestDirection - yaw < -M_PI) {
            nearestDirection += 2 * M_PI;
        }
        return nearestDirection;
    }
    
    static int nearest_direction_index(double yaw) {
        yaw = yaw_mod(yaw, M_PI);

        static const double directions[5] = {0, M_PI / 2, M_PI, 3 * M_PI / 2, 2 * M_PI};

        double minDifference = std::abs(yaw - directions[0]);
        double nearestDirection = directions[0];

        int closest_index = 0;
        for (int i = 1; i < 5; ++i) {
            double difference = std::abs(yaw - directions[i]);
            if (difference < minDifference) {
                minDifference = difference;
                nearestDirection = directions[i];
                closest_index = i;
            }
        }
        if (closest_index == 4) {
            closest_index = 0;
        }
        return closest_index;
    }

    static double yaw_mod(double yaw, double ref=0) {
        while (yaw - ref > M_PI) yaw -= 2 * M_PI;
        while (yaw - ref <= -M_PI) yaw += 2 * M_PI;
        return yaw;
    }
    
    static std::string getSourceDirectory() {
        std::string file_path(__FILE__); 
        size_t last_dir_sep = file_path.rfind('/');
        if (last_dir_sep == std::string::npos) {
            last_dir_sep = file_path.rfind('\\'); 
        }
        if (last_dir_sep != std::string::npos) {
            return file_path.substr(0, last_dir_sep);  // Extract directory path
        }
        return "";  // Return empty string if path not found
    }

    void debug(const std::string& message, int level) {
        if (debugLevel >= level) {
            debug_msg.data = message;
            message_pub.publish(debug_msg);
            if (tcp_client != nullptr) tcp_client->send_message(debug_msg);
            ROS_INFO("%s", message.c_str());
        }
    }
};
