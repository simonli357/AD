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
#include "TrafficClient.hpp"
#include "utils/Lane2.h"
#include <std_srvs/Trigger.h>
#include <mutex>
#include <cmath>
#include <boost/asio.hpp>
#include "utils/constants.h"
#include "Tracking.h"
#include "Tunable.h"
#include <algorithm>

#include "LaneDetector.hpp"
#include "SignFastest.hpp"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.h"
#include "ros/ros.h"
#include "sensor_msgs/image_encodings.h"
#include "std_msgs/Header.h"
#include "yolo-fastestv2.h"
#include <chrono>
#include <librealsense2/rs.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Float32MultiArray.h>

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

    double l_r, l_f, odomRatio, maxspeed, center, image_center, p, d, last;
    int stopline = -1;
    double yaw, pitch = 0, height=0, velocity, steer_command, velocity_command, x_speed, y_speed;
    double odomX, odomY, odomYaw, dx, dy, dheight, dyaw, ekf_x, ekf_y, ekf_yaw, gps_x, gps_y;
    double initial_yaw = 0;
    double x0 = -1, y0 = -1, yaw0 = 0;
    std::string pathName;
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
    ros::Subscriber waypoints_sub;
    std::vector<float> detected_objects;
    ros::Subscriber model_sub;
    ros::Subscriber imu_sub;
    ros::Subscriber ekf_sub;
    ros::Subscriber tf_sub;

    // Initialization
    void initialize();
    void initialize_tcp_client();
    void fetch_run_params();

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
    void waypoints_callback(const std_msgs::Float32MultiArray::ConstPtr& msg);
    Eigen::MatrixXd lane_waypoints;
    void process_lane_data(const utils::Lane2& msg);
    void sign_callback(const utils::Sign::ConstPtr& msg);
    void process_sign_data(const utils::Sign& msg);
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
    boost::asio::io_service io;
    std::unique_ptr<boost::asio::serial_port> serial;
    double get_yaw() {
        return yaw;
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
        double X_c = x_norm * object_distance;
        double Y_c = y_norm * object_distance;
        // double Z_c = object_distance * std::sqrt(1 - x_norm*x_norm);
        double Z_c = object_distance;

        // 3D point in the camera frame
        static Eigen::Vector3d P_c;
        P_c << X_c, Y_c, Z_c;

        // Convert to vehicle coordinates (vehicle's x-axis is forward, y-axis is left/right)
        static Eigen::Vector3d P_v;
        P_v << Z_c, -X_c, 0;

        // std::cout << "before: " << P_v[0] << ", " << P_v[1] << std::endl;
        if (Tunable::real) {
            P_v[0] += REALSENSE_TF_REAL[0];
        } else {
            P_v[0] += REALSENSE_TF[0];
        }
        P_v[1] += REALSENSE_TF[1];
        // std::cout << "after: " << P_v[0] << ", " << P_v[1] << std::endl;

        // Rotation matrix from vehicle to world coordinates
        static Eigen::Matrix2d R_vw;
        R_vw << std::cos(yaw), -std::sin(yaw),
                std::sin(yaw), std::cos(yaw);

        // Translate to world coordinates
        static Eigen::Vector2d vehicle_pos;
        vehicle_pos << x, y;

        double latency = (ros::Time::now() - object_detection_time).toSec();
        P_v[0] -= latency * velocity_command;
        P_v[0] += sign_lon_offset_slope * P_v[0] + sign_lon_offset;
        P_v[1] += sign_lat_offset;
        static Eigen::Vector2d P_v_2d;
        P_v_2d << P_v[0], P_v[1];
        // std::cout << "object_distance4: " << P_v_2d[0] << std::endl;
        // std::cout << "relative position: " << P_v_2d[0] << ", " << P_v_2d[1] << ", yaw:" << yaw<< std::endl;

        return vehicle_pos + R_vw * P_v_2d;
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
        static bool first = true;
        static bool use_pid = false;
        if (serial == nullptr) {
            debug("send_speed_and_steer(): Serial is null", 4);
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
            
            boost::asio::write(*serial, boost::asio::buffer(pid_str.str()));
        }

        if(f_angle > 3.0) f_angle+=4.0;
        std::stringstream strs;
        char buff[100];
        snprintf(buff, sizeof(buff), "%.2f:%.2f;;\r\n", f_velocity * 100, f_angle);
        std::string number = use_pid ? "13" : "11";
        strs << "#" << number << ":" << buff;
        boost::asio::write(*serial, boost::asio::buffer(strs.str()));
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

    void reset_yaw() {
        initial_yaw = yaw;
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

    //----------- Camera Node ------------
    void cameraNodeConstructor(ros::NodeHandle& nh)
    {
        sign_ptr = std::make_unique<SignFastest>(nh);
        lane_ptr = std::make_unique<LaneDetector>(nh);
		depthImage = cv::Mat::zeros(480, 640, CV_16UC1);
		colorImage = cv::Mat::zeros(480, 640, CV_8UC3);
		std::string nodeName = ros::this_node::getName();
		nh.param(nodeName + "/lane", doLane, true);
		nh.param(nodeName + "/sign", doSign, true);
		nh.param(nodeName + "/realsense", realsense, false);
		nh.param(nodeName + "/rate", realsense_rate, 30.0);
        nh.param(nodeName + "/sign_rate", sign_rate, 30.0);
        nh.param(nodeName + "/lane_rate", lane_rate, 60.0);
		nh.param(nodeName + "/pubImage", pubImage, false);
		nh.param(nodeName + "/thread", useRosTimer, false);

		if (!realsense) {
			if (sign_ptr->hasDepthImage) {
				std::string topic;
				bool is_real;
				if (!nh.getParam(nodeName + "/real", is_real)) {
					ROS_WARN("Failed to get 'real' parameter. Defaulting to false.");
				}
				if (is_real) {
					std::cout << "real, depth topic is /camera/aligned_depth_to_color/image_raw" << std::endl;
					topic = "/camera/aligned_depth_to_color/image_raw";
				} else {
					std::cout << "not real, depth topic is /camera/depth/image_raw" << std::endl;
					topic = "/camera/depth/image_raw";
				}
				depth_sub = it.subscribe(topic, 3, &Utility::depthCallback, this);
				std::cout << "depth_sub created, waiting for " << topic << std::endl;
				ros::topic::waitForMessage<sensor_msgs::Image>(topic, nh);
				std::cout << "got it" << std::endl;
			}
			rgb_sub = it.subscribe("/camera/color/image_raw", 3, &Utility::imageCallback, this);
			std::cout << "waiting for rgb image" << std::endl;
			ros::topic::waitForMessage<sensor_msgs::Image>("/camera/color/image_raw", nh);
			std::cout << "got color image" << std::endl;
		} else {
			align_to_color = std::make_unique<rs2::align>(RS2_STREAM_COLOR);
			depth_frame = rs2::frame();
			color_frame = rs2::frame();
			data = rs2::frameset();

			cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
			cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
			cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);
			cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
			pipe.start(cfg);

			auto profiles = pipe.get_active_profile().get_streams();

			for (auto &&p : profiles)
			{
                if (p.stream_type() == RS2_STREAM_COLOR)
                {
                    auto vid_profile = p.as<rs2::video_stream_profile>();
                    rs2_intrinsics intr = vid_profile.get_intrinsics();

                    double fx = intr.fx;
                    double fy = intr.fy;
                    double cx = intr.ppx; // principal point x
                    double cy = intr.ppy; // principal point y
                    ROS_INFO("camera intrinsics: fx=%.2f, fy=%.2f, cx=%.2f, cy=%.2f", fx, fy, cx, cy);
                    break;
                }
			}

			std::cout.precision(4);
			if (pubImage) {
				color_pub = nh.advertise<sensor_msgs::Image>("/camera/color/image_raw", 1);
				depth_pub = nh.advertise<sensor_msgs::Image>("/camera/depth/image_raw", 1);
				std::cout << "pub created" << std::endl;
			}
			cameraThreadRunning = true;
			cameraThread = std::thread(&Utility::cameraThreadFunc, this);
			std::cout << "camera thread created" << std::endl;
		}

		if (!doLane) {
			ROS_WARN("Lane detection is disabled");
		}
		if (!doSign) {
			ROS_WARN("Sign detection is disabled");
		}
		if (useRosTimer) {
			ROS_INFO("RosTimer is enabled");
		} else {
			ROS_INFO("RosTimer is disabled");
		}

		if (useRosTimer) {
			if (doLane) {
				ROS_INFO("starting lane timer");
				laneTimer = nh.createTimer(ros::Duration(1.0 / lane_rate), &Utility::lane_timer_callback, this);
			}
			if (doSign) {
				ROS_INFO("starting sign timer");
				signTimer = nh.createTimer(ros::Duration(1.0 / sign_rate), &Utility::sign_timer_callback, this);
			}
            if (realsense) {
                ROS_INFO("starting realsense timer");
                realsenseTimer = nh.createTimer(ros::Duration(1.0 / realsense_rate), &Utility::realsense_timer_callback, this);
            }
		}
	}

	// SignFastest Sign;
	// LaneDetector Lane;
    std::unique_ptr<SignFastest> sign_ptr;
    std::unique_ptr<LaneDetector> lane_ptr;
    
	sensor_msgs::ImagePtr color_msg, depth_msg;

	image_transport::Subscriber rgb_sub;
	image_transport::Subscriber depth_sub;
	image_transport::ImageTransport it;
	cv::Mat depthImage, colorImage;
	cv_bridge::CvImagePtr cv_ptr;
	cv_bridge::CvImagePtr cv_ptr_depth;
	ros::Timer signTimer, laneTimer, realsenseTimer;

	bool doLane, doSign, realsense, pubImage, useRosTimer;
	double realsense_rate, sign_rate, lane_rate;

	// lock
	std::thread cameraThread;
	bool cameraThreadRunning;
	std::mutex image_mutex;
	void cameraThreadFunc() {
        ros::Rate cameraRate(30);
        while (ros::ok() && cameraThreadRunning) {
            get_frame();
            cameraRate.sleep();
        }
	}

	// rs
	ros::Publisher color_pub, depth_pub;

	rs2::pipeline pipe;
	rs2::config cfg;
	rs2::frame color_frame;
	rs2::frame depth_frame;
	rs2::frameset data;
	rs2::frame gyro_frame;
	rs2::frame accel_frame;
	std::unique_ptr<rs2::align> align_to_color;

	void depthCallback(const sensor_msgs::ImageConstPtr &msg) {
		cv_ptr_depth = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
		if (cv_ptr_depth == nullptr) {
			ROS_WARN("cv_ptr_depth is null");
			return;
		}
		{
			std::lock_guard<std::mutex> lock(image_mutex);
			depthImage = cv_ptr_depth->image.clone();
		}
		if (sign_ptr->tcp_client != nullptr) {
        	sign_ptr->tcp_client->send_image_depth(depthImage);
		}
	}
	void imageCallback(const sensor_msgs::ImageConstPtr &msg) {
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		if (cv_ptr == nullptr) {
			ROS_WARN("cv_ptr is null");
			return;
		}
		{
			std::lock_guard<std::mutex> lock(image_mutex);
            colorImage = cv_ptr->image.clone();
		}
		if (sign_ptr->tcp_client != nullptr) {
        	sign_ptr->tcp_client->send_image_rgb(colorImage);
		}
	}

	void lane_timer_callback(const ros::TimerEvent &event) { run_lane_once(); }
	void sign_timer_callback(const ros::TimerEvent &event) { run_sign_once(); }
    void realsense_timer_callback(const ros::TimerEvent &event) { get_frame(); }
	void run_lane_once() {
		if (colorImage.empty()) {
			ROS_WARN("colorImage is empty");
			return;
		}
        {
            std::lock_guard<std::mutex> lock(image_mutex);
		    lane_ptr->publish_lane(colorImage);
        }
        process_lane_data(lane_ptr->lane_msg);
	}
	void run_sign_once() {
		if (colorImage.empty()) {
			ROS_WARN("colorImage is empty");
			return;
		}
		if (depthImage.empty()) {
			ROS_WARN("depthImage is empty");
			return;
		}
        {
            std::lock_guard<std::mutex> lock(image_mutex);
		    sign_ptr->publish_sign(colorImage, depthImage);
        }
        process_sign_data(sign_ptr->sign_msg);
	}

    void get_frame() {
		data = pipe.wait_for_frames();
		auto aligned_frames = align_to_color->process(data);
		color_frame = aligned_frames.get_color_frame();
		depth_frame = aligned_frames.get_depth_frame();
		gyro_frame = data.first_or_default(RS2_STREAM_GYRO);
		accel_frame = data.first_or_default(RS2_STREAM_ACCEL);
		if (!color_frame || !depth_frame) {
			ROS_WARN("No frame received");
			return;
		}
        {
            std::lock_guard<std::mutex> lock(image_mutex);
            colorImage = cv::Mat(cv::Size(640, 480), CV_8UC3, (void *)color_frame.get_data(), cv::Mat::AUTO_STEP);
            depthImage = cv::Mat(cv::Size(640, 480), CV_16UC1, (void *)depth_frame.get_data(), cv::Mat::AUTO_STEP);
        }

		// if (!useRosTimer) {
		// 	if (doLane) {
		// 		run_lane_once();
		// 	}
		// 	if (doSign) {
		// 		run_sign_once();
		// 	}
		// }
		if (pubImage) {
			color_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", colorImage).toImageMsg();
			depth_msg = cv_bridge::CvImage(std_msgs::Header(), "mono16", depthImage).toImageMsg();
			if (sign_ptr->tcp_client != nullptr) {
				sign_ptr->tcp_client->send_image_rgb(colorImage);
				sign_ptr->tcp_client->send_image_depth(depthImage);
			}
			color_pub.publish(color_msg);
			depth_pub.publish(depth_msg);
		}
	}
};
