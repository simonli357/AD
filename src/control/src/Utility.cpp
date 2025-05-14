#include <chrono>
#include <ros/ros.h>
#include "TcpClient.hpp"
#include "TrafficClient.hpp"
#include "Utility.hpp"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32MultiArray.h>
#include <utils/Sign.h>
#include <std_msgs/String.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/Imu.h>
#include <gazebo_msgs/ModelStates.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/impl/utils.h>
#include <tf2/utils.h>
#include <vector>
#include <array>
#include <eigen3/Eigen/Dense>
#include <std_srvs/Trigger.h>
#include <mutex>
#include <cmath>
#include <robot_localization/SetPose.h>
#include <iostream>
#include "Runs.h"
#include "PathManager.h"
#include "Tunable.h"

Utility::Utility(ros::NodeHandle& nh_, bool pubOdom) 
    : nh(nh_), pubOdom(pubOdom), object_detection_time(ros::Time::now())
{
    std::cout << "Utility constructor" << std::endl;  
    message_pub = nh.advertise<std_msgs::String>("/message", 10);
    initialize_tcp_client();
    initialize();
    fetch_run_params();
}

Utility::~Utility() {
    stop_car(); 
}

void Utility::initialize_tcp_client() {
    if(Tunable::use_traffic_server) {
        debug("Utility constructor: Attempting to create Traffic Server TCP client...", 1);
        traffic_client = std::make_unique<TrafficClient>(Tunable::traffic_server_ip);
        debug("Utility constructor: TCP client created successfully.", 1);
    }
    if(Tunable::use_tcp) {
        debug("Utility constructor: Attempting to create TCP client...", 1);
        tcp_client = std::make_shared<TcpClient>(true, "utility_node_client", Tunable::ip_address);
        debug("Utility constructor: TCP client created successfully.", 1);
    } else {
        tcp_client = nullptr;
        debug("Utility constructor: TCP client not created.", 1);
    }
}

void Utility::fetch_run_params() {
    if (!Tunable::useGps) {
        debug("GPS not found, skipping", 1);
        return;
    }

    this->pathName = "gps";

    const size_t sample_count = 15;
    std::vector<geometry_msgs::PoseWithCovarianceStamped::ConstPtr> samples;
    samples.reserve(sample_count);

    auto gps_cb = [&](const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
        if (samples.size() < sample_count) {
            samples.push_back(msg);
        }
    };

    ros::Subscriber sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/gps", 100, gps_cb);
    ros::Time start_time = ros::Time::now();
    ros::Rate rate(100);
    std::cout << "Utility::fetch_run_params: waiting for GPS data..." << std::endl;
    while (ros::ok() && samples.size() < sample_count && (ros::Time::now() - start_time).toSec() < 5.0) {
        ros::spinOnce();
        rate.sleep();
    }

    sub.shutdown();

    double sum_x = 0.0, sum_y = 0.0;
    for (const auto& m : samples) {
        sum_x += m->pose.pose.position.x;
        sum_y += m->pose.pose.position.y;
    }
    double avg_x = sum_x / static_cast<double>(samples.size());
    double avg_y = sum_y / static_cast<double>(samples.size());

    std::cout << "Utility::fetch_run_params: avg_x: " << avg_x << ", avg_y: " << avg_y << ", now waiting for IMU" << std::endl;
    std::cout << "Utility::fetch_run_params: IMU initialized, now calculating yaw" << std::endl;

    this->x0   = avg_x;
    this->y0   = avg_y;
    this->yaw0 = Sensing::yaw;
    debug("Utility::fetch_run_params: success: x0: " + std::to_string(x0) + ", y0: " + std::to_string(y0) + ", yaw0: " + std::to_string(yaw0), 1);
}

void Utility::initialize() {
    for (int i = 0; i < 5; ++i) {
        add_velocity_command(0.0);
    }
    // tunables
    double sigma_v = 0.1;
    double sigma_delta = 10.0; // degrees

    q_transform.setRPY(REALSENSE_TF[3], REALSENSE_TF[4], REALSENSE_TF[5]); // 3 values are roll, pitch, yaw of the imu
    rate = new ros::Rate(Tunable::rateVal);
    if (Tunable::real) {
        l_r = L_R_REAL;
        l_f = L_F_REAL;
    } else {
        l_r = L_R_SIM;
        l_f = L_F_SIM;
    }
    odomRatio = 1.0;
    maxspeed = 1.5;
    center = -1;
    image_center = 320;
    p = 0.005;
    d = 0.0005;
    last = 0;
    
    triggerServiceClient = nh.serviceClient<std_srvs::Trigger>("trigger_service");
    static_broadcaster = tf2_ros::StaticTransformBroadcaster();
    broadcaster = tf2_ros::TransformBroadcaster();
    publish_static_transforms();

    velocity = 0.0;
    odomX = 0.0;
    odomY = 0.0;
    odomYaw = yaw0;
    ekf_x = x0;
    ekf_y = y0;
    ekf_yaw = yaw0;
    initializationFlag = false;
    gps_x = x0;
    gps_y = y0;
    steer_command = 0.0;
    velocity_command = 0.0;

    initializationTimer = std::nullopt;
    timerpid = std::nullopt;

    std::fill(std::begin(odom_msg.pose.covariance), std::end(odom_msg.pose.covariance), 0.0);
    std::fill(std::begin(odom_msg.twist.covariance), std::end(odom_msg.twist.covariance), 0.0);

    double dt = 1.0 / Tunable::odom_rate;
    double variance_v = sigma_v * sigma_v;
    double sigma_delta_rad = sigma_delta * M_PI / 180;
    double variance_yaw_rate = std::pow((sigma_v / 0.27 * std::tan(sigma_delta_rad)), 2);
    double variance_x = variance_v * dt * dt;
    double variance_y = variance_v * dt * dt;
    odom_msg.pose.covariance[0] = variance_x;
    odom_msg.pose.covariance[7] = variance_y;
    odom_msg.pose.covariance[35] = variance_yaw_rate * dt * dt;
    odom_msg.twist.covariance[0] = variance_v;
    odom_msg.twist.covariance[7] = variance_yaw_rate;

    odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 3);
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "chassis";

    cmd_vel_pub = nh.advertise<std_msgs::String>("/" + robot_name + "/command", 8);
    waypoints_pub = nh.advertise<std_msgs::Float32MultiArray>("/waypoints", 3);
    state_offset_pub = nh.advertise<std_msgs::Float32MultiArray>("/state_offset", 3);
    
    if (pubOdom) {
        odom_pub_timer = nh.createTimer(ros::Duration(1.0 / Tunable::odom_rate), &Utility::odom_pub_timer_callback, this);
    }
    if (Tunable::ekf) {
        ekf_sub = nh.subscribe("/odometry/filtered", 3, &Utility::ekf_callback, this);
    } 
    if (subModel) {
        std::cout << "SUBMODEL IS TRUEEE!!!" << std::endl;
        model_sub = nh.subscribe("/gazebo/model_states", 3, &Utility::model_callback, this);
    }

    if (true) {
        lane_sub = nh.subscribe("/lane", 3, &Utility::lane_callback, this);
        lane_center_offset_sub = nh.subscribe("/lane_center_offset", 3, &Utility::lane_center_offset_callback, this);
        int horizon = 40;
        lane_waypoints = Eigen::MatrixXd(horizon, 3);
        waypoints_sub = nh.subscribe("/lane_waypoints", 3, &Utility::waypoints_callback, this);
        // std::cout << "waiting for lane message" << std::endl;
        // ros::topic::waitForMessage<utils::Lane2>("/lane");
        // std::cout << "received message from lane" << std::endl;
    }

    timerpid = ros::Time::now();
    if (Tunable::sign) {
        sign_sub = nh.subscribe("/sign", 3, &Utility::sign_callback, this);
        std::cout << "waiting for sign message" << std::endl;
        ros::topic::waitForMessage<utils::Sign>("/sign");
        std::cout << "received message from sign" << std::endl;
        car_pose_pub = nh.advertise<std_msgs::Float32MultiArray>("/car_locations", 10);
        road_object_pub = nh.advertise<std_msgs::Float32MultiArray>("/road_objects", 10);
    }

    timerodom = ros::Time::now();
    debug("Utility::initialize(): successful.", 1);
}

void Utility::odom_pub_timer_callback(const ros::TimerEvent&) {
    publish_odom();
}

void Utility::sign_callback(const utils::Sign::ConstPtr& msg) {
    process_sign_data(*msg);   
}
void Utility::process_sign_data(const utils::Sign& msg) {
    if (msg.data.size()) {
        num_obj = msg.data.size() / NUM_VALUES_PER_OBJECT;
        {
            // std::lock_guard<std::mutex> lock(general_mutex);
            detected_objects.assign(msg.data.begin(), msg.data.end());
        }
    } else {
        num_obj = 0;
    }
    if (num_obj == 1 && detected_objects[0] == -1.0) {
        emergency = true;
    } else {
        emergency = false;
    }
    std::lock_guard<std::mutex> lock(Tracking::container_mutex);
    object_detection_time = msg.header.stamp;
    double ego_x, ego_y, ego_yaw;
    get_states(ego_x, ego_y, ego_yaw);
    // std::cout << "sign_callback(): ego_x: " << ego_x << ", ego_y: " << ego_y << ", ego_yaw: " << ego_yaw << ", num_obj: " << num_obj << std::endl;
    // Tracking::ego_car->update(ego_x, ego_y, ego_yaw, filtered_encoder_speed, height, steer_command);
    Tracking::ego_car->update(ego_x, ego_y, ego_yaw, velocity_command, height, steer_command);
    Tracking::predict_dynamic_objects();
    for(int i = 0; i < num_obj; i++) {
        double dist = object_distance(i);
        if(dist > 4.0 || dist < 0.3) continue;
        auto type = static_cast<OBJECT>(msg.data[i * NUM_VALUES_PER_OBJECT + VehicleConstants::id]);
        double confidence = msg.data[i * NUM_VALUES_PER_OBJECT + VehicleConstants::confidence];
        bool found_same = false;

        double xmin = msg.data[i * NUM_VALUES_PER_OBJECT + VehicleConstants::x1];
        double ymin = msg.data[i * NUM_VALUES_PER_OBJECT + VehicleConstants::y1];
        double xmax = msg.data[i * NUM_VALUES_PER_OBJECT + VehicleConstants::x2];
        double ymax = msg.data[i * NUM_VALUES_PER_OBJECT + VehicleConstants::y2];
        double latency = (ros::Time::now() - object_detection_time).toSec();
        bool is_car = type == OBJECT::CAR;
        Eigen::Vector2d world_states = estimate_object_pose2d(ego_x, ego_y, ego_yaw, xmin, ymin, xmax, ymax, dist, is_car);
        // std::cout << "sign_callback(): detected object: " << OBJECT_NAMES[type] << ", confidence: " << confidence << ", latency: " << latency << ", type: " << type << ", iscar: " << is_car << ", worldstates: (" << world_states[0] << ", " << world_states[1] << ")" << std::endl;
        bool is_known_static = Tracking::is_known_static_object(type);
        auto* road_objects = Tracking::get_road_objects(type);
        if (!road_objects) {
            debug("Sign Callback(): Skipping object due to null road_objects for type: " + std::to_string(type), 1);
            return;
        }
        int min_index = -1;
        double min_error = 1000.0;
        for (int i = 0; i < road_objects->size(); ++i) {
            auto& obj = (*road_objects)[i];
            double error = std::hypot(obj->x - world_states[0], obj->y - world_states[1]);
            if (error < min_error) {
                min_index = i;
                min_error = error;
            }
        }
        if (min_index >= 0) {
            auto closest_obj = (*road_objects)[min_index];
            if (closest_obj->is_same_object(world_states[0], world_states[1]) && closest_obj->is_same_type(type)) {
                found_same = true;
                closest_obj->merge(world_states[0], world_states[1], ego_yaw, confidence, type);
            }
        }
        if (!found_same) {
            if (is_known_static) {
                std::string sign_name;
                const auto& relevant_signs = get_relevant_signs(type, sign_name);
                int min_index = 0;
                double min_error_sq = 1000.0;
                Eigen::Vector2d sign_pose = {world_states[0], world_states[1]};

                if (helper::get_min_object_index(sign_pose, relevant_signs, min_index, min_error_sq, 0.357)) {
                    double sign_yaw = relevant_signs[min_index][2];
                    double yaw_error = helper::compare_yaw(sign_yaw, ego_yaw);

                    if (yaw_error < 45 * M_PI / 180) {
                        Tracking::create_known_static_object(static_cast<OBJECT>(type),
                            world_states[0], world_states[1], Sensing::yaw, confidence, relevant_signs[min_index]);

                        // debug("Sign Callback(): new " + sign_name + " (known static object) detected at (" +
                            // std::to_string(relevant_signs[min_index][0]) + ", " +
                            // std::to_string(relevant_signs[min_index][1]) + "), known static objects size: " +
                            // std::to_string(Tracking::road_known_static_objects.size()), 2);
                    }
                }
            } else {
                double object_yaw = ego_yaw;
                if (type == OBJECT::CAR) {
                    int closest_index = PathManager::find_closest_waypoint2(world_states, 0.25);
                    if (closest_index >= 0) {
                        object_yaw = PathManager::state_refs_original(closest_index, 2);
                        // debug("Sign Callback()!!: new CAR detected at (" +
                        //     std::to_string(world_states[0]) + ", " + std::to_string(world_states[1]) +
                        //     "), closest waypoint: " + std::to_string(closest_index) + ", object_yaw: " +
                        //     std::to_string(object_yaw), 2);
                    } else {
                        // TODO: Check against known parking spots
                        double min_error_sq = 1000.0;
                        int min_index = 0;
                        if(helper::get_min_object_index(world_states, GroundTruth::PARKING_SPOTS, min_index, min_error_sq, 0.15)) {
                            // this means the detected object is a parked car. all parking spots are facing east
                            object_yaw = 0.0;
                            // debug("Sign Callback(): new PARKED CAR detected at (" +
                            //     std::to_string(world_states[0]) + ", " + std::to_string(world_states[1]) +
                            //     "), closest parking spot: (" + std::to_string(GroundTruth::PARKING_SPOTS[min_index][0]) + ", " +
                            //     std::to_string(GroundTruth::PARKING_SPOTS[min_index][1]) + "), object_yaw: " +
                            //     std::to_string(object_yaw), 2);
                        }
                    }
                }
                Tracking::create_object(static_cast<OBJECT>(type), world_states[0], world_states[1], object_yaw, confidence);
                // debug("Sign Callback(): new " + OBJECT_NAMES[type] + " detected at (" +
                //     std::to_string(world_states[0]) + ", " + std::to_string(world_states[1]) +
                //     "), road_objects size: " + std::to_string(road_objects->size()), 2);
            }
        }
    }
    Tracking::cleanup_stale_objects();
    auto road_object_msg = Tracking::create_all_msgs();
    static bool publish_objects = true;
    if(publish_objects) {
        road_object_pub.publish(road_object_msg);
        // safety check
        if (tcp_client != nullptr) {
            tcp_client->send_road_object(road_object_msg);
        }
        if (traffic_client != nullptr) {
            traffic_client->send_car_data(road_object_msg);
        }
    }
}

void Utility::lane_callback(const utils::Lane2::ConstPtr& msg) {
    process_lane_data(*msg);
}
void Utility::lane_center_offset_callback(const std_msgs::Float32::ConstPtr& msg) {
    lane_center_offset = msg->data;
}
void Utility::waypoints_callback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
    if(msg->data.size() < lane_waypoints.size()/3) {
        ROS_WARN("waypoints_callback: received fewer waypoints than expected: %lu", msg->data.size());
        return;
    }
    for(int i = 0; i < lane_waypoints.size(); i+=3) {
        lane_waypoints(i/3, 0) = msg->data[i];
        lane_waypoints(i/3, 1) = msg->data[i+1];
        lane_waypoints(i/3, 2) = msg->data[i+2];
    }
}
void Utility::process_lane_data(const utils::Lane2& msg) {
    {
        // std::lock_guard<std::mutex> lock(general_mutex);
        center = msg.center;
        stopline = msg.stopline;
    }
    tcp_client->send_lane2(msg);
}

void Utility::ekf_callback(const nav_msgs::Odometry::ConstPtr& msg) {
    // std::lock_guard<std::mutex> lock(general_mutex);
    // ros::Time now = ros::Time::now();
    double gps_offset_x = 0.0;
    double gps_offset_y = 0.0;
    double offset_x = gps_offset_x * std::cos(Sensing::yaw) - gps_offset_y * std::sin(Sensing::yaw);
    double offset_y = gps_offset_x * std::sin(Sensing::yaw) + gps_offset_y * std::cos(Sensing::yaw);
    ekf_x = msg->pose.pose.position.x + offset_x;
    ekf_y = msg->pose.pose.position.y + offset_y;
    // x0 = ekf_x - odomX;
    // y0 = ekf_y - odomY;
    // tf2::fromMsg(msg->pose.pose.orientation, tf2_quat);
    // ekf_yaw = tf2::impl::getYaw(tf2_quat);
}
void Utility::model_callback(const gazebo_msgs::ModelStates::ConstPtr& msg) {
    // std::lock_guard<std::mutex> lock(general_mutex);
    if (!car_idx.has_value()) {
        auto it = std::find(msg->name.begin(), msg->name.end(), robot_name);
        if (it != msg->name.end()) {
            car_idx = std::distance(msg->name.begin(), it);
            std::cout << "automobile found: " << *car_idx << std::endl;
        } else {
            printf("automobile not found\n");
            return; 
        }
    }
    auto& car_inertial = msg->twist[*car_idx];
    x_speed = msg->twist[*car_idx].linear.x;
    y_speed = msg->twist[*car_idx].linear.y;
    gps_x = msg->pose[*car_idx].position.x;
    gps_y = msg->pose[*car_idx].position.y;
}

void Utility::stop_car() {
    // std::cout << "Stopping car" << std::endl;
    msg.data = "{\"action\":\"1\",\"speed\":" + std::to_string(0.0) + "}";
    msg2.data = "{\"action\":\"2\",\"steerAngle\":" + std::to_string(0.0) + "}";
    for (int i = 0; i < 10; i++) {
        publish_cmd_vel(0.0, 0.0);
        ros::Duration(0.15).sleep();
    }
    // std::cout << "sent commands to stop car" << std::endl;
}

void Utility::publish_odom() {
    {
        filtered_encoder_speed = filter_encoder(Sensing::encoder_speed);
        update_states_rk4(filtered_encoder_speed, steer_command);
        {
            odomX += dx;
            odomY += dy;
        }
        height -= dheight;
        // ROS_INFO("odomX: %.3f, gps_x: %.3f, odomY: %.3f, gps_y: %.3f, error: %.3f", odomX, gps_x, odomY, gps_y, sqrt((odomX - gps_x) * (odomX - gps_x) + (odomY - gps_y) * (odomY - gps_y))); // works
    }
}

int Utility::object_index(int obj_id) {
    std::lock_guard<std::mutex> lock(general_mutex);
    if (num_obj < 1) {
        return -1;
    }
    if (num_obj == 1) {
        if (detected_objects[id] == obj_id) {
            return 0;
        }
        return -1;
    }
    for (int i = 0; i < num_obj; ++i) {
        if (detected_objects[i * NUM_VALUES_PER_OBJECT + id] == obj_id) {
            return i;
        }
    }
    return -1;
}

std::vector<int> Utility::object_indices(int obj_id) {
    std::lock_guard<std::mutex> lock(general_mutex);
    std::vector<int> indices;
    if (num_obj < 1) {
        return indices;
    }
    if (num_obj == 1) {
        if (detected_objects[id] == obj_id) {
            indices.push_back(0);
        }
        return indices;
    }
    for (int i = 0; i < num_obj; ++i) {
        if (detected_objects[i * NUM_VALUES_PER_OBJECT + id] == obj_id) {
            indices.push_back(i);
        }
    }
    return indices;
}

double Utility::object_distance(int index) {
    std::lock_guard<std::mutex> lock(general_mutex);
    if (num_obj == 1) {
        return detected_objects[distance];
    } else if (index >= 0 && index < num_obj) {
        return detected_objects[index * NUM_VALUES_PER_OBJECT + distance];
    }
    return -1;
}

std::array<double, 4> Utility::object_box(int index) {
    // std::lock_guard<std::mutex> lock(general_mutex);
    std::array<double, 4> box;

    if (num_obj == 1) {
        box[0] = detected_objects[VehicleConstants::x1];
        box[1] = detected_objects[VehicleConstants::y1];
        box[2] = detected_objects[VehicleConstants::x2];
        box[3] = detected_objects[VehicleConstants::y2];
    } else if (index >= 0 && index < num_obj) {
        int startIndex = index * NUM_VALUES_PER_OBJECT;
        box[0] = detected_objects[startIndex + VehicleConstants::x1];
        box[1] = detected_objects[startIndex + VehicleConstants::y1];
        box[2] = detected_objects[startIndex + VehicleConstants::x2];
        box[3] = detected_objects[startIndex + VehicleConstants::y2];
    }

    return box;
}
void Utility::object_box(int index, std::array<double, 4>& oBox) {
    // std::lock_guard<std::mutex> lock(general_mutex);
    if (num_obj == 1) {
        oBox[0] = detected_objects[VehicleConstants::x1];
        oBox[1] = detected_objects[VehicleConstants::y1];
        oBox[2] = detected_objects[VehicleConstants::x2];
        oBox[3] = detected_objects[VehicleConstants::y2];
    } else if (index >= 0 && index < num_obj) {
        int startIndex = index * NUM_VALUES_PER_OBJECT;
        oBox[0] = detected_objects[startIndex + VehicleConstants::x1];
        oBox[1] = detected_objects[startIndex + VehicleConstants::y1];
        oBox[2] = detected_objects[startIndex + VehicleConstants::x2];
        oBox[3] = detected_objects[startIndex + VehicleConstants::y2];
    }
}
void Utility::set_initial_pose(double x, double y, double yaw) {
    // std::lock_guard<std::mutex> lock(general_mutex);
    // initializationTimer = ros::Time::now();
    debug("Setting initial pose: x: " + std::to_string(x) + ", y: " + std::to_string(y) + ", yaw: " + std::to_string(yaw), 1);
    odomX = x;
    odomY = y;
    odomYaw = yaw;
}
void Utility::reset_odom() {
    set_initial_pose(0, 0, 0);
}
int Utility::update_states_rk4 (double speed, double steering_angle, double dt) {
    static auto timer2 = std::chrono::steady_clock::now();
    if (dt < 0) {
        dt = (ros::Time::now() - timerodom).toSec();
        timerodom = ros::Time::now();
        double dt2 = (std::chrono::steady_clock::now() - timer2).count() / 1e9;
        timer2 = std::chrono::steady_clock::now();
        // std::cout << "ros time: " << dt << ", steady time: " << dt2 << std::endl;
        dt = dt2;
    }
    double yaw = Sensing::yaw;
    double pitch = Sensing::pitch;
    if (std::abs(pitch) <3 * M_PI / 180) {
        pitch = 0;
    }
    dheight = speed * dt * odomRatio * sin(pitch);
    double v_eff = speed * cos(pitch);

    double delta_rad = -steering_angle * M_PI / 180.0;
    double beta = 0;
    if (Tunable::use_beta) beta = atan((l_r / WHEELBASE) * tan(delta_rad));

    double magnitude = v_eff * dt * odomRatio;
    double yaw_rate = dt * magnitude * tan(delta_rad) / WHEELBASE * cos(beta);

    double k1_x = magnitude * cos(yaw + beta);
    double k1_y = magnitude * sin(yaw + beta);

    double k2_x = magnitude * cos((yaw + yaw_rate / 2) + beta);
    double k2_y = magnitude * sin((yaw + yaw_rate / 2) + beta);

    double k3_x = magnitude * cos((yaw + yaw_rate / 2) + beta);
    double k3_y = magnitude * sin((yaw + yaw_rate / 2) + beta);

    double k4_x = magnitude * cos((yaw + yaw_rate / 2) + beta);
    double k4_y = magnitude * sin((yaw + yaw_rate / 2) + beta);

    dx = 1 / 6.0 * (k1_x + 2 * k2_x + 2 * k3_x + k4_x);
    dy = 1 / 6.0 * (k1_y + 2 * k2_y + 2 * k3_y + k4_y);
    dyaw = yaw_rate;
    // printf("dt: %.3f, v: %.3f, yaw: %.3f, steer: %.3f, dx: %.3f, dy: %.3f, dyaw: %.3f\n", dt, speed, yaw, steering_angle, dx, dy, dyaw);
    return 1;
}
void Utility::publish_cmd_vel(double steering_angle, double velocity, bool clip) {
    if (velocity < -3.5) velocity = maxspeed;
    if (clip) {
        if (steering_angle > HARD_MAX_STEERING) steering_angle = HARD_MAX_STEERING;
        if (steering_angle < -HARD_MAX_STEERING) steering_angle = -HARD_MAX_STEERING;
    }
    
    // Check for NaN values and handle them
    if (std::isnan(steering_angle)) {
        std::cerr << "Error: Steering angle is NaN!" << std::endl;
        steering_angle = 0.0;
    }

    if (std::isnan(velocity)) {
        std::cerr << "Error: Velocity is NaN!" << std::endl;
        velocity = 0.0;
    }
    {
        steer_command = steering_angle;
        velocity_command = velocity;
        add_velocity_command(velocity_command);
    }
    // apply offset correction
    if(std::abs(steering_angle) > steer_offset_minimum && std::abs(steering_angle) < steer_offset_maximum) {
        steering_angle += steer_offset * std::abs(steering_angle) / steering_angle;
    }
    if(std::abs(velocity) > 0.1) {
        velocity += speed_offset * std::abs(velocity) / velocity;
    }
    // std::cout << "after: " << steering_angle << ", " << velocity << std::endl;

    float vel = velocity;
    float steer = steering_angle;
    tcp_client->send_steer(steer);
    if(true) {
        send_speed_and_steer(vel, steer);
    }
    msg2.data = "{\"action\":\"2\",\"steerAngle\":" + std::to_string(steer) + "}";
    cmd_vel_pub.publish(msg2);
    // ros::Duration(0.03).sleep();
    msg.data = "{\"action\":\"1\",\"speed\":" + std::to_string(vel) + "}";
    cmd_vel_pub.publish(msg);

    // velocity_command = velocity;
    // steer_command = steering_angle;
    // EgoCar::send_speed_and_steer(velocity, steering_angle);
}
void Utility::lane_follow() {
    steer_command = get_steering_angle();
    publish_cmd_vel(steer_command, 0.175);
}
void Utility::idle() {
    steer_command = 0.0;
    velocity_command = 0.0;
    publish_cmd_vel(steer_command, velocity_command);
}
double Utility::get_steering_angle(double offset) {
    ros::Time now = ros::Time::now();
    double dt = 0.1;
    if (timerpid) { 
        dt = (now - timerpid.value()).toSec(); 
        timerpid = now;
    } 
    double error;
    {
        // std::lock_guard<std::mutex> lock(general_mutex); // lock because using center
        error = center - image_center + offset;
    }
    double d_error = (error - last) / dt;
    last = error;
    double steering_angle = (p * error + d * d_error) * 180 / M_PI;
    if (steering_angle > HARD_MAX_STEERING) steering_angle = HARD_MAX_STEERING;
    if (steering_angle < -HARD_MAX_STEERING) steering_angle = -HARD_MAX_STEERING;
    return steering_angle;
}
void Utility::publish_static_transforms() {
    std::vector<geometry_msgs::TransformStamped> static_transforms;

    geometry_msgs::TransformStamped t_camera = add_static_link(0, 0, 0.2, 0, 0, 0, "chassis", "camera");
    static_transforms.push_back(t_camera);

    geometry_msgs::TransformStamped t_laser = add_static_link(0, 0, 0.2, 0, 0, 0, "chassis", "laser");
    static_transforms.push_back(t_laser);

    geometry_msgs::TransformStamped t_imu0 = add_static_link(0, 0, 0, 0, 0, 0, "chassis", "imu0");
    static_transforms.push_back(t_imu0);

    geometry_msgs::TransformStamped t_imu_cam = add_static_link(REALSENSE_TF[0], REALSENSE_TF[1], REALSENSE_TF[2], 0, REALSENSE_TF[4], 0, "chassis", "realsense");
    static_transforms.push_back(t_imu_cam);

    geometry_msgs::TransformStamped t_imu_cam2 = add_static_link(REALSENSE_TF[0], REALSENSE_TF[1], REALSENSE_TF[2], 0, REALSENSE_TF[4], 0, "chassis", "camera_imu_optical_frame");
    static_transforms.push_back(t_imu_cam2);

    static_broadcaster.sendTransform(static_transforms);
}
geometry_msgs::TransformStamped Utility::add_static_link(double x, double y, double z, double roll, double pitch, double yaw, std::string parent, std::string child) {
    geometry_msgs::TransformStamped t;
    t.header.stamp = ros::Time::now();
    t.header.frame_id = parent;
    t.child_frame_id = child;
    t.transform.translation.x = x;
    t.transform.translation.y = y;
    t.transform.translation.z = z;
    tf2::Quaternion qtn;
    qtn.setRPY(roll, pitch, yaw);
    t.transform.rotation.x = qtn.x();
    t.transform.rotation.y = qtn.y();
    t.transform.rotation.z = qtn.z();
    t.transform.rotation.w = qtn.w();
    return t;
} 
void Utility::callTriggerService() {
    std_srvs::Trigger srv;
    try {
        if (triggerServiceClient.call(srv)) {
            debug("trigger service response: " + srv.response.message, 2);
        } else {
            debug("ERROR: Failed to call service trigger_service", 1);
        }
    } catch (const std::exception& e) {
        debug("ERROR: Failed to call service trigger_service" + std::string(e.what()), 1);
    }
}
std::array<double, 3> Utility::get_real_states() const {
    return {gps_x, gps_y, Sensing::yaw};
}
void Utility::spin() {
    debug("Utility node spinning at a rate of " + std::to_string(rateVal), 2);
    while (ros::ok()) {
        ros::spinOnce();
        rate->sleep();
    }
}
