#pragma once

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <boost/asio.hpp>
#include <memory>
#include <mutex>
#include <vector>
#include <thread>
#include <atomic>
#include "Tunable.h"
#include "utils/helper.h"
#include "utils/encoder.h"
#include <chrono>
#include "utils/constants.h"
#include "BicycleKalmanFilter.h"
#include "Tracking.h"

using namespace VehicleConstants;

namespace EgoCar {

inline double roll = 0.0;
inline double pitch = 0.0;
inline double yaw = 0.0;
inline double speed = 0.0;
inline double x0 = 0.0;
inline double y0 = 0.0;
inline double height = 0.0;
inline double odomX = 0.0;
inline double odomY = 0.0;
inline double odomYaw = 0.0;
inline double dx = 0.0;
inline double dy = 0.0;
inline double dyaw = 0.0;
inline double dheight = 0.0;
inline double wheelbase = VehicleConstants::WHEELBASE;
inline double l_r = VehicleConstants::L_R_REAL;

inline double steer_command = 0.0;
inline double velocity_command = 0.0;
inline std::chrono::steady_clock::time_point last_imu_time;
inline std::chrono::steady_clock::time_point last_speed_time;

inline std::unique_ptr<BicycleKalmanFilter> bkf;
inline std::chrono::steady_clock::time_point last_predict_time;

inline std::mutex state_mutex;
inline std::atomic<bool> first_imu_received{false};
inline std::atomic<bool> first_speed_received{false};

inline boost::asio::io_service io_service;
inline std::unique_ptr<boost::asio::serial_port> serial_port;

// ROS interfaces
inline ros::Subscriber imu_subscriber;
inline ros::Subscriber encoder_subscriber;
inline ros::Publisher command_publisher;
inline std_msgs::String command_msg;
inline ros::Timer imu_timer;

inline std::atomic<bool> sensors_running{false};
inline std::atomic<bool> prediction_running{false};
inline std::unique_ptr<std::thread> sensors_thread;
inline std::unique_ptr<std::thread> prediction_thread;

inline double raw_roll()   {
    std::lock_guard<std::mutex> lock(state_mutex);
    return roll;
}
inline double raw_pitch()  {
    std::lock_guard<std::mutex> lock(state_mutex);
    return pitch;
}
inline double raw_yaw()    {
    std::lock_guard<std::mutex> lock(state_mutex);
    return yaw;
}
inline double raw_speed()  {
    std::lock_guard<std::mutex> lock(state_mutex);
    return speed;
}
inline double get_x() {
    std::lock_guard<std::mutex> lock(state_mutex);
    return bkf ? bkf->getX() : odomX + x0;
}
inline double get_y() {
    std::lock_guard<std::mutex> lock(state_mutex);
    return bkf ? bkf->getY() : odomY + y0;
}
inline double get_yaw() {
    std::lock_guard<std::mutex> lock(state_mutex);
    return bkf ? bkf->getYaw() : yaw;
}
inline void update_states(Eigen::Vector3d& o_state) {
    std::lock_guard<std::mutex> lock(state_mutex);
    if (bkf) {
        auto bkf_state = bkf->getState();
        o_state = bkf_state;
    } else {
        o_state[0] = odomX + x0;
        o_state[1] = odomY + y0;
        o_state[2] = yaw;
    }
}
inline double get_speed_command() {
    std::lock_guard<std::mutex> lock(state_mutex);
    return velocity_command;
}
inline double get_steer_command() {
    std::lock_guard<std::mutex> lock(state_mutex);
    return steer_command;
}
inline void update_orientation(double r, double p, double yy) {
    std::lock_guard<std::mutex> lock(state_mutex);
    roll = r;
    pitch = p;
    yaw = yy;
    last_imu_time = std::chrono::steady_clock::now();
}

inline void imu_callback(const sensor_msgs::Imu::ConstPtr& msg) {
    tf2::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
    double r, p, yy;
    tf2::Matrix3x3(q).getRPY(r, p, yy);
    update_orientation(r, p, yy);
    double yaw = raw_yaw();
    if (bkf) bkf->updateYaw(yaw, 0.312262*M_PI/180.0);
    if (!first_imu_received) {
        first_imu_received = true;
        ROS_INFO("EgoCar(): IMU initialized");
    }
}

inline void encoder_callback(const utils::encoder& msg) {
    {
        std::lock_guard<std::mutex> lock(state_mutex);
        speed = msg.speed;
        // if (bkf) bkf->updateSpeed(speed, 0.01047);
        last_speed_time = std::chrono::steady_clock::now();
    }
    if (!first_speed_received) {
        first_speed_received = true;
        ROS_INFO("EgoCar(): Speed initialized");
    }
}

inline void serial_update() {
    static char data_buf[256];
    static std::string buffer;
    size_t len = serial_port->read_some(
        boost::asio::mutable_buffer(data_buf, sizeof(data_buf)));
    buffer.append(data_buf, len);

    auto marker = buffer.find("@7");
    if (marker == std::string::npos) return;
    auto nl = buffer.find('\n');
    if (nl == std::string::npos) return;

    std::string line = buffer.substr(0, nl);
    buffer.erase(0, nl + 1);
    if (line.find("@7") == std::string::npos) return;

    auto sep = line.find(':');
    if (sep == std::string::npos || sep + 1 >= line.size()) return;

    std::vector<std::string> fields;
    size_t start = sep + 1;
    while (fields.size() < 9) {
        auto pos = line.find(';', start);
        if (pos == std::string::npos) return;
        fields.push_back(line.substr(start, pos - start));
        start = pos + 1;
    }

    try {
        double rd = std::stod(fields[0]);
        double pd = std::stod(fields[1]);
        double yd = std::stod(fields[2]);
        double ax = std::stod(fields[3]);
        double ay = std::stod(fields[4]);
        double az = std::stod(fields[5]);
        double gx = std::stod(fields[6]);
        double gy = std::stod(fields[7]);
        double gz = std::stod(fields[8]);

        double r = rd * M_PI / 180.0;
        double p = pd * M_PI / 180.0;
        double y = -yd * M_PI / 180.0;
        y = helper::yaw_mod(y);
        update_orientation(r, p, y);
        double cur_yaw = raw_yaw();
        if (bkf) bkf->updateYaw(cur_yaw, 0.312262*M_PI/180.0);
        {
            std::lock_guard<std::mutex> lock(state_mutex);
            last_speed_time = std::chrono::steady_clock::now();
        }
        if (!first_imu_received) {
            first_imu_received = true;
            ROS_INFO("First IMU data received (serial)");
        }
        if (!first_speed_received) {
            first_speed_received = true;
            ROS_INFO("First speed data received (serial)");
        }
    } catch (const std::exception& e) {
        ROS_ERROR("Failed to parse IMU data: %s", e.what());
    }
}

inline int rk4() {
    double speed = get_speed_command();
    double steering_angle = get_steer_command();
    auto now = std::chrono::steady_clock::now();
    double dt = std::chrono::duration<double>(now - last_predict_time).count();
    if (dt < 0) {
        return 0;
    }
    last_predict_time = now;
    if (std::abs(pitch) <3 * M_PI / 180) {
        pitch = 0;
    }
    dheight = speed * dt * sin(pitch);
    double v_eff = speed * cos(pitch);

    double delta_rad = -steering_angle * M_PI / 180.0;
    double beta = 0;
    if (Tunable::use_beta) beta = atan((l_r / wheelbase) * tan(delta_rad));

    double magnitude = v_eff * dt;
    double yaw_rate = dt * magnitude * tan(delta_rad) / wheelbase * cos(beta);

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
    return 1;
}
inline void odometry() {
    {
        rk4();
        {
            std::lock_guard<std::mutex> lock(state_mutex);
            odomX += dx;
            odomY += dy;
            height -= dheight;
        }
    }
}

inline int recalibrate_states(double x_offset, double y_offset) {
    if(bkf) {
        double x = bkf->getX();
        double y = bkf->getY();
        double new_x = x + x_offset;
        double new_y = y + y_offset;
        bkf->updatePosition(new_x, new_y, 0.001, 0.001);
    } else {
        x0 += x_offset;
        y0 += y_offset;
    }
    return 1;
}
inline int set_states(double x, double y) {
    if (!bkf) {
        std::lock_guard<std::mutex> lock(state_mutex);
        x0 = x;
        y0 = y;
        odomX = 0;
        odomY = 0;
        return 0;
    } else {
        std::lock_guard<std::mutex> lock(state_mutex);
        bkf->updatePosition(x, y, 0.001, 0.001);
        return 1;
    }
}

inline void read_sensors() {
    if (!Tunable::real) {
        ros::spinOnce();
    } else {
        serial_update();
        io_service.poll();
    }
}

inline void sensors_thread_loop() {
    ros::Rate rate(50.0);
    last_predict_time = std::chrono::steady_clock::now();
    while (sensors_running) {
        read_sensors();
        // {
        //     std::lock_guard<std::mutex> lock(state_mutex);
        //     ROS_INFO("EgoCar(): roll: %.2f, pitch: %.2f, yaw: %.2f, speed: %.2f",
        //              roll * 180.0 / M_PI, pitch * 180.0 / M_PI,
        //              yaw * 180.0 / M_PI, speed);
        // }
        rate.sleep();
    }
}

inline void start_sensors_thread() {
    if (sensors_running) return;
    sensors_running = true;
    sensors_thread = std::make_unique<std::thread>(
        []{ sensors_thread_loop(); }
    );
}

inline void stop_sensors_thread() {
    if (!sensors_running) return;
    sensors_running = false;
    if (sensors_thread && sensors_thread->joinable()) {
        sensors_thread->join();
    }
    sensors_thread.reset();
}

inline void prediction_thread_loop(double hz) {
    ros::Rate rate(hz);
    while (prediction_running) {
        if (!(first_imu_received && first_speed_received)) {
            rate.sleep();
            continue;
        }
        if (bkf) {
            auto now = std::chrono::steady_clock::now();
            double dt = std::chrono::duration<double>(now - last_predict_time).count();
            if (dt > 0) {
                double curr_speed = get_speed_command();
                double cmd_steer = get_steer_command();
                bkf->predict(cmd_steer, curr_speed, dt);
                last_predict_time = now;
            }
        } else {
            odometry();
        }
        rate.sleep();
    }
}

inline void start_prediction_thread() {
    if (prediction_running) return;
    prediction_running = true;
    last_predict_time = std::chrono::steady_clock::now();
    prediction_thread = std::make_unique<std::thread>(
        []{ prediction_thread_loop(50); }
    );
}

inline void stop_prediction_thread() {
    if (!prediction_running) return;
    prediction_running = false;
    if (prediction_thread && prediction_thread->joinable()) {
        prediction_thread->join();
    }
    prediction_thread.reset();
}

inline void initialize(ros::NodeHandle& nh) {
    if (!Tunable::real) {
        imu_subscriber     = nh.subscribe("car1/imu", 1, imu_callback);
        encoder_subscriber = nh.subscribe("car1/encoder", 1, encoder_callback);
    } else {
        serial_port.reset();
        io_service.reset();
        try {
            serial_port = std::make_unique<boost::asio::serial_port>(
                io_service, "/dev/ttyACM0"
            );
            serial_port->set_option(
                boost::asio::serial_port_base::baud_rate(115200)
            );
            ROS_INFO("Serial port opened");
        } catch (const boost::system::system_error& e) {
            ROS_ERROR("Serial open error: %s", e.what());
        }
    }
    command_publisher = nh.advertise<std_msgs::String>("/car1/command", 8);
}

inline void initialize_prediction(double x0_, double y0_) {
    while (!first_imu_received) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    x0 = x0_;
    y0 = y0_;
    if (Tunable::ekf) {
        bkf = std::make_unique<BicycleKalmanFilter>(WHEELBASE, L_R_REAL, Tunable::use_beta);
        double yaw0 = raw_yaw();
        std::cout << "EgoCar::initialize_bkf(): initializing EKF: x0: " << x0 << ", y0: " << y0 << ", yaw0: " << yaw0 << std::endl;
        double std_x = 0.01; // x
        double std_y = 0.01; // y
        double std_yaw = 0.05 * M_PI/180; // yaw
        bkf->init(x0, y0, yaw0, std_x, std_y, std_yaw);
    } else {
        bkf = nullptr;
    }
}

inline bool send_speed_and_steer(double velocity, double steer) {
    // speed: m/s, steer: degrees
    if (steer > HARD_MAX_STEERING) steer = HARD_MAX_STEERING;
    if (steer < -HARD_MAX_STEERING) steer = -HARD_MAX_STEERING;
    if (std::isnan(steer)) {
        std::cerr << "Error: Steering angle is NaN!" << std::endl;
        steer = 0.0;
        return false;
    }
    if (std::isnan(velocity)) {
        std::cerr << "Error: Velocity is NaN!" << std::endl;
        velocity = 0.0;
        return false;
    }
    {
        std::lock_guard<std::mutex> lock(state_mutex);
        velocity_command = velocity;
        steer_command = steer * M_PI / 180.0;
    }
    if (true) {        
        static bool first = true;
        static bool use_pid = false;
        if (serial_port) {
            if (first && use_pid) {
                first = false;
                std::stringstream ss;
                char buf[100];
                snprintf(buf, sizeof(buf), "%.4f:%.4f:%.4f:%.4f;;\r\n",
                        1.0f, 1.0f, 0.0f, 0.0f);
                ss << "#12:" << buf;
                boost::asio::write(*serial_port, boost::asio::buffer(ss.str()));
            }
            if (steer > 3.0f) steer += 4.0f;
            std::stringstream ss;
            char buf2[100];
            snprintf(buf2, sizeof(buf2), "%.2f:%.2f;;\r\n", velocity*100.0f, steer);
            ss << (use_pid?"#13:":"#11:") << buf2;
            boost::asio::write(*serial_port, boost::asio::buffer(ss.str()));
        }
    }
    if (true) {
        float velocity_f = static_cast<float>(velocity);
        float steer_f = static_cast<float>(steer);
        command_msg.data = "{\"action\":\"2\",\"steerAngle\":" + std::to_string(steer_f) + "}";
        command_publisher.publish(command_msg);
        command_msg.data = "{\"action\":\"1\",\"speed\":" + std::to_string(velocity_f) + "}";
        command_publisher.publish(command_msg);
    }
    return true;
}

} // namespace EgoCar

