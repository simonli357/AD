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

inline double steer_command = 0.0;
inline double velocity_command = 0.0;
inline std::chrono::steady_clock::time_point last_imu_time;
inline std::chrono::steady_clock::time_point last_speed_time;

inline std::unique_ptr<BicycleKalmanFilter> ekf;
inline std::chrono::steady_clock::time_point last_ekf_time;

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
inline std::atomic<bool> ekf_running{false};
inline std::unique_ptr<std::thread> sensors_thread;
inline std::unique_ptr<std::thread> ekf_thread;

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
    if (ekf) ekf->updateYaw(yaw, 0.312262*M_PI/180.0);
    if (!first_imu_received) {
        first_imu_received = true;
        ROS_INFO("EgoCar(): IMU initialized");
    }
}

inline void encoder_callback(const utils::encoder& msg) {
    {
        std::lock_guard<std::mutex> lock(state_mutex);
        speed = msg.speed;
        // if (ekf) ekf->updateSpeed(speed, 0.01047);
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
        if (ekf) ekf->updateYaw(cur_yaw, 0.312262*M_PI/180.0);
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

// Single-step update: process either ROS callbacks or serial I/O
inline void update_values_once() {
    if (!Tunable::real) {
        ros::spinOnce();
    } else {
        serial_update();
        io_service.poll();
    }
}

inline void sensors_thread_loop() {
    ros::Rate rate(50.0);
    while (sensors_running) {
        update_values_once();
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

inline void ekf_thread_loop(double hz) {
    ros::Rate rate(hz);
    while (ekf_running) {
        if (!(first_imu_received && first_speed_received && ekf)) {
            rate.sleep();
            continue;
        }
        auto now = std::chrono::steady_clock::now();
        double dt = std::chrono::duration<double>(now - last_ekf_time).count();
        if (dt > 0) {
            double curr_speed, cmd_steer;
            {
                std::lock_guard<std::mutex> lock(state_mutex);
                curr_speed = velocity_command;
                std::cout << "curr_speed: " << curr_speed << ", steer_command: " << steer_command << std::endl;
                cmd_steer = steer_command;
            }
            ekf->predict(cmd_steer, curr_speed, dt);
            last_ekf_time = now;
            double x = ekf->getX();
            double y = ekf->getY();
            double yaw = ekf->getYaw();
            double speed = curr_speed;
            Tracking::ego_car->update(x, y, yaw, speed, 0, steer_command * 180.0 / M_PI);
        }
        rate.sleep();
    }
}

inline void start_ekf_thread() {
    if (ekf_running) return;
    ekf_running = true;
    last_ekf_time = std::chrono::steady_clock::now();
    ekf_thread = std::make_unique<std::thread>(
        []{ ekf_thread_loop(50); }
    );
}

inline void stop_ekf_thread() {
    if (!ekf_running) return;
    ekf_running = false;
    if (ekf_thread && ekf_thread->joinable()) {
        ekf_thread->join();
    }
    ekf_thread.reset();
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

inline void initialize_ekf(double x0, double y0) {
    ekf = std::make_unique<BicycleKalmanFilter>(WHEELBASE, L_R_REAL, Tunable::use_beta);
    while (!first_imu_received) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    double yaw0 = raw_yaw();
    std::cout << "EgoCar::initialize_ekf(): initializing EKF: x0: " << x0 << ", y0: " << y0 << ", yaw0: " << yaw0 << std::endl;
    double std_x = 0.01; // x
    double std_y = 0.01; // y
    double std_yaw = 0.05 * M_PI/180; // yaw
    ekf->init(x0, y0, yaw0, std_x, std_y, std_yaw);
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

