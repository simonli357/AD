#pragma once

#include <boost/asio.hpp>
#include <array>
#include <atomic>
#include <cstring>
#include <iostream>
#include <memory>
#include <ostream>
#include <thread>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "Tunable.h"
#include "utils/encoder.h"

namespace Sensing {

inline std::atomic<double> encoder_speed{0.0};  // [m s⁻¹]
inline std::atomic<double> yaw{0.0};            // [rad]
inline std::atomic<double> raw_yaw{0.0};        // [rad]
inline std::atomic<double> pitch{0.0};          // [rad]
inline std::atomic<double>    sys_calib{0.0};     
inline std::atomic<double>    gyro_calib{1.0};     
inline std::atomic<double>    mag_calib{2.0};     
inline std::atomic<double>    accel_calib{3.0};

inline std::atomic<double> yaw_offset{0.0};

// ---------- serial machinery ----------
inline boost::asio::io_service io;
inline std::shared_ptr<boost::asio::serial_port> serial;

inline constexpr std::size_t RX_CAP = 512;
inline std::array<char, RX_CAP> rxBuf{};
inline std::size_t rxLen = 0;

// Very small fast_atof version (same as in approach 1)
inline bool fast_atof(const char* s, const char* e, double& out)
{
    bool neg = false;
    if (*s == '-') { neg = true; ++s; }
    double i = 0;
    while (s < e && *s >= '0' && *s <= '9')
        i = i * 10 + (*s++ - '0');
    double f = 0, base = 1;
    if (s < e && *s == '.') {
        ++s;
        while (s < e && *s >= '0' && *s <= '9') {
            f = f * 10 + (*s++ - '0');
            base *= 10;
        }
    }
    out = (i + f / base) * (neg ? -1.0 : 1.0);
    return s == e;
}

// Helper to wrap yaw to [‑π, π)
inline double yaw_mod(double v) {
    while (v < -M_PI) v += 2 * M_PI;
    while (v >=  M_PI) v -= 2 * M_PI;
    return v;
}

inline void start_async_read();
inline void scan_frames();

inline void parse_and_publish(char id, const char* p, std::size_t len)
{
    if (id == '5') {            // encoder frame
        const char* semi = static_cast<const char*>(memchr(p, ';', len));
        const char* end  = semi ? semi : p + len;
        double speed_cm;
        if (!fast_atof(p, end, speed_cm)) return;
        encoder_speed.store(0.01 * speed_cm, std::memory_order_relaxed);
        // std::cout << "Sensing: speed: " << speed_cm << std::endl;
        return;
    }

    if (id == '7') {                    // ----- IMU frame -----------------------
        const char* cur  = p;           // start of current field
        const char* end  = p + len;     // end of payload

        auto next_field = [&](double& out)->bool {
        // look for the next semicolon in the remaining slice
        const char* semi = static_cast<const char*>(
            memchr(cur, ';', end - cur));

        const char* stop = semi ? semi : end;   // take ; or end-of-payload
        /* trim leading spaces */
        const char* s = cur;
        while (s < stop && *s == ' ') ++s;

        if (!fast_atof(s, stop, out)) return false;

        /* advance pointer unless we already are at the end */
        if (semi) {
            cur = semi + 1;
            while (cur < end && *cur == ' ') ++cur;
        } else {
            cur = end;          // no more data
        }
        return true;
    };

        double pitch_deg, yaw_deg, sys, gyro, mag, accel;

        if (!next_field(pitch_deg)) return;
        if (!next_field(yaw_deg))   return;
        if (!next_field(sys))       return;
        if (!next_field(gyro))      return;
        if (!next_field(mag))       return;
        if (!next_field(accel))     return;

        /* ------------ store the results ------------------------------- */
        pitch.store(pitch_deg * M_PI / 180.0, std::memory_order_relaxed);

        double tmp_yaw = -yaw_deg * M_PI / 180.0;
        yaw.store (yaw_mod(tmp_yaw + yaw_offset.load()), std::memory_order_relaxed);
        raw_yaw.store(tmp_yaw, std::memory_order_relaxed);

        sys_calib  .store(sys  , std::memory_order_relaxed);
        gyro_calib .store(gyro , std::memory_order_relaxed);
        mag_calib  .store(mag  , std::memory_order_relaxed);
        accel_calib.store(accel, std::memory_order_relaxed);
    }
}

inline void handle_rx(const boost::system::error_code& ec, std::size_t n)
{
    if (ec) {
        ROS_ERROR_STREAM("serial error: " << ec.message());
        return;
    }
    rxLen += n;
    scan_frames();
    start_async_read();
}

inline void start_async_read()
{
    serial->async_read_some(
        boost::asio::buffer(rxBuf.data() + rxLen, RX_CAP - rxLen),
        [](const boost::system::error_code& ec, std::size_t n) {
            handle_rx(ec, n);
        });
}

inline void scan_frames()
{
    const char* start = nullptr;
    char        id    = 0;

    for (std::size_t i = 0; i + 3 < rxLen; ++i) {
        if (!start) {
            if (rxBuf[i] == '@' && (rxBuf[i + 1] == '5' || rxBuf[i + 1] == '7') && rxBuf[i + 2] == ':') {
                id = rxBuf[i + 1];
                start = rxBuf.data() + i + 3;
            }
        }
        else if (rxBuf[i] == ';' && rxBuf[i + 1] == ';' && rxBuf[i + 2] == '\r' && rxBuf[i + 3] == '\n') {
            std::size_t len = (rxBuf.data() + i) - start;
            parse_and_publish(id, start, len);

            std::size_t used = i + 4;
            std::memmove(rxBuf.data(), rxBuf.data() + used, rxLen - used);
            rxLen -= used;
            start = nullptr;
            i = static_cast<std::size_t>(-1);
        }
    }
}

// ---------- ROS fallback machinery ----------
inline ros::Subscriber imu_sub;
inline ros::Subscriber enc_sub;

inline void imuCallback(const sensor_msgs::ImuConstPtr& msg)
{
    tf2::Quaternion q;
    tf2::fromMsg(msg->orientation, q);
    double r, p_, y_;
    tf2::Matrix3x3(q).getRPY(r, p_, y_);
    pitch.store(p_, std::memory_order_relaxed);
    yaw.store(yaw_mod(y_ + yaw_offset.load()), std::memory_order_relaxed);
    raw_yaw.store(y_, std::memory_order_relaxed);
    // std::cout << "Sensing: rawyaw: " << raw_yaw << " yaw_offset: " << yaw_offset
    //           << " yaw: " << yaw << std::endl;
    // sys_calib.store(sys_calib.load() + 1, std::memory_order_relaxed);
}

inline void encoderCallback(const utils::encoder::ConstPtr& msg)
{
    encoder_speed.store(msg->speed, std::memory_order_relaxed);
}

inline void reset_yaw(double new_yaw) {
    while(new_yaw < -M_PI) new_yaw += 2 * M_PI;
    while(new_yaw >= M_PI) new_yaw -= 2 * M_PI;
    // if (Tunable::real) {
    if (false) {
        std::stringstream strs;
        char buff[100];
        snprintf(buff, sizeof(buff), "%.3f;;\r\n", 
                 new_yaw * 180.0 / M_PI);
        std::string number = "3";
        strs << "#" << number << ":" << buff;
        boost::asio::write(*serial, boost::asio::buffer(strs.str()));
        std::cout << "Sensing: reset yaw to " << new_yaw * 180.0 / M_PI
                  << " degrees" << std::endl;
    } else {
        const double raw   = raw_yaw.load(std::memory_order_relaxed);
        const double want  = new_yaw;

        yaw_offset.store(yaw_mod(want - raw), std::memory_order_relaxed);
    
        yaw.store(yaw_mod(want), std::memory_order_relaxed);
        // std::cout << "raw: " << raw << " want: " << want
        //           << " yaw_offset: " << yaw_offset.load() << " yaw: " << yaw.load() << std::endl;
    }
}
inline void reset_yaw_to_direction(int direction)
{
    static constexpr double desired_heading[4] =
        { 0.0,  M_PI/2,  M_PI, -M_PI/2 };           // E, N, W, S
    if (direction < 0 || direction > 3) return;
    double new_yaw = desired_heading[direction];
    reset_yaw(new_yaw);
    // if (Tunable::real) {
    //     std::stringstream strs;
    //     char buff[100];
    //     snprintf(buff, sizeof(buff), "%.3f;;\r\n", 
    //              desired_heading[direction] * 180.0 / M_PI);
    //     std::string number = "3";
    //     strs << "#" << number << ":" << buff;
    //     boost::asio::write(*serial, boost::asio::buffer(strs.str()));
    //     std::cout << "Sensing: reset yaw to " << desired_heading[direction] * 180.0 / M_PI
    //               << " degrees" << std::endl;
    // } else {
    //     const double raw   = raw_yaw.load(std::memory_order_relaxed);
    //     const double want  = desired_heading[direction];

    //     yaw_offset.store(yaw_mod(want - raw), std::memory_order_relaxed);
    
    //     yaw.store(yaw_mod(want), std::memory_order_relaxed);
    //     // std::cout << "raw: " << raw << " want: " << want
    //     //           << " yaw_offset: " << yaw_offset.load() << " yaw: " << yaw.load() << std::endl;
    // }
}

inline void initialize_sensing(ros::NodeHandle& nh)
{
    if (!Tunable::initialized) {
        std::cerr << "Sensing: Tunable not initialized" << std::endl;
        exit(1);
    }
    if (Tunable::real) {
        try {
            serial = std::make_shared<boost::asio::serial_port>(io, "/dev/ttyACM0");
            serial->set_option(boost::asio::serial_port_base::baud_rate(115200));
            start_async_read();
            std::thread([] { io.run(); }).detach();
            ROS_INFO("Sensing: serial port opened and async IO started");
        }
        catch (const boost::system::system_error& e) {
            ROS_ERROR_STREAM("Sensing: failed to open serial port: " << e.what());
        }
    } else {
        imu_sub = nh.subscribe("/car1/imu", 10, imuCallback);
        enc_sub = nh.subscribe("/car1/encoder", 10, encoderCallback);
        ROS_INFO("Sensing: operating in ROS‑topic mode (/car1/imu, /car1/encoder)");
    }
}

}
