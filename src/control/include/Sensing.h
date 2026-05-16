#pragma once

#include <boost/asio.hpp>
#include <array>
#include <atomic>
#include <chrono>
#include <cstring>
#include <cmath>
#include <iostream>
#include <memory>
#include <mutex>
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
inline std::atomic<double> estimated_yaw{0.0};  // [rad]
inline std::atomic<double> pitch{0.0};          // [rad]
inline std::atomic<double> gyro_z{0.0};         // [rad s^-1], raw BNO frame
inline std::atomic<double> gyro_z_yaw_rate{0.0}; // [rad s^-1], yaw convention used by Sensing::yaw
inline std::atomic<double> gyro_z_bias{0.0};    // [rad s^-1], yaw convention used by Sensing::yaw
inline std::atomic<bool> gyro_z_available{false};
inline std::atomic<bool> gyro_z_sign_locked{false};
inline std::atomic<double>    sys_calib{0.0};     
inline std::atomic<double>    gyro_calib{1.0};     
inline std::atomic<double>    mag_calib{2.0};     
inline std::atomic<double>    accel_calib{3.0};

inline std::atomic<double> yaw_offset{0.0};

// ---------- yaw estimator state ----------
inline constexpr double DEFAULT_GYRO_Z_SIGN = -1.0;
inline constexpr double MAX_GYRO_DT = 0.20;
inline constexpr double MAX_ABS_GYRO_Z = 8.0;
inline constexpr double MAX_ABS_GYRO_BIAS = 0.35;
inline constexpr double STATIONARY_SPEED_THRESH = 0.015;
inline constexpr double STATIONARY_GYRO_THRESH = 0.12;
inline constexpr double GYRO_BIAS_ALPHA = 0.01;
inline constexpr int GYRO_SIGN_LOCK_SCORE = 8;
inline constexpr double MIN_SIGN_OBS_DELTA = 0.015;

inline std::mutex yaw_estimator_mutex;
inline bool yaw_estimator_initialized = false;
inline bool gyro_time_initialized = false;
inline bool imu_sample_time_initialized = false;
inline bool raw_yaw_history_initialized = false;
inline std::chrono::steady_clock::time_point last_gyro_time;
inline double last_imu_sample_time = 0.0;
inline double yaw_estimate_state = 0.0;
inline double gyro_bias_state = 0.0;
inline double last_raw_yaw_for_sign = 0.0;
inline int gyro_sign_score = 0;
inline double gyro_sign = DEFAULT_GYRO_Z_SIGN;

// ---------- serial machinery ----------
inline boost::asio::io_service io;
inline std::shared_ptr<boost::asio::serial_port> serial;

inline constexpr std::size_t RX_CAP = 512;
inline std::array<char, RX_CAP> rxBuf{};
inline std::size_t rxLen = 0;

// Very small fast_atof version (same as in approach 1)
inline bool fast_atof(const char* s, const char* e, double& out)
{
    if (s >= e) return false;
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

inline void publish_yaw_locked(double new_yaw, double latest_raw_yaw)
{
    yaw_estimate_state = yaw_mod(new_yaw);
    yaw.store(yaw_estimate_state, std::memory_order_relaxed);
    estimated_yaw.store(yaw_estimate_state, std::memory_order_relaxed);
    yaw_offset.store(yaw_mod(yaw_estimate_state - latest_raw_yaw), std::memory_order_relaxed);
}

inline void reset_yaw_estimator_state_locked(double new_yaw, double latest_raw_yaw)
{
    yaw_estimator_initialized = true;
    publish_yaw_locked(new_yaw, latest_raw_yaw);
}

inline void update_gyro_sign_locked(double latest_raw_yaw, double raw_gyro_z, double dt)
{
    if (!raw_yaw_history_initialized) {
        last_raw_yaw_for_sign = latest_raw_yaw;
        raw_yaw_history_initialized = true;
        return;
    }

    const double raw_yaw_delta = yaw_mod(latest_raw_yaw - last_raw_yaw_for_sign);
    const double gyro_delta = raw_gyro_z * dt;
    last_raw_yaw_for_sign = latest_raw_yaw;

    if (std::abs(raw_yaw_delta) < MIN_SIGN_OBS_DELTA ||
        std::abs(gyro_delta) < MIN_SIGN_OBS_DELTA) {
        return;
    }

    gyro_sign_score += (raw_yaw_delta * gyro_delta >= 0.0) ? 1 : -1;
    if (gyro_sign_score >= GYRO_SIGN_LOCK_SCORE) {
        gyro_sign = 1.0;
        gyro_z_sign_locked.store(true, std::memory_order_relaxed);
    } else if (gyro_sign_score <= -GYRO_SIGN_LOCK_SCORE) {
        gyro_sign = -1.0;
        gyro_z_sign_locked.store(true, std::memory_order_relaxed);
    }
}

inline void update_yaw_from_imu(double pitch_rad,
                                double latest_raw_yaw,
                                double sys,
                                double gyro,
                                double mag,
                                double accel,
                                bool has_raw_gyro_z,
                                double raw_gyro_z,
                                bool has_sample_time,
                                double sample_time_s)
{
    pitch.store(pitch_rad, std::memory_order_relaxed);
    raw_yaw.store(latest_raw_yaw, std::memory_order_relaxed);
    sys_calib  .store(sys  , std::memory_order_relaxed);
    gyro_calib .store(gyro , std::memory_order_relaxed);
    mag_calib  .store(mag  , std::memory_order_relaxed);
    accel_calib.store(accel, std::memory_order_relaxed);

    const auto now = std::chrono::steady_clock::now();
    std::lock_guard<std::mutex> lock(yaw_estimator_mutex);

    if (!yaw_estimator_initialized) {
        reset_yaw_estimator_state_locked(yaw_mod(latest_raw_yaw + yaw_offset.load(std::memory_order_relaxed)),
                                         latest_raw_yaw);
        last_gyro_time = now;
        gyro_time_initialized = has_raw_gyro_z;
        if (has_sample_time && std::isfinite(sample_time_s)) {
            last_imu_sample_time = sample_time_s;
            imu_sample_time_initialized = true;
        }
        last_raw_yaw_for_sign = latest_raw_yaw;
        raw_yaw_history_initialized = true;
        return;
    }

    if (has_raw_gyro_z && std::isfinite(raw_gyro_z)) {
        gyro_z.store(raw_gyro_z, std::memory_order_relaxed);
        gyro_z_available.store(true, std::memory_order_relaxed);

        if (!gyro_time_initialized) {
            last_gyro_time = now;
            gyro_time_initialized = true;
            if (has_sample_time && std::isfinite(sample_time_s)) {
                last_imu_sample_time = sample_time_s;
                imu_sample_time_initialized = true;
            }
            publish_yaw_locked(yaw_estimate_state, latest_raw_yaw);
            return;
        }

        double dt = 0.0;
        if (has_sample_time && std::isfinite(sample_time_s)) {
            if (imu_sample_time_initialized) {
                dt = sample_time_s - last_imu_sample_time;
            }
            last_imu_sample_time = sample_time_s;
            imu_sample_time_initialized = true;
        } else {
            dt = std::chrono::duration<double>(now - last_gyro_time).count();
        }
        last_gyro_time = now;

        if (dt > 0.0 && dt <= MAX_GYRO_DT && std::abs(raw_gyro_z) <= MAX_ABS_GYRO_Z) {
            update_gyro_sign_locked(latest_raw_yaw, raw_gyro_z, dt);

            const double signed_gyro_z = gyro_sign * raw_gyro_z;
            gyro_z_yaw_rate.store(signed_gyro_z, std::memory_order_relaxed);

            if (std::abs(encoder_speed.load(std::memory_order_relaxed)) < STATIONARY_SPEED_THRESH &&
                std::abs(signed_gyro_z) < STATIONARY_GYRO_THRESH &&
                gyro >= 2.0) {
                gyro_bias_state = (1.0 - GYRO_BIAS_ALPHA) * gyro_bias_state +
                                  GYRO_BIAS_ALPHA * signed_gyro_z;
                if (gyro_bias_state > MAX_ABS_GYRO_BIAS) gyro_bias_state = MAX_ABS_GYRO_BIAS;
                if (gyro_bias_state < -MAX_ABS_GYRO_BIAS) gyro_bias_state = -MAX_ABS_GYRO_BIAS;
                gyro_z_bias.store(gyro_bias_state, std::memory_order_relaxed);
            }

            publish_yaw_locked(yaw_estimate_state + (signed_gyro_z - gyro_bias_state) * dt,
                               latest_raw_yaw);
            return;
        }

        gyro_z_yaw_rate.store(0.0, std::memory_order_relaxed);
        publish_yaw_locked(yaw_estimate_state, latest_raw_yaw);
        return;
    }

    gyro_z_available.store(false, std::memory_order_relaxed);
    gyro_z_yaw_rate.store(0.0, std::memory_order_relaxed);
    if (gyro_time_initialized) {
        publish_yaw_locked(yaw_estimate_state, latest_raw_yaw);
        return;
    }
    publish_yaw_locked(yaw_mod(latest_raw_yaw + yaw_offset.load(std::memory_order_relaxed)),
                       latest_raw_yaw);
}

inline bool apply_yaw_measurement(double measured_yaw, double max_residual, double gain)
{
    if (!std::isfinite(measured_yaw) || !std::isfinite(max_residual) || !std::isfinite(gain)) {
        return false;
    }
    if (gain < 0.0) gain = 0.0;
    if (gain > 1.0) gain = 1.0;

    measured_yaw = yaw_mod(measured_yaw);
    std::lock_guard<std::mutex> lock(yaw_estimator_mutex);

    const double latest_raw_yaw = raw_yaw.load(std::memory_order_relaxed);
    if (!yaw_estimator_initialized) {
        reset_yaw_estimator_state_locked(measured_yaw, latest_raw_yaw);
        return true;
    }

    const double residual = yaw_mod(measured_yaw - yaw_estimate_state);
    if (std::abs(residual) > max_residual) {
        return false;
    }

    publish_yaw_locked(yaw_estimate_state + gain * residual, latest_raw_yaw);
    return true;
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
            if (cur >= end) return false;

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
        double raw_gyro_z = 0.0;
        double imu_time_us = 0.0;
        bool has_raw_gyro_z = false;
        bool has_imu_time = false;

        if (!next_field(pitch_deg)) return;
        if (!next_field(yaw_deg))   return;
        if (!next_field(sys))       return;
        if (!next_field(gyro))      return;
        if (!next_field(mag))       return;
        if (!next_field(accel))     return;
        if (cur < end) {
            has_raw_gyro_z = next_field(raw_gyro_z);
        }
        if (has_raw_gyro_z && cur < end) {
            has_imu_time = next_field(imu_time_us);
        }

        /* ------------ store the results ------------------------------- */
        double tmp_yaw = -yaw_deg * M_PI / 180.0;
        update_yaw_from_imu(pitch_deg * M_PI / 180.0,
                            tmp_yaw,
                            sys,
                            gyro,
                            mag,
                            accel,
                            has_raw_gyro_z,
                            raw_gyro_z,
                            has_imu_time,
                            imu_time_us * 1e-6);
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
    update_yaw_from_imu(p_,
                        y_,
                        sys_calib.load(std::memory_order_relaxed),
                        gyro_calib.load(std::memory_order_relaxed),
                        mag_calib.load(std::memory_order_relaxed),
                        accel_calib.load(std::memory_order_relaxed),
                        false,
                        0.0,
                        false,
                        0.0);
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
        const double want  = new_yaw;
        apply_yaw_measurement(want, 2.0 * M_PI, 1.0);
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
