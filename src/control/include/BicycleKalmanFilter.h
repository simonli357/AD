#pragma once

#include <Eigen/Dense>
#include <cmath>
#include <iostream>

class BicycleKalmanFilter {
public:
    // wheelbase: distance between axles
    // l_r: distance from rear axle to center of gravity
    // use_beta: whether to include slip angle beta
    BicycleKalmanFilter(double wheelbase, double l_r, bool use_beta)
        : wheelbase_(wheelbase), l_r_(l_r), use_beta_(use_beta) {
        // initialize state & covariance
        x_.setZero();
        P_.setIdentity();

        // process noise Q (for [x, y, yaw])
        Q_.setZero();
        Q_(0,0) = std::pow(0.0005, 2); // standard deviation of 0.01 m
        Q_(1,1) = std::pow(0.0005, 2); // standard deviation of 0.01 m
        Q_(2,2) = std::pow(1 * M_PI/180.0, 2); // 5.73 degrees

        // constant matrices
        I3_.setIdentity();
        H_pos_.setZero();
        H_pos_(0,0) = 1.0;
        H_pos_(1,1) = 1.0;

        H_yaw_.setZero();
        H_yaw_(2) = 1.0;

        R_pos_.setZero();
        R_yaw_.setZero();
    }

    // initialize state and covariance by providing standard deviations
    void init(double x0, double y0, double yaw0,
              double std_x, double std_y, double std_yaw) {
        x_ << x0, y0, normalizeAngle(yaw0);
        P_.setZero();
        P_(0,0) = std_x * std_x;
        P_(1,1) = std_y * std_y;
        P_(2,2) = std_yaw * std_yaw;
    }

    // Predict state using control inputs: steering_angle and speed
    inline void predict(double steering_angle, double speed, double dt) {
        if (dt <= 0) return;

        // slip angle beta
        const double beta = use_beta_
            ? std::atan((l_r_ / wheelbase_) * std::tan(-steering_angle))
            : 0.0;
        const double psi = x_(2) + beta;
        const double cos_psi = std::cos(psi);
        const double sin_psi = std::sin(psi);
        const double tan_delta = std::tan(-steering_angle);

        // RK4 integration for x, y, yaw
        k1_(0) = speed * cos_psi;
        k1_(1) = speed * sin_psi;
        k1_(2) = speed * tan_delta / wheelbase_ * std::cos(beta);

        const double yaw_k2 = x_(2) + 0.5 * dt * k1_(2);
        k2_(0) = speed * std::cos(yaw_k2 + beta);
        k2_(1) = speed * std::sin(yaw_k2 + beta);
        k2_(2) = speed * tan_delta / wheelbase_ * std::cos(beta);

        const double yaw_k3 = x_(2) + 0.5 * dt * k2_(2);
        k3_(0) = speed * std::cos(yaw_k3 + beta);
        k3_(1) = speed * std::sin(yaw_k3 + beta);
        k3_(2) = speed * tan_delta / wheelbase_ * std::cos(beta);

        const double yaw_k4 = x_(2) + dt * k3_(2);
        k4_(0) = speed * std::cos(yaw_k4 + beta);
        k4_(1) = speed * std::sin(yaw_k4 + beta);
        k4_(2) = speed * tan_delta / wheelbase_ * std::cos(beta);

        x_ += (dt / 6.0) * (k1_ + 2.0 * k2_ + 2.0 * k3_ + k4_);
        x_(2) = normalizeAngle(x_(2));

        printf("steer: %.3f, speed: %.3f, dt: %.3f\n", -steering_angle, speed, dt);
        printf("x: %.3f, y: %.3f, yaw: %.3f\n", x_(0), x_(1), x_(2));

        // linearize and propagate covariance
        Eigen::Matrix3d J;
        J.setZero();
        {
            const double psi_j = x_(2) + beta;
            const double cs = std::cos(psi_j);
            const double sn = std::sin(psi_j);
            J(0,2) = -speed * sn;
            J(1,2) =  speed * cs;
            // no yaw derivative wrt yaw beyond psi normalization
        }

        F_ = I3_ + J * dt;
        P_ = F_ * P_ * F_.transpose() + Q_;
    }

    // Position update (x, y)
    inline void updatePosition(double meas_x, double meas_y,
                               double std_dev_x, double std_dev_y) {
        Eigen::Vector2d z;
        z << meas_x, meas_y;
        R_pos_(0,0) = std_dev_x * std_dev_x;
        R_pos_(1,1) = std_dev_y * std_dev_y;
        ekfUpdate(H_pos_, z, R_pos_);
    }

    // Yaw update
    inline void updateYaw(double meas_yaw, double std_dev) {
        while (x_(2) - meas_yaw > M_PI)  meas_yaw += 2.0 * M_PI;
        while (x_(2) - meas_yaw < -M_PI) meas_yaw -= 2.0 * M_PI;
        Eigen::VectorXd z(1);
        z(0) = meas_yaw;
        R_yaw_(0,0) = std_dev * std_dev;
        ekfUpdate(H_yaw_, z, R_yaw_);
    }

    // getters
    inline double getX() const   { return x_(0); }
    inline double getY() const   { return x_(1); }
    inline double getYaw() const { return normalizeAngle(x_(2)); }
    inline const Eigen::Vector3d& getState() const { return x_; }
    inline const Eigen::Matrix3d& getCovariance() const { return P_; }

private:
    Eigen::Vector3d x_;          // state: [x, y, yaw]
    Eigen::Matrix3d P_;          // covariance
    Eigen::Matrix3d Q_;          // process noise

    double wheelbase_;
    double l_r_;
    bool use_beta_;

    // RK4 temporaries
    Eigen::Vector3d k1_, k2_, k3_, k4_;
    Eigen::Matrix3d F_, I3_;

    Eigen::Matrix<double,2,3> H_pos_;
    Eigen::RowVector3d        H_yaw_;
    Eigen::Matrix2d           R_pos_;
    Eigen::MatrixXd           R_yaw_{1,1};

    // normalize angle to [-pi, pi]
    static inline double normalizeAngle(double angle) {
        while (angle > M_PI)  angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

    // generic EKF update
    template<typename HType, typename ZType, typename RType>
    void ekfUpdate(const HType& H,
                   const ZType& z,
                   const RType& R) {
        // innovation
        Eigen::VectorXd y_temp = z - H * x_;
        // normalize yaw innovation if needed
        if (H.rows() == 1 && H.cols() == 3 && H(0,2) == 1.0) {
            y_temp(0) = normalizeAngle(y_temp(0));
        }
        Eigen::MatrixXd S = H * P_ * H.transpose() + R;
        Eigen::CompleteOrthogonalDecomposition<Eigen::MatrixXd> cod(S);
        if (!cod.isInvertible()) {
            std::cerr << "[BicycleKalmanFilter] Warning: S not invertible. Skipping update.\n";
            return;
        }
        Eigen::MatrixXd K = P_ * H.transpose() * cod.pseudoInverse();
        x_ += K * y_temp;
        x_(2) = normalizeAngle(x_(2));
        P_ = (I3_ - K * H) * P_ * (I3_ - K * H).transpose() + K * R * K.transpose();
    }
};
