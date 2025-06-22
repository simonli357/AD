#pragma once
#include <Eigen/Dense>
#include <cmath>

class KalmanFilter {
public:
    using Vector4d = Eigen::Vector4d;
    using Matrix4d = Eigen::Matrix4d;
    using Vector2d = Eigen::Vector2d;
    using Matrix2d = Eigen::Matrix2d;
    using Matrix42d = Eigen::Matrix<double, 4, 2>;
    using Matrix24d = Eigen::Matrix<double, 2, 4>;

    KalmanFilter(double x_init, double y_init,
             double yaw_init = 0.0, double v_init = 0.0)
    {
        x_ << x_init, y_init, yaw_init, v_init;

        P_.setZero();
        P_.diagonal() << 0.04, 0.04, 0.05, 0.04;

        // R: 10 cm 1-σ  ⇒  variance = 0.01
        R_.setZero();
        R_.diagonal().setConstant(0.01);   // use 0.0049 if 7 cm is correct

        H_.setZero();
        H_(0,0) = H_(1,1) = 1;

        Q_.setZero();          // will be filled in predict()
    }

    void predict(double dt) {
        double yaw = x_(2);
        double v = x_(3);

        x_(0) += v * dt * std::cos(yaw);  // x
        x_(1) += v * dt * std::sin(yaw);  // y
        // yaw remains constant
        // v remains constant

        Matrix4d F = Matrix4d::Identity();
        F(0, 2) = -v * dt * std::sin(yaw);  // ∂x/∂yaw
        F(0, 3) = dt * std::cos(yaw);       // ∂x/∂v
        F(1, 2) = v * dt * std::cos(yaw);   // ∂y/∂yaw
        F(1, 3) = dt * std::sin(yaw);       // ∂y/∂v

        const double sa2 = 0.01;      // (0.1 m s⁻²)²
        const double sw2 = 1.9e-03;   // (0.0436 rad s⁻¹)²
        // Build Q scaled by dt
        Q_(0,0) = 0.5*sa2*dt*dt;
        Q_(1,1) = 0.5*sa2*dt*dt;
        Q_(2,2) = sw2*dt*dt;
        Q_(3,3) = sa2;
        Q_(0,3) = Q_(3,0) = sa2*dt;

        P_ = F*P_*F.transpose() + Q_;
    }

    void update(double meas_x, double meas_y) {
        Vector2d z;
        z << meas_x, meas_y;

        Vector2d z_pred = H_ * x_;
        Vector2d y = z - z_pred;  // innovation

        Matrix2d S = H_ * P_ * H_.transpose() + R_;
        Matrix42d K = P_ * H_.transpose() * S.inverse();

        // Update state and covariance
        x_ += K * y;
        P_ = (Matrix4d::Identity() - K * H_) * P_;
    }

    const Vector4d& state() const { return x_; }
    const Matrix4d& covariance() const { return P_; }

    double x() const { return x_(0); }
    double y() const { return x_(1); }
    double yaw() const { return x_(2); }
    double speed() const { return x_(3); }

private:
    Vector4d x_;    // [x, y, yaw, v]
    Matrix4d P_;    // Covariance
    Matrix4d Q_;    // Process noise
    Matrix2d R_;    // Measurement noise
    Matrix24d H_;   // Measurement matrix
};
