// CEncoder.hpp
#ifndef CENCODER_HPP
#define CENCODER_HPP

#include "mbed.h"
#include <utils/task.hpp>
#include "PwmIn.h"
#include <cmath>
#include <algorithm>

struct Kalman2D {
    // State estimate
    float ang   = 0.0f;     // θ  (deg, unwrapped)
    float speed = 0.0f;     // ω  (deg/s)

    // Covariance matrix (symmetric)
    float P00 = 1.0f, P01 = 0.0f, P11 = 1.0f;

    // Tunables (process & measurement noise)
    float Q_angle = 0.01f;     // (deg²)   – how “wandery” the true angle is
    float Q_speed = 100.0f;    // (deg²/s²)– how “wandery” the true speed is
    float R_angle = 4.0f;      // (deg²)   – encoder quantisation / noise

    inline void predict(float dt)
    {
        /* x = F·x ,  with  F = [1  dt; 0 1] */
        ang   += dt * speed;

        /* P = F·P·Fᵀ + Q  (exploiting symmetry & sparsity) */
        const float P00_tmp = P00 + dt * (P01 + P01 + P11 * dt);
        const float P01_tmp = P01 + P11 * dt;
        const float P11_tmp = P11 + Q_speed;

        P00 = P00_tmp + Q_angle;
        P01 = P01_tmp;
        P11 = P11_tmp;
    }

    inline void update(float z)
    {
        /* Innovation */
        const float y = z - ang;

        /* S = H·P·Hᵀ + R,  with H = [1 0]  →  S = P00 + R */
        const float S  = P00 + R_angle;
        const float K0 = P00 / S;   // Kalman gain for angle
        const float K1 = P01 / S;   // Kalman gain for speed

        /* State update */
        ang   += K0 * y;
        speed += K1 * y;

        /* Covariance update   P = (I − K·H)·P   */
        const float P00_new = P00 - K0 * P00;
        const float P01_new = P01 - K0 * P01;
        const float P11_new = P11 - K1 * P01;

        P00 = P00_new;
        P01 = P01_new;
        P11 = P11_new;
    }
};

namespace periodics {

class CEncoder : public utils::CTask {
public:
    float _speedCommand = 0.0f;  ///< speed command for the encoder
    /**
     * @brief Constructor for PWM-based AS5048A encoder.
     * @param f_periodTicks Task period in ticks
     * @param g_baseTick    Base tick duration in seconds
     * @param f_serial      Serial interface for debugging
     * @param pwm_pin       PWM input pin
     */
    CEncoder(uint32_t f_periodTicks,
             float g_baseTick,
             BufferedSerial& f_serial,
             PinName pwm_pin);

    ~CEncoder();

    /** @return filtered, de‑wrapped angle in [0,360)° */
    float readAngleDegrees();

    /** @return angular speed (deg/s) */
    float readAngularSpeed();
    float readAngularSpeedKf();

    /** @return angular acceleration (deg/s²) */
    float readAngularAcceleration();

    /** @return filtered angle in [0,360)° */
    float applyHampel(float rawAngleDeg);

    static constexpr size_t HAMPEL_WINDOW = 7;       // must be odd
    static constexpr float  HAMPEL_K      = 3.0f;    // threshold factor (3×MAD)
    static constexpr float  HAMPEL_MINTH  = 0.5f;    // minimum absolute threshold [deg]

    float  _hampelBuf[HAMPEL_WINDOW] = {0};
    size_t _hampelIdx               = 0;
    size_t _hampelCount             = 0;

    /** @return turn count */
    float getTotalDisplacementDegrees();

    /** @return linear speed (m/s) */
    float getLinearSpeed();

    /** @return linear acceleration (m/s²) */
    float getLinearAcceleration();

private:
    virtual void _run() override;

    PwmIn               m_pwm;                ///< PWM input
    BufferedSerial&   m_serial;             ///< Serial for debug
    uint32_t            m_periodTicks;        ///< Task period in ticks
    float               m_dt;                 ///< dt in seconds
    float               m_prevAngle{0.0f};   ///< last angle for derivative
    float               m_prevSpeed{0.0f};   ///< last speed for derivative
    float               DEGREE_PER_CM{-145.0f}; ///< degrees per cm
    Kalman2D     _kf;
    int          _unwrapRevs      = 0;   // revolution counter for unwrapping
    // float        _prevRawAngleDeg = 0.0f;
    float _prevRawForUnwrap = 0.0f;   // last raw, any quality
    float _prevGoodRaw      = 0.0f;   // last sample accepted by the gate
    bool lastRejected{false};  // last sample was rejected by the gate
    Timer        _kfTimer;
    bool         _kfTimerStarted  = false;

    // Biquad structure for 2nd-order Butterworth
    struct Biquad {
        float b0, b1, b2, a1, a2;
        float z1{0.0f}, z2{0.0f};
        float process(float x) {
            float y = b0*x + b1*z1 + b2*z2 - a1*z1 - a2*z2;
            z2 = z1;
            z1 = y;
            return y;
        }
        void reset() { z1 = z2 = 0.0f; }
    };

    float applyHysteresis(float angle);
    float applySpeedHysteresis(float speed);

    float _fs;         ///< sampling frequency (Hz)
    float _hys;        ///< hysteresis half-width (°)
    float _speedHys;   ///< speed hysteresis half-width (deg/s)
    Biquad _sinF, _cosF;
    float  _lastAngle{0.0f};
    float  _lastSpeed{0.0f};
    float  REPORT_INTERVAL_SEC = 0.02f;  ///< speed report interval (s)
    float  sumDelta{0.0f};             ///< sum of deltas for speed
    float  lastT{0.0f};                ///< last time for speed report
    float  lastPublishedSpeed{0.0f};   ///< last published speed
    float displacementDeg{0.0f};         ///< total displacement in degrees
};

} // namespace periodics

#endif // CENCODER_HPP
