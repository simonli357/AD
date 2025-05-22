// CEncoder.hpp
#ifndef CENCODER_HPP
#define CENCODER_HPP

#include "mbed.h"
#include <utils/task.hpp>
#include "PwmIn.h"
#include <cmath>
#include <algorithm>

namespace periodics {

class CEncoder : public utils::CTask {
public:
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
    float               DEGREE_PER_CM{-146.0f}; ///< degrees per cm

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
