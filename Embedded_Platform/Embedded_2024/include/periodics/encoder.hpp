// CEncoder.hpp
#ifndef CENCODER_HPP
#define CENCODER_HPP

#include "mbed.h"
#include <utils/task.hpp>
#include "PwmIn.h"
#include <cmath>

namespace periodics {

class CEncoder : public utils::CTask {
public:
    /**
     * @brief Constructor for PWM-based AS5048A encoder.
     * @param f_period Task period in milliseconds
     * @param f_serial Serial interface for debugging
     * @param pwm_pin PWM input pin
     */
    CEncoder(uint32_t f_period,
             UnbufferedSerial& f_serial,
             PinName pwm_pin);

    ~CEncoder();

    /** @return filtered, de‑wrapped angle in [0,360)° */
    float readAngleDegrees();

    /** @return angular speed (deg/s) */
    float readAngularSpeed();

    /** @return angular acceleration (deg/s²) */
    float readAngularAcceleration();

    /** @return turn count */
    int getTurnCount();

    /** @return linear speed (m/s) */
    float getLinearSpeed();

    /** @return linear acceleration (m/s²) */
    float getLinearAcceleration();

private:
    virtual void _run() override;

    PwmIn               m_pwm;                ///< PWM input
    UnbufferedSerial&   m_serial;             ///< Serial for debug
    uint32_t            m_period;             ///< Task period (ms)
    float               m_dt;                 ///< dt = m_period/1000
    float               m_prevAngle{0.0f};   ///< last angle for derivative
    float               m_prevSpeed{0.0f};   ///< last speed for filtered derivative

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

    // IIR filter state
    float _fs;        ///< sampling frequency (Hz)
    float _hys;       ///< hysteresis half-width (°)
    Biquad _sinF, _cosF;
    float  _lastAngle{0.0f};
};

} // namespace periodics

#endif // CENCODER_HPP
