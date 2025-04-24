
// CEncoder.cpp
#include "periodics/encoder.hpp"
#include <cstdio>
#include <cmath>

namespace periodics {

CEncoder::CEncoder(uint32_t f_period,
                   UnbufferedSerial& f_serial,
                   PinName pwm_pin)
    : utils::CTask(f_period),
      m_pwm(pwm_pin),
      m_serial(f_serial),
      m_period(f_period),
      m_dt(f_period / 1000.0f)
{
    // compute sampling frequency and hysteresis
    _fs  = 1000.0f / m_period;   // Hz
    _hys = 3.0f;                 // degrees of hysteresis

    // design 2nd-order Butterworth via bilinear transform
    float cutoffHz = 10.0f;
    float K  = tanf(M_PI * cutoffHz / _fs);
    float K2 = K * K;
    float norm = 1.0f + sqrtf(2.0f)*K + K2;

    _sinF.b0 = _cosF.b0 =  K2 / norm;
    _sinF.b1 = _cosF.b1 =  2.0f * K2 / norm;
    _sinF.b2 = _cosF.b2 =  K2 / norm;

    _sinF.a1 = _cosF.a1 =  2.0f * (K2 - 1.0f) / norm;
    _sinF.a2 = _cosF.a2 =  (1.0f - sqrtf(2.0f)*K + K2) / norm;

    _sinF.reset();
    _cosF.reset();

    printf("PWM Encoder Initialized on pin %d\n", pwm_pin);
}

CEncoder::~CEncoder() {
}

float CEncoder::applyHysteresis(float angle) {
    if      (angle >  _lastAngle + _hys) _lastAngle = angle;
    else if (angle <  _lastAngle - _hys) _lastAngle = angle;
    else                                  angle = _lastAngle;
    return _lastAngle = angle;
}

float CEncoder::readAngleDegrees() {
    // 1) Raw PWM → angle (0..360°)
    float rawDeg = m_pwm.dutycycle() * 360.0f;
    // 2) Convert to radians
    float rawRad = rawDeg * M_PI / 180.0f;
    // 3) Split and filter
    float s = sinf(rawRad);
    float c = cosf(rawRad);
    float fs = _sinF.process(s);
    float fc = _cosF.process(c);
    // 4) Reconstruct angle
    float ang = atan2f(fs, fc) * 180.0f / M_PI;
    if (ang < 0.0f) ang += 360.0f;
    // 5) Hysteresis
    return applyHysteresis(ang);
}

float CEncoder::readAngularSpeed() {
    float angle = readAngleDegrees();
    float dt = m_dt;
    // MATLAB-style filtered derivative
    constexpr float Tfd = 0.07f;
    float a = Tfd / (Tfd + dt);
    float b = 1.0f / (Tfd + dt);

    float delta = angle - m_prevAngle;
    if (delta >  180.0f) delta -= 360.0f;
    if (delta < -180.0f) delta += 360.0f;

    float speed = a * m_prevSpeed + b * delta;
    m_prevAngle = angle;
    m_prevSpeed = speed;
    return speed;
}

float CEncoder::readAngularAcceleration() {
    float dt = m_dt;
    static bool first = true;
    static float prevSpeed = 0.0f;
    static float filtAcc = 0.0f;
    float curSpeed = readAngularSpeed();
    float rawAcc = first ? 0.0f : (curSpeed - prevSpeed) / dt;
    first = false;
    prevSpeed = curSpeed;
    // EMA for acceleration
    constexpr float alpha = 0.2f;
    filtAcc = alpha*rawAcc + (1.0f-alpha)*filtAcc;
    return filtAcc;
}

int CEncoder::getTurnCount() {
    float current = readAngleDegrees();
    static float prev = current;
    static int count = 0;
    float delta = current - prev;
    if (delta < -180.0f) count++;
    else if (delta > 180.0f) count--;
    prev = current;
    return count;
}

float CEncoder::getLinearSpeed() {
    return readAngularSpeed() * 0.1f;
}

float CEncoder::getLinearAcceleration() {
    return readAngularAcceleration() * 0.1f;
}

void CEncoder::_run() {
    float angle = readAngleDegrees();
    float speed = readAngularSpeed();
    float acc   = readAngularAcceleration();
    printf("PWM Angle: %.2f°\n", angle);
    printf("PWM Speed: %.2f°/s\n", speed);
    printf("PWM Acceleration: %.2f°/s²\n", acc);
}

} // namespace periodics
