
// CEncoder.cpp
#include "periodics/encoder.hpp"
#include <cstdio>
#include <cmath>
#include <algorithm>

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
    _hys = 1.0f;                 // degrees of hysteresis
    _speedHys  = 50.0f;         // speed hysteresis (deg/s)


    // design 2nd-order Butterworth via bilinear transform
    float cutoffHz = 30.0f;
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

float CEncoder::applyHampel(float newSampleDeg)
{
    // 1) insert into circular buffer
    _hampelBuf[_hampelIdx] = newSampleDeg;
    _hampelIdx = (_hampelIdx + 1) % HAMPEL_WINDOW;
    if (_hampelCount < HAMPEL_WINDOW) {
        ++_hampelCount;
        return newSampleDeg;         // not enough data yet
    }

    // 2) copy & sort to find median
    float sorted[HAMPEL_WINDOW];
    for (size_t i = 0; i < HAMPEL_WINDOW; ++i)
        sorted[i] = _hampelBuf[i];
    std::sort(sorted, sorted + HAMPEL_WINDOW);
    float median = sorted[HAMPEL_WINDOW/2];

    // 3) compute Median Absolute Deviation (MAD)
    float dev[HAMPEL_WINDOW];
    for (size_t i = 0; i < HAMPEL_WINDOW; ++i)
        dev[i] = fabsf(_hampelBuf[i] - median);
    std::sort(dev, dev + HAMPEL_WINDOW);
    float mad = dev[HAMPEL_WINDOW/2];

    // 4) threshold = max(K·MAD, minimum)
    float thresh = HAMPEL_K * mad;
    if (thresh < HAMPEL_MINTH) thresh = HAMPEL_MINTH;

    // 5) spike veto
    if (fabsf(newSampleDeg - median) > thresh) {
        return median;    // reject spike
    }
    return newSampleDeg;  // accept sample
}


float CEncoder::applyHysteresis(float angle) {
    if      (angle >  _lastAngle + _hys) _lastAngle = angle;
    else if (angle <  _lastAngle - _hys) _lastAngle = angle;
    else                                  angle = _lastAngle;
    return _lastAngle = angle;
}

float CEncoder::applySpeedHysteresis(float speed) {
    if      (speed >  _lastSpeed + _speedHys) _lastSpeed = speed;
    else if (speed <  _lastSpeed - _speedHys) _lastSpeed = speed;
    else                                      speed = _lastSpeed;
    return _lastSpeed = speed;
}

float CEncoder::readAngleDegrees() {
    // 1) Raw PWM → angle (0..360°)
    float rawDeg = m_pwm.dutycycle() * 360.0f;
    // 1.1) Sanitize angle
    // rawDeg = applyHampel(rawDeg);
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
    float dt    = m_dt;

    // wrap-around difference
    float delta = angle - m_prevAngle;
    if (delta >  180.0f) delta -= 360.0f;
    if (delta < -180.0f) delta += 360.0f;

    // compute raw derivative
    float rawDeriv = delta / dt;

    // first-order low-pass on derivative
    constexpr float Tfd = 0.03f;               // derivative filter time constant
    float alpha = Tfd / (Tfd + dt);
    float beta  = dt  / (Tfd + dt);
    float rawSpeed = alpha * m_prevSpeed + beta * rawDeriv;

    // optional extra smoothing
    static float speedFilt = rawSpeed;
    constexpr float tau = 0.1f;               // extra smoothing time constant
    float a2 = tau / (tau + dt);
    speedFilt = a2 * speedFilt + (1.0f - a2) * rawSpeed;

    // apply hysteresis to speed
    float speed = applySpeedHysteresis(speedFilt);

    // update state
    m_prevAngle = angle;
    m_prevSpeed = rawSpeed;

    return -speed;
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
    // --- 1) One‐time setup of our timestamp Timer
    static Timer ts_timer;
    static bool   ts_started = false;
    if (!ts_started) {
        ts_timer.start();
        ts_started = true;
    }

    // --- 2) Grab our timestamp (µs since ts_timer.start())
    uint32_t now_us = ts_timer.read_us();

    // --- 3) Read your sensor values
    float angle = readAngleDegrees();
    float speed = readAngularSpeed();
    // float acc   = readAngularAcceleration(); // if you need it later

    // --- 4) Single printf with timestamp, angle, speed
    printf("@TS:%lu us, AN:%.2f°, SP:%.2f°/s\n",
           now_us,
           angle,
           speed);
}

} // namespace periodics
