// CEncoder.cpp
#include "periodics/encoder.hpp"
#include <cstdio>
#include <cmath>
#include <algorithm>
#include <deque>

// Custom clamp function for compatibility with older C++ standards
template <typename T>
constexpr const T& clamp(const T& v, const T& lo, const T& hi) {
    return (v < lo) ? lo : (v > hi) ? hi : v;
}
#include "mbed.h"                          // <-- for Timer

namespace periodics {

#if ENCODER_USE_I2C
CEncoder::CEncoder(uint32_t f_periodTicks,
                float g_baseTick,
                BufferedSerial& f_serial,
                PinName sda_pin, PinName scl_pin)
    : utils::CTask(f_periodTicks),
      m_i2c(nullptr),
      m_serial(f_serial),
      m_periodTicks(f_periodTicks),
      DEGREE_PER_CM(-145.0f)
{
    m_i2c = new I2C(sda_pin, scl_pin);
    m_i2c->frequency(400000); // 400 kHz Fast-mode
#else
CEncoder::CEncoder(uint32_t f_periodTicks,
                float g_baseTick,
                BufferedSerial& f_serial,
                PinName pwm_pin)
    : utils::CTask(f_periodTicks),
      m_pwm(pwm_pin),
      m_serial(f_serial),
      m_periodTicks(f_periodTicks),
      DEGREE_PER_CM(-145.0f)
{
#endif
    _kf.Q_angle = 0.005f;    // deg²  (slow wander)
    _kf.Q_speed = 6.0f;      // (deg/s)²  (covers 0→50 cm/s braking)
    _kf.R_angle = 0.02f;     // deg²  (~0.14 deg rms measurement noise)

    // 1) convert ticks → seconds
    m_dt = 0.001f;

    // 2) sampling rate
    _fs  = 1.0f / m_dt;                   // e.g. 1/0.0001 = 10 000 Hz

    // compute sampling frequency and hysteresis
    _hys = 1.0f;                 // degrees of hysteresis
    _speedHys  = 10.0f;         // speed hysteresis (deg/s)


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


    printf("AS5600 Encoder Initialized (%s mode)\n",
           ENCODER_USE_I2C ? "I2C" : "PWM");
}

CEncoder::~CEncoder() {
#if ENCODER_USE_I2C
    delete m_i2c;
    m_i2c = nullptr;
#endif
}

float CEncoder::getRawAngleDeg() {
#if ENCODER_USE_I2C
    // Register 0x0C: RAW ANGLE bits [11:8] (high nibble)
    // Register 0x0D: RAW ANGLE bits [7:0]  (low byte)
    char cmd[1];
    char data[2] = {0};

    // Read high byte from register 0x0C
    cmd[0] = RAW_ANGLE_REG_H;              // 0x0C
    m_i2c->write(AS5600_ADDR, cmd, 1);
    m_i2c->read(AS5600_ADDR, &data[0], 1);

    // Read low byte from register 0x0D
    cmd[0] = RAW_ANGLE_REG_H + 1;          // 0x0D
    m_i2c->write(AS5600_ADDR, cmd, 1);
    m_i2c->read(AS5600_ADDR, &data[1], 1);

    uint16_t raw12 = (static_cast<uint8_t>(data[0]) & 0x0F) << 8
                   |  static_cast<uint8_t>(data[1]);
    return raw12 * 360.0f / 4096.0f;
#else
    return m_pwm.dutycycle() * 360.0f;
#endif
}

float CEncoder::readAngleDegrees() {
    float rawDeg = getRawAngleDeg();
    float rawRad = rawDeg * M_PI / 180.0f;
    float s = sinf(rawRad);
    float c = cosf(rawRad);
    float fs = _sinF.process(s);
    float fc = _cosF.process(c);
    float ang = atan2f(fs, fc) * 180.0f / M_PI;
    if (ang < 0.0f) ang += 360.0f;
    return applyHysteresis(ang);
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

float CEncoder::applySpeedHysteresis(float speed) {
    if      (speed >  _lastSpeed + _speedHys) _lastSpeed = speed;
    else if (speed <  _lastSpeed - _speedHys) _lastSpeed = speed;
    else                                      speed = _lastSpeed;
    return _lastSpeed = speed;
}

float CEncoder::applyHysteresis(float angle) {
    if      (angle >  _lastAngle + _hys) _lastAngle = angle;
    else if (angle <  _lastAngle - _hys) _lastAngle = angle;
    else                                  angle = _lastAngle;
    return _lastAngle = angle;
}

float clamp(float x, float min, float max) {
    if (x < min) return min;
    if (x > max) return max;
    return x;
}

float CEncoder::readAngularSpeedKf()
{
    if (!_kfTimerStarted) {
        _kfTimer.start();
        _kfTimerStarted = true;
    }

    static uint32_t last_us = 0;
    const uint32_t now_us = _kfTimer.read_us();
    if (last_us == 0) {
        last_us = now_us;
        return 0.0f;  // first call, no dt yet
    }

    const float dt = (now_us - last_us) * 1e-6f;   // seconds (≈ 0.001)
    last_us = now_us;

    /* ───────────── 2. raw angle in degrees ─────────────────────── */
    float rawDeg = getRawAngleDeg();

    /* ───────────── 3. unwrap (always use raw for correct topology) ─── */
    static float prevRawForUnwrap = 0.0f;
    float wrapDiff = rawDeg - prevRawForUnwrap;   // range: -360…+360
    if      (wrapDiff >  180.0f) _unwrapRevs--;
    else if (wrapDiff < -180.0f) _unwrapRevs++;
    prevRawForUnwrap = rawDeg;

    float measDeg = rawDeg + 360.0f * _unwrapRevs; // absolute, unwrapped angle

    /* ───────────── 4. Kalman predict ─────────────────────────── */
    _kf.predict(dt);
    _kf.update(measDeg);

    /* ───────────── 5. compute speed result ───────────────────── */
    float speedDegPerSec = _kf.speed;
    speedDegPerSec = applySpeedHysteresis(speedDegPerSec);  // optional dead-band

    /* ───────────── 6. periodic publish ───────────────────────── */
    static float reportAccum = 0.0f;
    reportAccum += dt;
    if (reportAccum >= REPORT_INTERVAL_SEC) {
        reportAccum = 0.0f;

        float filteredCm   = speedDegPerSec / DEGREE_PER_CM;

        static float lastGoodSpeed = 0.0f;
        static std::deque<float> speedCommandHistory = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
        speedCommandHistory.push_back(_speedCommand);
        if (speedCommandHistory.size() > 5) {
            speedCommandHistory.pop_front();
        }
        float min_diff = std::abs(filteredCm - speedCommandHistory[0]);
        for (size_t i = 0; i < speedCommandHistory.size(); ++i) {
            float diff = std::abs(filteredCm - speedCommandHistory[i]);
            if (diff < min_diff) {
                min_diff = diff;
            }
        }
        if (min_diff / std::abs(_speedCommand) > 0.15f) { // 15%
        // if (min_diff > 9.9f) {
        // if (false) {
            // filteredCm = lastGoodSpeed; // filter out spikes
            filteredCm = _speedCommand; // filter out spikes
        } else {
            lastGoodSpeed = filteredCm; // update last good speed
        }

        char buf[128];
        // int n = std::snprintf(buf, sizeof(buf),
        //                     "@5:%.3f;%.3f;;\r\n",
        //                     sendSpeedCm,
        //                     _speedCommand);
        int n = std::snprintf(buf, sizeof(buf),
                            "@5:%.3f;%.3f;%.3f;%.3f;;\r\n",
                            filteredCm,
                            rawDeg,
                            speedDegPerSec,
                            _speedCommand);

        // int n = std::snprintf(buf, sizeof(buf),
        //         "@5:%.3f;;\r\n",
        //         filteredCm);

        if (n > 0 && static_cast<size_t>(n) < sizeof(buf)) {
            m_serial.write(buf, n);
        }

        // 5) for internal use, always keep the last "filtered" speed in deg/s
        lastPublishedSpeed = speedDegPerSec;
    }

    return speedDegPerSec;
}
float CEncoder::readAngularSpeed() {
    static Timer  timer;
    static bool   runOnce  = false;
    static float  prevAng  = 0.0f;
    static float  sumDelta = 0.0f;
    static float  lastT    = 0.0f;

    if (!runOnce) {
        timer.start();
        prevAng = readAngleDegrees();
        lastT   = timer.read();
        runOnce = true;
    }

    float ang   = readAngleDegrees();
    float delta = ang - prevAng;
    if      (delta >  180.0f) delta -= 360.0f;
    else if (delta < -180.0f) delta += 360.0f;
    sumDelta   += delta;
    prevAng     = ang;

    float now = timer.read();
    if (now - lastT < REPORT_INTERVAL_SEC) {
        return lastPublishedSpeed;  // or 0 if you only update periodically
    }

    float dt    = now - lastT;
    float speed = sumDelta / dt;     // deg/s

    //filtering / hysteresis
    static float speedIIR = 0.0f;
    constexpr float tau_speed = 0.025f;       // time-constant in seconds
    float alpha = tau_speed / (tau_speed + dt);
    speedIIR   = alpha * speedIIR + (1.0f - alpha) * speed;
    speed      = speedIIR;
    speed = applyHampel(speed);
    // speed = clamp(speed, -MAX_PHYS, +MAX_PHYS);
    speed = applySpeedHysteresis(speed);

    // 5) reset for next interval
    sumDelta = 0.0f;
    lastT    = now;
    lastPublishedSpeed = speed;

    char buf[64];
    int n = std::snprintf(buf, sizeof(buf),
                    "@5:%.3f;;\r\n",
                    speed / DEGREE_PER_CM);
    if (n > 0 && static_cast<std::size_t>(n) < sizeof(buf)) {
        m_serial.write(buf, n);
    }

    // int len = snprintf(buf, sizeof(buf),
    //                 "[Encoder] Total displacement = %.2f°\n",
    //                 displacementDeg);
    // m_serial.write(buf, len);

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

float CEncoder::getTotalDisplacementDegrees()
{
    const float ang_raw = getRawAngleDeg();
    static int   revs = 0;
    static float prev = ang_raw;

    float delta = ang_raw - prev;

    if (delta >  180.0f) revs--;      // wrapped 360→0 forward
    else if (delta < -180.0f) revs++; // wrapped 0→360 backward

    prev = ang_raw;
    float total = revs * 360.0f + ang_raw;
    return total;
}

float CEncoder::getLinearSpeed() {
    return readAngularSpeed() * 0.1f;
}

float CEncoder::getLinearAcceleration() {
    return readAngularAcceleration() * 0.1f;
}

void CEncoder::_run() {
    readAngularSpeedKf();
    // readAngularSpeed();

    // displacementDeg = getTotalDisplacementDegrees();
}

} // namespace periodics
