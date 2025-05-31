// CEncoder.cpp
#include "periodics/encoder.hpp"
#include <cstdio>
#include <cmath>
#include <algorithm>

// Custom clamp function for compatibility with older C++ standards
template <typename T>
constexpr const T& clamp(const T& v, const T& lo, const T& hi) {
    return (v < lo) ? lo : (v > hi) ? hi : v;
}
#include "mbed.h"                          // <-- for Timer

namespace periodics {

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
    _kf.Q_angle = 0.005f;    // deg²  (slow wander)
    _kf.Q_speed = 6.0f;      // (deg/s)²  (covers 0→50 cm/s braking)
    _kf.R_angle = 0.02f;     // deg²  (~0.14 deg rms measurement noise)
    
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


    printf("PWM Encoder Initialized on pin %d\n", pwm_pin);
}

CEncoder::~CEncoder() {
}

float CEncoder::readAngleDegrees() {
    // 1) Raw PWM → angle (0..360°)
    float rawDeg = m_pwm.dutycycle() * 360.0f;

    // 1.1) (optional) sanitize with Hampel
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

    // 5) Apply hysteresis
    return applyHysteresis(ang);
}

// float CEncoder::readAngleDegrees() {
//     float duty = m_pwm.dutycycle();
//     constexpr float TOTAL_CLOCKS   = 4351.0f;  // (128 high + 4095 data + 128 low)
//     constexpr float FRAME_OFFSET   = 128.0f;   // the fixed 128-high “start-of-frame” count
//     constexpr float MAX_DATA_COUNTS = 4095.0f; // data section length
//     float highCounts = duty * TOTAL_CLOCKS;

//     float dataCounts = highCounts - FRAME_OFFSET;

//     if (dataCounts < 0.0f)       dataCounts = 0.0f;
//     else if (dataCounts > MAX_DATA_COUNTS) dataCounts = MAX_DATA_COUNTS;

//     float rawDeg = dataCounts * (360.0f / 4096.0f);

//     float rawRad = rawDeg * (static_cast<float>(M_PI) / 180.0f);

//     float s = sinf(rawRad);
//     float c = cosf(rawRad);

//     float fs = _sinF.process(s);
//     float fc = _cosF.process(c);

//     float ang = atan2f(fs, fc) * (180.0f / static_cast<float>(M_PI));
//     if (ang < 0.0f) {
//         ang += 360.0f;
//     }

//     return applyHysteresis(ang);
// }

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
    /* ───────────── 1. time base ───────────── */
    if (!_kfTimerStarted) { _kfTimer.start(); _kfTimerStarted = true; }

    static uint32_t last_us = 0;
    const uint32_t now_us = _kfTimer.read_us();
    if (last_us == 0) { last_us = now_us; return 0.0f; }

    const float dt = (now_us - last_us) * 1e-6f;     // ≈ 0.001 s
    last_us = now_us;

    /* ───────────── 2. raw angle ────────────── */
    float rawDeg = m_pwm.dutycycle() * 360.0f;
    // char buf1[48];
    // int n = std::snprintf(buf1, sizeof(buf1),
    //                         "@3:%.3f;;\r\n",
    //                         rawDeg);
    // if (n > 0 && (size_t)n < sizeof(buf1)) m_serial.write(buf1, n);

    /* ───────────── 3. unwrap ───────────────── */
    float diffRaw = rawDeg - _prevRawForUnwrap;           // -360…+360
    if      (diffRaw >  180.0f) _unwrapRevs--;
    else if (diffRaw < -180.0f) _unwrapRevs++;
    _prevRawForUnwrap = rawDeg; 

    float measDeg = rawDeg + 360.0f * _unwrapRevs;       // absolute angle

    /* ───────────── 4. Kalman predict ───────── */
    _kf.predict(dt);

    /* ───────────── 4a. step‑size glitch gate ─ */
    constexpr float STEP_LIM = 15.0f;   // max is 7.5 for 50cm/s at 1000 Hz
    static int  overLimitCnt = 0;       // counts consecutive big steps
    static int  overLimitCntThreshold = 3; // threshold for accepting big steps

    float stepDiff = rawDeg - _prevGoodRaw;
    bool accept;
    if (fabsf(stepDiff) <= STEP_LIM) {               // small step
        accept = true;
    } else if (!lastRejected) {                      // first big step
        accept = false;
        lastRejected = true;
    } else {                                         // second big step in a row
        accept = true;
        lastRejected = false;
    }
    if (accept) {
        _kf.update(measDeg);
        _prevGoodRaw = rawDeg;                 // <- update only on accept
        lastRejected = false;
    }

    /* ───── 4a. innovation‑based glitch gate ────────────────────── */
    // float innov = measDeg - _kf.ang;                     // prediction residual
    // innov -= 360.0f * std::round(innov / 360.0f);        // wrap to (‑180, +180]

    // constexpr float INNOV_LIM = 45.0f;                   // 3 ms of max speed

    // if (fabsf(innov) <= INNOV_LIM) {                     // plausible ?
    //     _kf.update(measDeg);                             // yes -> use it
    // }

    // _kf.update(measDeg);

    /* ───────────── 5. result ───────────────── */
    float speedDegPerSec = _kf.speed;
    speedDegPerSec = applySpeedHysteresis(speedDegPerSec);

    /* ───────────── 6. periodic publish ─────── */
    static float accum = 0.0f;
    accum += dt;
    if (accum >= REPORT_INTERVAL_SEC) {
        accum = 0.0f;
        char buf[48];
        int n = std::snprintf(buf, sizeof(buf),
                              "@5:%.3f;;\r\n",
                              speedDegPerSec / DEGREE_PER_CM);
        if (n > 0 && (size_t)n < sizeof(buf)) m_serial.write(buf, n);
        lastPublishedSpeed = speedDegPerSec;
    }
    // char buf[64];
    // int len = snprintf(buf, sizeof(buf),
    //                 "[Encoder] Total displacement = %.2f°\n",
    //                 displacementDeg);
    // m_serial.write(buf, len);

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
    const float ang_raw = m_pwm.dutycycle() * 360.0f;
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
    // readAngularSpeedKf();
    readAngularSpeed();
    
    // displacementDeg = getTotalDisplacementDegrees(); 
}

} // namespace periodics
