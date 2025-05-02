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
                UnbufferedSerial& f_serial,
                PinName pwm_pin)
    : utils::CTask(f_periodTicks),
      m_pwm(pwm_pin),
      m_serial(f_serial),
      m_periodTicks(f_periodTicks)
{
    // 1) convert ticks → seconds
    m_dt = m_periodTicks * g_baseTick;    // e.g. 1 * 0.0001 = 0.0001 s

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

float CEncoder::readAngularSpeed() {
    // ————— state (static or member)
    static Timer  timer;
    static bool   runOnce  = false;
    static float  prevAng  = 0.0f;
    static float  sumDelta = 0.0f;
    static float  lastT    = 0.0f;

    // one-time timer start
    if (!runOnce) {
        timer.start();
        prevAng = readAngleDegrees();
        lastT   = timer.read();
        runOnce = true;
    }

    // 1) sample angle & delta
    float ang   = readAngleDegrees();
    float delta = ang - prevAng;
    if      (delta >  180.0f) delta -= 360.0f;
    else if (delta < -180.0f) delta += 360.0f;
    sumDelta   += delta;
    prevAng     = ang;

    // 2) check if window elapsed
    float now = timer.read();
    if (now - lastT < REPORT_INTERVAL_SEC) {
        return lastPublishedSpeed;  // or 0 if you only update periodically
    }

    // 3) compute & publish speed
    float dt    = now - lastT;
    float speed = sumDelta / dt;     // deg/s

    // 4) optional filtering / hysteresis
    static float speedIIR = 0.0f;
    constexpr float tau_speed = 0.2f;       // time-constant in seconds
    float alpha = tau_speed / (tau_speed + dt);
    speedIIR   = alpha * speedIIR + (1.0f - alpha) * speed;
    speed      = speedIIR;
    // speed = clamp(speed, -MAX_PHYS, +MAX_PHYS);
    speed = applySpeedHysteresis(speed);

    // 5) reset for next interval
    // printf("[Time] = %.2f s, delta = %.2f°\n", dt, sumDelta);
    sumDelta = 0.0f;
    lastT    = now;
    lastPublishedSpeed = speed;
    // printf("\n[Encoder] speed = %.2f°/s\n", speed);

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
    // ————— One-time Timer setup
    static Timer execTimer;
    static bool timerStarted = false;
    if (!timerStarted) {
        execTimer.start();
        timerStarted = true;
    }

    // ————— 1) Mark start of this execution
    uint32_t start_us = execTimer.read_us();

    // ————— 2) Read the filtered angle every tick
    float angleDeg = readAngleDegrees();
    float speedDeg = readAngularSpeed();
    printf("[Encoder] angle = %.2f°, speed = %.2f°/s\n", angleDeg, speedDeg);

    // ————— 4) Package angle+speed into a TelemetryMsg and push
    {
        TelemetryMsg msg;
        msg.type   = PacketType::Encoder;
        msg.ts_us  = execTimer.read_us();
        msg.data.encoder.angle_hundredths = static_cast<int16_t>(angleDeg);
        msg.data.encoder.speed_hundredths = static_cast<int32_t>(speedDeg);
        rb_push(msg);
    }

    // ————— 5) Mark end of execution and accumulate for average print
    uint32_t end_us     = execTimer.read_us();
    uint32_t elapsed_us = end_us - start_us;

    static uint64_t sum_exec   = 0;
    static uint32_t count_exec = 0;
    static uint64_t sum_interval_us = 0;
    static uint32_t count_interval  = 0;
    static uint32_t prevStart_us    = 0;

    // Accumulate period statistics (unchanged)
    if (prevStart_us != 0) {
        uint32_t delta_start = start_us - prevStart_us;
        sum_interval_us += delta_start;
        count_interval++;
    }
    prevStart_us = start_us;

    sum_exec   += elapsed_us;
    count_exec += 1;

    constexpr uint32_t AVG_N = 200;
    if (count_exec >= AVG_N) {
        uint32_t avg_exec     = sum_exec / AVG_N;
        uint32_t avg_interval = (count_interval > 0)
                                  ? static_cast<uint32_t>(sum_interval_us / count_interval)
                                  : 0;

        printf("\n[Encoder] avg exec = %u µs, avg period = %u µs over %u runs\n",
               avg_exec, avg_interval, AVG_N);

        sum_exec         = 0;
        count_exec       = 0;
        sum_interval_us  = 0;
        count_interval   = 0;
    }
    printf("[Encoder Run] angle = %.2f°, speed = %.2f°/s\n", angleDeg, speedDeg);
}

} // namespace periodics
