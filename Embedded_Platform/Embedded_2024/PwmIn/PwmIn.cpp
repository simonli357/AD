#include "PwmIn.h"

PwmIn::PwmIn(PinName p)
    : _input(p),
      _pulsewidth(0.0f),
      _period(0.0f)
{
    // Attach callbacks for rising/falling edges
    _input.rise(callback(this, &PwmIn::rise_handler));
    _input.fall(callback(this, &PwmIn::fall_handler));

    _timer.start();
}

float PwmIn::period() {
    return _period;
}

float PwmIn::pulsewidth() {
    return _pulsewidth;
}

float PwmIn::dutycycle() {
    // Avoid division by zero
    return (_period > 0.0f) ? (_pulsewidth / _period) : 0.0f;
}

void PwmIn::rise_handler() {
    // The time since last edge is the period
    _period = std::chrono::duration<float>(_timer.elapsed_time()).count();
    _timer.reset();
}

void PwmIn::fall_handler() {
    // The time since last rising edge is the pulse width
    _pulsewidth = std::chrono::duration<float>(_timer.elapsed_time()).count();
}
