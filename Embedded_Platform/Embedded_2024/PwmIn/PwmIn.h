#ifndef MBED_PWMIN_H
#define MBED_PWMIN_H

#include "mbed.h"
#include <chrono>

using namespace std::chrono;

/** PwmIn class to read PWM inputs
 * 
 * Uses InterruptIn to measure changes on the input pin
 * and record the time they occur
 */
class PwmIn {
public:
    /**
     * @brief Create a PwmIn object on a pin that supports InterruptIn.
     * @param p PinName for the PWM input signal
     */
    PwmIn(PinName p);
    
    /**
     * @brief Read the current period of the signal in seconds.
     */
    float period();
    
    /**
     * @brief Read the current pulse width (high time) in seconds.
     */
    float pulsewidth();
    
    /**
     * @brief Read the current duty cycle as a float in [0.0, 1.0].
     */
    float dutycycle();

private:
    /**
     * @brief Interrupt handler for rising edge.
     */
    void rise_handler();
    
    /**
     * @brief Interrupt handler for falling edge.
     */
    void fall_handler();
    
    InterruptIn _input;       ///< The input pin
    Timer       _timer;       ///< Timer for measuring intervals
    float       _pulsewidth;  ///< Most recent high-time measurement (s)
    float       _period;      ///< Most recent period measurement (s)
};

#endif // MBED_PWMIN_H
