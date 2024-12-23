/**
 * Copyright (c) 2019, Bosch Engineering Center Cluj and BFMC organizers
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:

 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.

 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.

 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.

 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
*/

/* Include guard */
#ifndef STEERINGMOTOR_HPP
#define STEERINGMOTOR_HPP

#include <mbed.h>
#include <utility>

namespace drivers
{
    /**
     * @brief Interface to control the steering angle
     * 
     */
    class ISteeringCommand
    {
        public:
            virtual void setAngle(float f_angle) = 0 ;
            virtual bool inRange(float f_angle) = 0 ;
    };


    /**  
     * @brief Steering servo motor driver
     * 
     * It is used to control the servo motor, which is connected to the steering wheels. The steering angle can be accessed through 'setAngle' method. 
     * 
     */
    class CSteeringMotor: public ISteeringCommand
    {
        public:
            /* Constructor */
            CSteeringMotor(
                PinName f_pwm_pin,
                float f_inf_limit,
                float f_sup_limit
            );
            /* Destructor */
            ~CSteeringMotor();
            /* Set angle */
            void setAngle(float f_angle); 
            /* Check if angle in range */
            bool inRange(float f_angle);
        private:
            /** @brief PWM output pin */
            PwmOut m_pwm_pin;
            /** @brief 0 default */
            float zero_default = 0.07672070;
            /** @brief ms_period */
            int8_t ms_period = 20;
            /** @brief step_value */
            float step_value = 0.0009505;
            /** @brief Inferior limit */
            const float m_inf_limit;
            /** @brief Superior limit */
            const float m_sup_limit;
            /* convert angle degree to duty cycle for pwm signal */
            float conversion(float f_angle); //angle to duty cycle

            /* interpolate the step value and the zero default based on the steering value */
            std::pair<float, float> interpolate(float steering, const float steeringValueP[], const float steeringValueN[], const float stepValues[], const float zeroDefaultValues[], int size);

            // Predefined values for steering reference and interpolation
            const float steeringValueP[2] = {15.0, 20.0};
            const float steeringValueN[2] = {-15.0, -20.0};
            const float stepValues[2] = {0.0008594, 0.000951570};
            const float zeroDefaultValues[2] = {0.07714891, 0.07672070};
    }; // class ISteeringCommand
}; // namespace drivers


#endif //STEERINGMOTOR_HPP