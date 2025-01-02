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

#include <drivers/steeringmotor.hpp>
#include <cmath>


namespace drivers{
    /**
     * @brief It initializes the pwm parameters and it sets the steering in zero position, the limits of the input degree value.
     * 
     * @param f_pwm               pin connected to servo motor
     * @param f_inf_limit         inferior limit 
     * @param f_sup_limit         superior limit
     * 
     */
    CSteeringMotor::CSteeringMotor(
            PinName f_pwm_pin, 
            float f_inf_limit, 
            float f_sup_limit
        )
        :m_pwm_pin(f_pwm_pin)
        ,m_inf_limit(f_inf_limit)
        ,m_sup_limit(f_sup_limit)
    {
        // Set the ms_period on the pwm_pin
        m_pwm_pin.period_ms(ms_period); 
        // Set position to zero   
        m_pwm_pin.write(zero_default);
    };


    /** @brief  CSteeringMotor class destructor
     */
    CSteeringMotor::~CSteeringMotor()
    {
    };
    
    /**
    * @brief Interpolates values based on steering input.
    *
    * This function interpolates `stepValues` and `zeroDefaultValues` based on the provided `steering` input.
    * The interpolation is made using `steeringValueP` and `steeringValueN` as reference values.
    *
    * @param steering The input steering value for which the values need to be interpolated.
    * @param steeringValueP Positive reference values for steering.
    * @param steeringValueN Negative reference values for steering.
    * @param stepValues Step values corresponding to steeringValueP and steeringValueN which need to be interpolated.
    * @param zeroDefaultValues Zero default values corresponding to steeringValueP and steeringValueN for interpolation.
    * @param size The size of the arrays.
    * @return A pair of interpolated values: { interpolated stepValue, interpolated zeroDefaultValue }.
    */
    std::pair<float, float> CSteeringMotor::interpolate(float steering, const float steeringValueP[], const float steeringValueN[], const float stepValues[], const float zeroDefaultValues[], int size)
    {
        // If steering is within the bounds of the first positive and negative reference values
        if(steering <= steeringValueP[0]){
            if (steering >= steeringValueN[0])
            {
                return {stepValues[0], zeroDefaultValues[0]};
            }
            else{
                for(int i=1; i<size; i++)
                {
                    // Find the interval of negative reference values where steering falls into
                    if (steering >= steeringValueN[i])
                    {
                        // Calculate slopes for interpolation
                        float slopeStepValue = (stepValues[i] - stepValues[i-1]) / (steeringValueN[i] - steeringValueN[i-1]);
                        float slopeZeroDefault = (zeroDefaultValues[i] - zeroDefaultValues[i-1]) / (steeringValueN[i] - steeringValueN[i-1]);

                        // Return the interpolated values
                        return {stepValues[i-1] + slopeStepValue * (steering - steeringValueN[i-1]), zeroDefaultValues[i-1] + slopeZeroDefault * (steering - steeringValueN[i-1])};
                    }
                }
            }
            
        }

        // Boundary conditions for positive and negative reference values
        if(steering >= steeringValueP[size-1]) return {stepValues[size-1], zeroDefaultValues[size-1]};
        if(steering <= steeringValueN[size-1]) return {stepValues[size-1], zeroDefaultValues[size-1]};

        // Interpolation for values between positive reference values
        for(int i=1; i<size; i++)
        {
            if (steering <= steeringValueP[i])
            {
                // Calculate slopes for interpolation
                float slopeStepValue = (stepValues[i] - stepValues[i-1]) / (steeringValueP[i] - steeringValueP[i-1]);
                float slopeZeroDefault = (zeroDefaultValues[i] - zeroDefaultValues[i-1]) / (steeringValueP[i] - steeringValueP[i-1]);

                // Return the interpolated values
                return {stepValues[i-1] + slopeStepValue * (steering - steeringValueP[i-1]), zeroDefaultValues[i-1] + slopeZeroDefault * (steering - steeringValueP[i-1])};
            }
        }

        // Default return if no interval is found
        return {-1, -1};
    };

    /** @brief  It modifies the angle of the servo motor, which controls the steering wheels. 
     *
     *  @param f_angle      angle degree, where the positive value means right direction and negative value the left direction. 
     */
    void CSteeringMotor::setAngle(float f_angle)
    {
        std::pair<float, float> interpolationResult;

        interpolationResult = interpolate(f_angle, steeringValueP, steeringValueN, stepValues, zeroDefaultValues, 2);
        step_value = interpolationResult.first;
        zero_default = interpolationResult.second;

        m_pwm_pin.write(conversion(f_angle));
    };

    /**
     * MODIFIED FUNCTION BY MALO
     * @brief Takes as input a PWM value to set the servo to a specified position
     * @param f_PWM PWM value to be given, AKA duty cycle
     *  */    
        void CSteeringMotor::PWMAngle(float f_PWM)
    {
        // Writes the PWM value to the PIN
        m_pwm_pin.write(f_PWM);
    };

    /**
     * MODIFIED FUNCTION BY MALO
     * @brief Takes as input an angle to set the servo to a specified position
     * @param f_angle angle to be given
     * Computes the angle based on quadratic function, experimentally defined
     *  */    
        void CSteeringMotor::CalculateAngle(float f_angle)
    {
        // dutyCycle output value to the pin
        float dutyCycle = zero_default;
        // Previous input angle
        static float prev_angle =0;
        // Quadratic function parameters
        float alpha = 0;
        float beta = 0;
        float gamma = 0;
        // Zero default when returning from a left turn
        ZD_left = 0.0779;
        // Zero default when returning from a right turn
        ZD_right = 0.0763;
        // Function to calculate the positive angle (LEFT TURN)
        if(f_angle > 0)
        {
            // Update quadratic function parameters
            alpha = -20697;
            beta = 1815.5;
            gamma = -17.982;
            // Compute the dutyCycle 
            dutyCycle = (-beta - std::sqrt(beta*beta - 4*alpha*(gamma - f_angle)))/(2*alpha);
        }
        // Function to calculate the negative angles (RIGHT TURN)
        if(f_angle < 0)
        {
            // Update quadratic function parameters
            alpha = -22406;
            beta = 4884.5;
            gamma = -245.36;
            // Compute the dutyCycle 
            dutyCycle = (-beta + std::sqrt(beta*beta - 4*alpha*(gamma + f_angle)))/(2*alpha);
        }
        // Special case if we want to reset to 0 and set the car to go straight
        if(f_angle == 0)
        {
            if(prev_angle >= 0) // Zero default for returning from LEFT turns
            {
                dutyCycle = ZD_left;
            }
            
            if(prev_angle < 0) // Zero default for returning from RIGHT turns
            {
                dutyCycle = ZD_right;
            }
        }
        //Update the angle to remember
        prev_angle = f_angle;
        // Write the output to the pin
        m_pwm_pin.write(dutyCycle);
    };

    /** @brief  It converts angle degree to duty cycle for pwm signal. 
     * 
     *  @param f_angle    angle degree
     *  \return         duty cycle in interval [0,1]
     */
    float CSteeringMotor::conversion(float f_angle)
    {
        return (step_value * f_angle + zero_default);
    };
//
    /**
     * @brief It verifies whether a number is in a given range
     * 
     * @param f_angle value 
     * @return true means, that the value is in the rangem_inf_limit
     * @return false means, that the value isn't in the range
     */
    bool CSteeringMotor::inRange(float f_angle){
        return m_inf_limit<=f_angle && f_angle<=m_sup_limit;
    };  
}; // namespace hardware::drivers