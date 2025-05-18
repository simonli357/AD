#ifndef DCMOTOR_HPP
#define DCMOTOR_HPP

#include <mbed.h>
#include <cmath>

namespace hardware::drivers{

    /**
     * @brief Command setter interface.
     * 
     */
    class IMotorCommand{
        public:
            virtual void setSpeed(float f_pwm) = 0;
            virtual void brake() = 0;
            virtual bool inRange(float f_pwm) =0;
    };

    /**
     * @brief VNH5019 Motor Driver class 
     * 
     * It has role to control the vnh5019 motor driver: the motor speed and the direction.
     * The direction and the rotation speed are regulated by a single value, where the positive values define the forward move, 
     * similary negative values represent the backward move. The magnitude of values gives the motor speed. So generally, 
     * the input signal can vary between [-1,1]. We reduced this interval to avoid the robot high speed.  
     * 
     */
    class CMotorDriverVnh:public IMotorCommand {
        public:
            /* Constructor */
            CMotorDriverVnh(PinName, PinName, PinName, float, float);
            /* Destructor */
            ~CMotorDriverVnh();
            /* Run */
            void setSpeed(float f_pwm);
            /* Brake */
            void brake();
            /* Check the allowed range */
            bool inRange(float f_pwm);
            
        private:
            /** @brief PWM output pin */
            PwmOut      m_pwm;
            /** @brief pin A for direction */
            DigitalOut  m_ina;
            /** @brief pin B for direction */
            DigitalOut  m_inb;

            const float m_pwm_inf_limit;
            const float m_pwm_sup_limit;
    };


}; // namespace hardware::drivers


#endif