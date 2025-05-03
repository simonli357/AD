/* Include guard */
#ifndef SPEEDINGMOTOR_HPP
#define SPEEDINGMOTOR_HPP

/* The mbed library */
#include <mbed.h>

/* cmath library for calculating the square root*/
#include <cmath>

#include <cmath>
#include <utils/task.hpp>
#include <chrono>
#include <ctime>
#include <mbed.h>
#include <periodics/encoder.hpp>

namespace periodics {
    class CEncoder;
}

namespace drivers
{
    /**
     * @brief Interface to control the brushless motor.
     * 
     */
    class ISpeedingCommand
    {
        public:
            virtual void PWMSpeed(float f_PWM) = 0 ;
            virtual void CalculateSpeed(float f_speed) = 0;
    };

    /**  
     * @brief Speeding motor driver
     * 
     * It is used to control the Brushless motor (more precisely the ESC), which is connected to driving shaft. The reference speed can be accessed through 'setSpeed' method. 
     * 
     */


    class CSpeedingMotor: public ISpeedingCommand, public utils::CTask
    {
        public:
            /* Constructor */
            CSpeedingMotor(
                uint32_t f_period,
                BufferedSerial&  f_serialPort,
                PinName     f_pwm_pin,
                periodics::CEncoder& f_encoder
            );
            /* Destructor */
            ~CSpeedingMotor();
            void PWMSpeed(float f_PWM);
            // Calculate Speed from Equation
            void CalculateSpeed(float f_speed);
            // Return current speed
            float getSpeed();
            virtual void _run();

        private:
            /** @brief PWM output pin */
            PwmOut m_pwm_pin;
            /* reference to Serial object */
            BufferedSerial&   m_serialPort;
            /* Encoder class reference */
            periodics::CEncoder& m_encoder;
            /** @brief 0 default */
            float zero_default = 0.074568;
            /** @brief 0 default */
            int8_t ms_period = 20;
            /** @brief step_value */
            float step_value = 0.00051;
            float m_dt;             ///< Time interval for speed calculation
            // Class variable that keeps track of current speed, initialized at 0
            float m_currentSpeed = 0;
            float m_currentDutyCycle;
            /* convert speed value to duty cycle for pwm signal */
            float conversion(float f_speed); //angle to duty cycle
            /*PID for speed control based on the encoder*/
            void PIDCommand();
            /* PID control variables */
            float m_desiredSpeed = 0;
            float m_integral = 0.0f;
            float m_proportional = 0.0f;
            float m_derivative = 0.0f;
            float previous_error = 0.0f;
            float time_step = 0.01f; // 10ms
            float error = 0.0f;
            float integral = 0.0f;
            float m_slope;
            float newSpeed;

    }; // class CSpeedingMotor
}; // namespace drivers

#endif// SPEEDINGMOTOR_HPP