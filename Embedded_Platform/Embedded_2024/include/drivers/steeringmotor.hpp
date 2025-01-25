/* Include guard */
#ifndef STEERINGMOTOR_HPP
#define STEERINGMOTOR_HPP

#include <mbed.h>
#include <utility>
#include <cmath>
#include <utils/task.hpp>
#include <chrono>

// 1) Forward-declare the IMU class
namespace periodics {
    class CImu;
}

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
            virtual void PWMAngle(float f_PWM) = 0 ;
            virtual float CalculateAngle(float f_angle) = 0 ;
            // Variable to know if the PID is active
            bool m_pidActive = false;
            // PID controller values
            float m_proportional = 0.5;
            float m_integral = 0;
            float m_derivative = 0;
            // Desired steer value
            float m_desiredSteer = 0;
            /** @brief 0 default */
            float zero_default = 0.07672070;
    };

    class CSpeedingMotor;

    /**  
     * @brief Steering servo motor driver
     * 
     * It is used to control the servo motor, which is connected to the steering wheels. The steering angle can be accessed through 'setAngle' method. 
     * 
     */
    class CSteeringMotor: public ISteeringCommand, public utils::CTask
    {
        public:
            /* Constructor */
            CSteeringMotor(
                uint32_t f_period,
                UnbufferedSerial&             f_serialPort,
                PinName f_pwm_pin,
                float f_inf_limit,
                float f_sup_limit,
                periodics::CImu& f_imu,
                drivers::CSpeedingMotor& f_speeding
            );
            /* Destructor */
            ~CSteeringMotor();
            /* Set angle */
            void setAngle(float f_angle); 
            /* Check if angle in range */
            bool inRange(float f_angle);
            void PWMAngle( float f_PWM);
            float CalculateAngle(float f_angle);
            void steerPID();
            virtual void _run();
            void setDutyCycle(float f_dutyCycle);
            
        private:
            void setYaw();
            float imu_yaw = 0;
            /** @brief PWM output pin */
            PwmOut m_pwm_pin;
            /** @brief ms_period */
            int8_t ms_period = 20;
            /** @brief step_value */
            float step_value = 0.0009505;
            /** @brief Inferior limit */
            const float m_inf_limit;
            /** @brief Superior limit */
            const float m_sup_limit;
            // IMU class reference
            periodics::CImu& m_imu;
            // Speeding class reference
            drivers::CSpeedingMotor& m_speedingControl;
            /* convert angle degree to duty cycle for pwm signal */
            float conversion(float f_angle); //angle to duty cycle
            // Values added by MALO
            // Zero default value for when returning from a LEFT Turn
            float ZD_left = 0.0779;
            // Zero default value for when returning from a RIGHT Turn
            float ZD_right = 0.0763;
            // Current steering angles
            float m_currentSteer = 0;
            float m_currentDutyCycle;
            /* reference to Serial object */
            UnbufferedSerial&   m_serialPort;

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