/* Include guard */
#ifndef STEERINGMOTOR_HPP
#define STEERINGMOTOR_HPP

#include <mbed.h>
#include <utility>

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
                float f_sup_limit,
                periodics::CImu& f_imu
            );
            /* Destructor */
            ~CSteeringMotor();
            /* Set angle */
            void setAngle(float f_angle); 
            /* Check if angle in range */
            bool inRange(float f_angle);
        private:
            void setYaw();
            periodics::CImu& m_imu;
            float yaw = 0;
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