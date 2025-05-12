#include <drivers/speedingmotor.hpp>
#include <cmath>
#include <utils/task.hpp>
#include <chrono>
#include <ctime>
#include <mbed.h>
#include <periodics/encoder.hpp>

namespace drivers{
    /**
     * @brief It initializes the pwm parameters and it sets the speed reference to zero position, and the limits of the car speed.
     * 
     * @param f_pwm_pin               pin connected to servo motor
     * 
     */

    CSpeedingMotor::CSpeedingMotor(
            uint32_t f_period,
            BufferedSerial&  f_serialPort,
            PinName f_pwm_pin, 
            periodics::CEncoder& f_encoder
        )
        : utils::CTask(f_period)
        , m_serialPort(f_serialPort)
        , m_pwm_pin(f_pwm_pin)
        , m_encoder(f_encoder)
        , m_dt(f_period / 1000.0f) // Convert milliseconds to seconds

    {
        // Set the ms_period on the pwm_pin
        m_pwm_pin.period_ms(ms_period); 
        // Set position to zero   
        m_pwm_pin.write(zero_default);
        m_currentSpeed = 0;
        m_currentDutyCycle = zero_default;
    };

    /** @brief  CSpeedingMotor class destructor
     */
    CSpeedingMotor::~CSpeedingMotor()
    {
    };
    
    /** 
     * MODIFIED FUNCTION BY MALO
     * @brief  Computes the speed of the car based on experimentally determined values
     *  @param f_PWM PWM value to be input to the motor
     **/ 
     void CSpeedingMotor::PWMSpeed(float f_PWM)
    {
        m_pwm_pin.write(f_PWM);
        m_currentDutyCycle = f_PWM;
    };

    /** 
     * MODIFIED FUNCTION BY MALO
     * @brief  Returns the current speed of the motor
     **/ 
    float CSpeedingMotor::getSpeed()
    {
        return m_currentSpeed;
    }

    /** 
     * MODIFIED FUNCTION BY MALO
     * @brief   Calculates the speed of the car based on experimentally defined equation
     * @param   f_speed speed the car will go at in cm/s
     */
    void CSpeedingMotor::CalculateSpeed(float f_speed)
    {
        // dutyCycle output value to the pin
        float dutyCycle = zero_default;
        // Quadratic function parameters
        float alpha = 0;
        float beta = 0;
        float gamma = 0;
        if(f_speed > 0 )
        {
            // Update quadratic function parameters
            alpha = 895099.49;
            beta = -135002.57;
            gamma = 5071.03;
            // Compute the dutyCycle 
            dutyCycle = (-beta - std::sqrt(beta*beta - 4*alpha*(gamma - f_speed)))/(2*alpha);
        }

        if(f_speed < 0 )
        {
            // Update quadratic function parameters
            alpha = 1798545.44;
            beta = -258826.54;
            gamma = 9311.76;
            // Compute the dutyCycle 
            dutyCycle = 0.0055 + (-beta + std::sqrt(beta*beta - 4*alpha*(gamma + f_speed)))/(2*alpha);
        }
        if(f_speed == 0)
        {
            dutyCycle = zero_default;
        }
        // Write the appropriate dutyCycle to the pin
        m_pwm_pin.write(dutyCycle);

        // m_currentDutyCycle = dutyCycle;
        // m_currentSpeed = f_speed;
        // printf("Speed has been set: %f\n", m_currentSpeed);
        // printf("Duty Cycle has been set: %f\n", m_currentDutyCycle);
        // printf("[ACK] speed_set = %.2f\n", f_speed);

        // char buf[48];
        // snprintf(buf, sizeof(buf), "[ACK] speed_set = %.2f\n", f_speed);
        // m_serialPort.write(buf, strlen(buf));
    }

    /** @brief  It converts speed reference to duty cycle for pwm signal. 
     * 
     *  @param f_speed    speed
     *  \return         pwm value
     */
    float CSpeedingMotor::conversion(float f_speed)
    {   
        float dutyCycle =  m_slope * f_speed;
        return dutyCycle;
    };
    

    void CSpeedingMotor::PIDCommand()
    {
        const float MAX_ERROR = 10.0f;  // meters per second

        float currentSpeed = m_encoder.getLinearSpeed();

        // Calculate the error
        float error = m_desiredSpeed - currentSpeed;

        // PID control
        integral += error * m_dt;
        // Integral windup
        if (integral > MAX_ERROR) integral = MAX_ERROR;
        if (integral < -MAX_ERROR) integral = -MAX_ERROR;
        // Derivative term        
        float derivative = (error - previous_error) / m_dt;
        previous_error = error;

        // Adjust steering
        float dutyCycle =  m_currentDutyCycle + (m_proportional * error + m_integral * integral + m_derivative * derivative);

        // Write the appropriate dutyCycle to the pin
        m_pwm_pin.write(dutyCycle);
        m_currentDutyCycle = dutyCycle;

    };

    void CSpeedingMotor::_run()
    {
        // Call the PID function to adjust the steering angle
        PIDCommand();
        return;
    }

}; // namespace hardware::drivers