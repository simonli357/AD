#include <drivers/steeringmotor.hpp>
#include <periodics/imu.hpp>
#include <cmath>
#include <mbed.h>
#include <speedingmotor.hpp>
#include <utils/task.hpp>
#include <chrono>
#include <ctime>

namespace drivers{
    /**
     * @brief It initializes the pwm parameters and it sets the steering in zero position, the limits of the input degree value.
     * 
     * @param f_pwm               pin connected to servo motor
     * @param f_inf_limit         inferior limit 
     * @param f_sup_limit         superior limit
     * 
     */

    // Create timer for timing time_step on PID task
    Timer PID_timer;

    CSteeringMotor::CSteeringMotor(
            uint32_t f_period,
            UnbufferedSerial&  f_serialPort,
            PinName f_pwm_pin, 
            periodics::CImu& f_imu,
            drivers::CSpeedingMotor& f_speedingControl

        )
        : utils::CTask(f_period)
        , m_serialPort(f_serialPort)
        , m_pwm_pin(f_pwm_pin)
        , m_imu(f_imu)
        , m_speedingControl(f_speedingControl)
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
     * MODIFIED FUNCTION BY MALO
     * @brief PID to adjust steering angle using feedback from IMU
     */
    void CSteeringMotor::steerPID() {

        const float WHEELBASE = 0.260f; // meters
        const float MAX_ERROR = 100.0f;  // degrees

        // Retrieve current speed in m/s
        float c_speed = 0.01 * m_speedingControl.getSpeed();

        // Calculate time step
        PID_timer.stop();
        auto raw_elapsed_time = PID_timer.elapsed_time(); // Raw time in microoseconds
        // printf("Raw elapsed time: %lld us\n", raw_elapsed_time.count());

        float time_step = raw_elapsed_time.count() * 1e-6; // Convert microseconds to seconds
        printf("Time step: %f seconds\n", time_step);
        if(time_step > 0.1) time_step = 0.1; // Cap time step to 0.1 seconds
        if(time_step < 0.001) time_step = 0.001; // Cap time step to 0.001 seconds  

        // Update time elapsed
        time_elapsed += time_step;

        PID_timer.reset();
        PID_timer.start();

        // Compute yaw rate and cumulative yaw
        float yaw_rate = (c_speed / WHEELBASE) * (tan(m_desiredSteer * M_PI / 180.0f) * 180 / M_PI);
        // printf("Yaw rate: %f\n", yaw_rate);
        yaw_calc += yaw_rate * time_step;

        // Keep the yaw beteween 0 and 360
        if(yaw_calc > 360.0f) yaw_calc -= 360.0f;
        if(yaw_calc < 0.0f) yaw_calc += 360.0f;

        // printf("Current speed : %f\n", c_speed);

        // Error needs to be between [-180,180]
        float error = yaw_calc - imu_yaw;
        if(error > 180.0f) error -= 360.0f;
        if(error < -180.0f) error += 360.0f;

        // Print debug information
        printf("@E1:%f\n", error);

        // Convert to float (Unix timestamp + milliseconds fraction)
        printf("@T1:%f\n", time_elapsed);
        printf("@Y1:%f\n", yaw_calc);
        printf("@Y2:%f\n", imu_yaw);

        // PID control
        integral += error * time_step;
        // Integral windup
        if(integral > MAX_ERROR) integral = MAX_ERROR;
        float derivative = (error - previous_error) / time_step;
        previous_error = error;

        // Adjust steering
        newSteer =  m_desiredSteer + (m_proportional * error + 0*m_integral * integral + m_derivative * derivative);

        float integral_calc = m_integral * integral ;
        float derivative_calc = m_derivative * derivative;
        m_currentSteer = newSteer;
        float clipSteer = newSteer;
        if(newSteer > 20.8) clipSteer= 20.8;
        if(newSteer < -21.8) clipSteer = -21.8;

        // printf("Current Steer: %f\n", newSteer);
        // printf("Desired Steer: %f\n", m_desiredSteer);

        // Actuate steering
        newDutyCycle = CalculateAngle(newSteer);
        PWMAngle(newDutyCycle);
        printf("@Y4:%f\n", newSteer);
        printf("@Y5:%f\n", clipSteer);
        printf("@S1:%f\n", m_desiredSteer);

        printf("@E4:%f\n", m_proportional);
        printf("@E2:%f\n", integral_calc);
        printf("@E3:%f\n", derivative_calc);

        printf("Sent duty cycle: %f\n", newDutyCycle);

    }

    /**
     * MODIFIED FUNCTION BY MALO
     * @brief Takes as input a PWM value to set the servo to a specified position
     * @param f_PWM PWM value to be given, AKA duty cycle
     *  */    
        void CSteeringMotor::PWMAngle(float f_PWM)
    {
        // Writes the PWM value to the PIN
        m_pwm_pin.write(f_PWM);
        m_currentDutyCycle = f_PWM;
    };

    /**
     * MODIFIED FUNCTION BY MALO
     * @brief Takes as input an angle and returns the PWM value to set servo to that position
     * @param f_angle angle to be given
     * Computes the angle based on quadratic function, experimentally defined
     *  */    
        float CSteeringMotor::CalculateAngle(float f_angle)
    {
        // dutyCycle output value to the pin
        float dutyCycle = zero_default;
        // Previous input angle
        static float prev_angle = 0;
        // Quadratic function parameters
        float alpha = 0;
        float beta = 0;
        float gamma = 0;
        // Zero default when returning from a left turn
        ZD_left = 0.0772;
        // Zero default when returning from a right turn
        ZD_right = 0.0755;
        // Clip the steering angle for safety
        if(f_angle > 20.8) f_angle = 20.8; 
        if(f_angle < -21.8) f_angle = -21.8;

        // Function to calculate the positive angle (LEFT TURN)
        if(f_angle < 0)
        {
            // Update quadratic function parameters
            alpha = -22254;
            beta = 1995.5;
            gamma = -23.127;
            // Compute the dutyCycle 
            dutyCycle = (-beta - std::sqrt(beta*beta - 4*alpha*(gamma + f_angle)))/(2*alpha);
        }

        // Function to calculate the negative angles (RIGHT TURN)
        if(f_angle > 0)
        {
            // Update quadratic function parameters
            alpha = -18728;
            beta = 4175;
            gamma = -211.28;
            // Compute the dutyCycle 
            dutyCycle = (-beta + std::sqrt(beta*beta - 4*alpha*(gamma - f_angle)))/(2*alpha);
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
        // printf("Current Duty Cycle:%f\n", dutyCycle);
        //Update the angle to remember
        prev_angle = f_angle;
        // Return the computed dutyCycle
        return dutyCycle;
    };

    /**
     * MODIFIED FUNCTION by Malo
     * @brief Sets the servo to a specified position
     * @param f_dutyCycle Duty Cycle to set the servo position
     */
    void CSteeringMotor::setDutyCycle(float f_dutyCycle)
    {
        m_pwm_pin.write(f_dutyCycle);

    }
    /** @brief  It converts angle degree to duty cycle for pwm signal. 
     * 
     *  @param f_angle    angle degree
     *  \return         duty cycle in interval [0,1]
     */
    float CSteeringMotor::conversion(float f_angle)
    {
        return (step_value * f_angle + zero_default);
    };

    void CSteeringMotor::setYaw(){
        // imu_yaw = m_imu.getYaw();
    };

    void CSteeringMotor::setPID(float f_proportional, float f_integral, float f_derivative){
        m_proportional = f_proportional;
        m_integral = f_integral;
        m_derivative = f_derivative;
        m_pidActive = true;
        yaw_calc = 0.0f;
        integral = 0.0f;
        previous_error = 0.0f;
        newSteer = 0.0f;
        newDutyCycle = 0.0f;
        PID_timer.reset();
    };

    void CSteeringMotor::_run()
    {
        // Check to see if the PID is active, if not exit the function
        if(!m_pidActive) return;
        // Retrieve the yaw/heading value
        setYaw();
        // Call the PID function to adjust the steering angle
        steerPID();
        return;
    }
}; // namespace hardware::drivers