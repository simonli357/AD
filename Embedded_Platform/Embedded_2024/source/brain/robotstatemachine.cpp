#include <brain/robotstatemachine.hpp>

namespace brain{

    /**
     * @brief CRobotStateMachine Class constructor
     * 
     * @param f_period              period for controller execution in seconds
     * @param f_serialPort          reference to serial communication object
     * @param f_steeringControl     reference to steering motor control interface
     * @param f_speedingControl     reference to brushless motor control interface
     */
    CRobotStateMachine::CRobotStateMachine(
            uint32_t                      f_period,
            BufferedSerial&             f_serialPort,
            drivers::ISteeringCommand&    f_steeringControl,
            drivers::ISpeedingCommand&    f_speedingControl
        ) 
        : utils::CTask(f_period)
        , m_serialPort(f_serialPort)
        , m_steeringControl(f_steeringControl)
        , m_speedingControl(f_speedingControl)
    {
    }

    /** @brief  CRobotStateMachine class destructor
     */
    CRobotStateMachine::~CRobotStateMachine()
    {
    };

    /** \brief  _Run method contains the main application logic, where it controls the lower lever drivers (dc motor and steering) based the given command and state.
     * It has three main states: 
     *  - 1 - move state -> control the motor rotation speed by giving a speed reference, which is then converted to PWM
     *  - 2 - steering state -> trigger the steering of the motor
     *  - 3 - Brake state -> make the motor enter into a brake state.          
     */
    void CRobotStateMachine::_run()
    {   
        switch(m_state)
        {
            // speed state - control the dc motor rotation speed and the steering angle. 
            case 1:
                break;

            // Steering state
            case 2:
                break;

            // Brake state
            case 3:
                break;
        }
    }

    /**
     * @brief Callback function to have direct PWM value input
     *   - Uses same computation for speed
     *   - For speed, takes as input a straight PWM value that it directly writes to the output pin
     */
    void CRobotStateMachine::serialCallbackPWMcommand(char const * a, char * b)
    {
        printf("PWM command: %s\n", a);
        float l_speed;
        float l_angle;
        uint32_t l_res = sscanf(a, "%f:%f", &l_speed, &l_angle);
        if (2 == l_res)
        {

            m_state = 1;

            m_speedingControl.PWMSpeed(l_speed); // Set the reference speed
            m_steeringControl.PWMAngle(l_angle); 

            sprintf(b,a);
        }
        else
        {
            sprintf(b,"syntax error");
        }
    }

    void CRobotStateMachine::serialCallbackPIDcommand(char const *a, char *b) {
        float pid_active;
        float integral = 0.0f;
        float proportional = 0.0f;
        float derivative = 0.0f;

        // Attempt to parse the input string
        uint32_t l_res = sscanf(a, "%f:%f:%f:%f",
                                &pid_active,
                                &proportional,
                                &integral,
                                &derivative);

        if (l_res == 4) {
            if (pid_active == 0) {
                m_steeringControl.m_pidActive = false;
                sprintf(b, "ack -- PID inactive");
            } else if (pid_active == 1) {
                // Activate the PID with the given parameters
                m_steeringControl.setPID(proportional, integral, derivative);
                m_steeringControl.time_elapsed = 0.0f;
                sprintf(b, "ack -- PID active");
            } else {
                // Invalid pid_active value
                sprintf(b, "The PID command is not valid");
            }
        } else if (l_res == 1) {
            if (pid_active == 0) {
                m_steeringControl.m_pidActive = false;
                sprintf(b, "ack -- PID inactive");
            } else if (pid_active == 1) {
                sprintf(b, "syntax error -- missing PID parameters");
            } else {
                sprintf(b, "The PID command is not valid");
            }
        } else if (l_res == 0 || l_res == EOF) {
            sprintf(b, "syntax error -- invalid input");
        } else {
            sprintf(b, "syntax error -- incomplete PID parameters");
        }
    }

    /**
     * Modified function by Malo
     * @brief function to compute the steering angle and speed based on experimentally defined parameteres
     */
    void CRobotStateMachine::serialCallbackComputecommand(char const * a, char * b)
    {
        float l_speed;
        float l_angle;
        float pwmValue = m_steeringControl.zero_default;

        uint32_t l_res = sscanf(a, "%f:%f", &l_speed, &l_angle);
        if (2 == l_res)
        {
            m_state = 1;

            m_speedingControl.CalculateSpeed(l_speed); // Set the reference speed
            pwmValue = m_steeringControl.CalculateAngle(l_angle);
            m_steeringControl.PWMAngle(pwmValue);
            m_steeringControl.m_desiredSteer = l_angle;

            sprintf(b,a);
        }
        else
        {
            sprintf(b,"syntax error");
        }
    }
    /**
     * 
     * Modified function by Malo
     * @brief function to compute the steering angle and speed based on experimentally defined parameteres
     */

    void CRobotStateMachine::serialCallbackSetcommand(char const * a, char * b)
    {
        float l_speed;
        float l_angle;
        float pwmValue = m_steeringControl.zero_default;

        uint32_t l_res = sscanf(a, "%f:%f", &l_speed, &l_angle);
        if (2 == l_res)
        {
            m_state = 1;

            m_speedingControl.CalculateSpeed(l_speed); // Set the reference speed
            m_steeringControl.m_desiredSteer = l_angle; // Set the desired steering angle

            printf(b,a);
        }
        else
        {
            sprintf(b,"syntax error");
        }
    }
    
}; // namespace brain