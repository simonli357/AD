#include <brain/robotstatemachine.hpp>

namespace brain{

    /**
     * @brief CRobotStateMachine Class constructor
     * 
     * @param f_period_sec          period for controller execution in seconds
     * @param f_serialPort          reference to serial communication object
     * @param f_dcMotor               reference to dc motor control interface
     * @param f_steeringControl     reference to steering motor control interface
     * @param f_dcMotorControl             reference to controller object
     */
    CRobotStateMachine::CRobotStateMachine(
            float f_period_sec,
            BufferedSerial& f_serialPort,
            hardware::drivers::IMotorCommand&               f_dcMotor,
            hardware::drivers::ISteeringCommand&            f_steeringControl,
            signal::controllers::CMotorController*          f_dcMotorControl) 
        : m_serialPort(f_serialPort)
        , m_dcMotor(f_dcMotor)
        , m_steeringControl(f_steeringControl)
        , m_period_sec(f_period_sec)
        , m_ispidActivated(false)
        , m_dcMotorControl(f_dcMotorControl)
        , m_timer()

    {
    }
    /** \brief  _Run method contains the main application logic, where it controls the lower lever drivers (dc motor and steering) based the given command and state.
     * It has three state: 
     *  - 1 - move state -> control the motor rotation speed by giving direct a PWM signal or by a pid controller
     *                   -> control the steering angle
     *  - 2 - brake state -> apply a dynamic braking on the motor and control the steering angle.          
     */
    void CRobotStateMachine::_run()
    {   
        switch(m_state)
        {
            // speed state - control the dc motor rotation speed and the steering angle. 
            case 1:
                if(m_ispidActivated && m_dcMotorControl!=NULL) // Check the pid controller 
                {
                    // Calculate control signal and return the controller state. 
                    int8_t l_isCorrect = m_dcMotorControl->control(); 
                    // Check the state of the control method 
                    if( l_isCorrect == -1 ) // High consecutive control signal 
                    {
                        // In this case the encoder is working fine and measures too high speed rotation, than it changes to the braking state.  
                        printf("@1:Too high speed and the encoder working;;\r\n");
                        m_dcMotor.brake();
                        m_dcMotorControl->clearSpeed();
                        m_state = 2;
                        break;
                    }
                    else if (l_isCorrect == -2 ) // High consecutive control signal without observation value. 
                    {
                        // In this case the encoder fails and measures 0 rps, but the control signal had a series high values. 
                        // This part protects the robot to run with high speed, when the encoder doesn't measure correctly or it's broker.
                        printf("@1:Encoder error;;\r\n");
                        m_dcMotor.brake();
                        m_dcMotorControl->clearSpeed();
                        m_state = 2;
                        break;
                    }                    
                    // It's all right and can control the robot. 
                    m_dcMotor.setSpeed(m_dcMotorControl->getSpeed());
                }
                break;

            // Brake state
            case 2:
                if( m_dcMotorControl!=NULL){ 
                    m_dcMotorControl->clearSpeed();
                }
                break;

            // Move state
            case 3:
                if(m_dcMotorControl!=NULL) // Check the pid controller 
                {
                    // Calculate control signal and return the controller state. 
                    int8_t l_isCorrect = m_dcMotorControl->control(); 
                    // Check the state of the control method 
                    if( l_isCorrect == -1 ) // High consecutive control signal 
                    {
                        // In this case the encoder is working fine and measures too high speed rotation, than it changes to the braking state.  
                        printf("@7:Too high speed and the encoder working;;\r\n");
                        m_dcMotor.brake();
                        m_dcMotorControl->clearSpeed();
                        m_state = 2;
                        break;
                    }
                    else if (l_isCorrect == -2 ) // High consecutive control signal without observation value. 
                    {
                        // In this case the encoder fails and measures 0 rps, but the control signal had a series high values. 
                        // This part protects the robot to run with high speed, when the encoder doesn't measure correctly or it's broker.
                        printf("@7:Encoder error;;\r\n");
                        m_dcMotor.brake();
                        m_dcMotorControl->clearSpeed();
                        m_state = 2;
                        break;
                    }

                    int8_t isDist = m_dcMotorControl->doneDist(); 
                    if( isDist == -1 ) // Setted distance done 
                    {
                        printf("@7:The distance is done;;\r\n");
                        m_dcMotor.brake();
                        m_dcMotorControl->clearDist();
                        m_dcMotorControl->clearSpeed();
                        m_state = 2;
                        break;
                    }
                    // It's all right and can control the robot. 
                    m_dcMotor.setSpeed(m_dcMotorControl->getSpeed());
                }
                break;
        }
        
    }

    /** \brief  Serial callback method for speed command
     *
     * Serial callback method setting controller to value received for dc motor control values. 
     * In the case of pid activated, the dc motor control values has to be express in meter per second, otherwise represent the duty cycle of PWM signal in percent. 
     * The steering angle has to express in degree, where the positive values marks the right direction and the negative values noticed the left turning direction.
     *
     * @param a                   string to read data 
     * @param b                   string to write data 
     * 
     */
    void CRobotStateMachine::serialCallbackSPEEDcommand(char const * a, char * b)
    {
        float l_speed;
        uint32_t l_res = sscanf(a,"%f",&l_speed);
        if (1 == l_res)
        {
            if( !m_ispidActivated && !m_dcMotor.inRange(l_speed)){ // Check the received control value
                sprintf(b,"The speed command is too high;;");
                return;
            }
            if( m_ispidActivated && !m_dcMotorControl->inRange(CRobotStateMachine::Mps2Rps(l_speed))){ //Check the received reference value
                sprintf(b,"The speed reference is too high;;");
                return;
            }
            
            if(m_state == 3) {
                m_dcMotorControl->clearDist();
            }
            m_state=1;
            
            if(!m_ispidActivated)
            { // The pid controller is deactivated and the dc motor is controlled by user control signal by giving duty cycle of PWM. 
                m_dcMotor.setSpeed(l_speed);
            }
            else
            {// The pid controller is activated and the dc motor speed is controlled by user control signal by giving the reference speed. 
                m_dcMotorControl->setRef(CRobotStateMachine::Mps2Rps( l_speed )); // Set the reference of dc motor speed
            }

            sprintf(b,"ack;;");
        }
        else
        {
            sprintf(b,"sintax error;;");
        }
    }

    /** \brief  Serial callback actions for brake command
     *
     * This method aims to change the state of controller to brake and sets the steering angle to the received value. 
     *
     * @param a                   string to read data 
     * @param b                   string to write data
     * 
     */
    void CRobotStateMachine::serialCallbackBRAKEcommand(char const * a, char * b)
    {
        float l_angle;
        uint32_t l_res = sscanf(a,"%f",&l_angle);
        if(1 == l_res)
        {
            if( !m_steeringControl.inRange(l_angle)){
                sprintf(b,"The steering angle command is too high;;");
                return;
            }
            m_steeringControl.setAngle(l_angle); // control the steering angle 
            
            if(m_state == 3) {
                m_dcMotorControl->clearDist();
            }
            
            m_state = 2;

            m_dcMotor.brake();
            sprintf(b,"ack;;");           
        }
        else
        {
            sprintf(b,"sintax error;;");
        }
    }

    /** \brief  Serial callback actions for movement
     *
     * This function provides an interface to make the car run for x steps. The function only works if the PID is activated. 
     * When the input string contains non-zero value, then it activates. 
     *
     * @param a                   string to read data 
     * @param b                   string to write data
     * 
     */
    void CRobotStateMachine::serialCallbackMOVEcommand(char const * a, char * b)
    {
        float l_move, l_speed;
        uint32_t l_res = sscanf(a,"%f;%f",&l_move, &l_speed);
        if (2 == l_res)
        {
            if(!m_ispidActivated){ // Check the received speed value
                sprintf(b,"PID not activated;;");
                return;
            }
            if(!m_dcMotorControl->inRange(CRobotStateMachine::Mps2Rps(l_speed))){ //Check the received reference value
                sprintf(b,"The speed reference is too high;;");
                return;
            }
            if(!m_dcMotorControl->distInRange(CRobotStateMachine::m2imp(l_move))){ //Check the received distance value
                sprintf(b,"The distance is too high;;");
                return;
            }
            if(l_speed < 0.0 && l_move > 0.0){ //Check the received distance value
                l_move = -l_move;
            }

            m_state=3;
 
            m_dcMotorControl->setRef(CRobotStateMachine::Mps2Rps( l_speed )); // Set the reference of dc motor speed
            m_dcMotorControl->setDist(CRobotStateMachine::m2imp( l_move )); // Set the reference of dc motor distance

            sprintf(b,"ack;;");
        }
        else
        {
            sprintf(b,"sintax error;;");
        }
    }

    /** \brief  Serial callback method for steering command
     *
     * Serial callback method setting controller to value received for steering angle.
     * The steering angle has to express in degree, where the positive values marks the right direction and the negative values noticed the left turning direction.
     *
     * @param a                   string to read data 
     * @param b                   string to write data 
     * 
     */
    void CRobotStateMachine::serialCallbackSTEERcommand(char const * a, char * b)
    {
        float l_angle;
        uint32_t l_res = sscanf(a,"%f",&l_angle);
        if (1 == l_res)
        {
            if( !m_steeringControl.inRange(l_angle)){ // Check the received steering angle
                sprintf(b,"The steering angle command is too high;;");
                return;
            }

            m_steeringControl.setAngle(l_angle); // control the steering angle 
            sprintf(b,"ack;;");
        }
        else
        {
            sprintf(b,"sintax error;;");
        }
    }

    /** \brief  Serial callback actions for PID activation command
     *
     * This function provides an interface to activate or deactivate the Pid controller. When the input string contains non-zero value, then it activates 
     * the pid functionality and the robot's linear velocity will be controlled in meter per second. When the value is zero, the user directly transmite 
     * the duty cycle of pwm signal to control the motor rotation speed. If the controller wasn't deffined for the motioncontroller object, this functionality 
     * cannot be activated. 
     *
     * @param a                   string to read data 
     * @param b                   string to write data
     * 
     */
    void CRobotStateMachine::serialCallbackACTIVPIDcommand(char const * a, char * b)
    {
        int l_isActivate=0;
        uint32_t l_res = sscanf(a,"%d",&l_isActivate);
        if(l_res==1)
        {   
            if(m_dcMotorControl==NULL){
                sprintf(b,"Control object wans't instances. Cannot be activate pid controller;;");
            }else{
                m_ispidActivated=(l_isActivate>=1);
                // Change to brake state
                m_state = 2;
                m_dcMotor.brake();
                sprintf(b,"ack;;");    
            }
            
        }else
        {
            sprintf(b,"sintax error;;");
        }
    }

    /**
     * @brief Function to convert from linear velocity ( meter per second ) of robot to angular velocity ( rotation per second ) of motor.
     * 
     * @param f_vel_mps linear velocity of robot
     * @return float angular velocity of motor
     */
    float CRobotStateMachine::Mps2Rps(float f_vel_mps){
        return f_vel_mps * 150.0;
    }

    /**
     * @brief Function to convert from meters to int.
     * 
     * @param f_meters linear velocity of robot
     * @return int
     */
    float CRobotStateMachine::m2imp(float f_meters){ //310299 impulses in 1 meter of navigation
        return f_meters * 310299;
    }

    /**
     * @brief Start periodically the _run method. 
     * 
     */
    void CRobotStateMachine::startTimer(){
        this->m_timer.attach(mbed::callback(this,&CRobotStateMachine::_run), m_period_sec);
    }

}; // namespace brain