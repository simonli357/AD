#ifndef ROBOT_STATE_MACHINE_HPP
#define ROBOT_STATE_MACHINE_HPP
#include <mbed.h>
#include <drivers/speedingmotor.hpp>
#include <drivers/steeringmotor.hpp>
#include <utils/taskmanager.hpp>

namespace brain
{
    /**
     * @brief CRobotStateMachine targets to implement the main state machine to control
     *  movement of robot and provide the interfaces to control functionality, like braking and moving.
     *  The state of robot can change by external signal received from a higher level controller.   
     * 
     */
    class CRobotStateMachine: public utils::CTask
    {
        public:
            /* Constructor */
            CRobotStateMachine(
                uint32_t                      f_period, 
                UnbufferedSerial&             f_serialPort, 
                drivers::ISteeringCommand&    f_steeringControl,
                drivers::ISpeedingCommand&    f_speedingControl
            );
            /* Destructor */
            ~CRobotStateMachine();
            /* Serial callback method for Speed */ 
            void serialCallbackPWMcommand(char const * a, char * b);
            void serialCallbackComputecommand(char const * a, char * b);
            /* Serial callback method for Steering */ 
            void serialCallbackSTEERcommand(char const * a, char * b);

            void serialCallbackPIDcommand(char const * a, char * b);
            void serialCallbackSetcommand(char const * a, char * b);

        private:
            /* Contains the state machine, which control the lower level drivers (motor and steering) based the current state. */
            virtual void _run();
            /* reference to Serial object */
            UnbufferedSerial&                    m_serialPort;
            /* Steering wheel control interface */
            drivers::ISteeringCommand&    m_steeringControl;
            /* Steering wheel control interface */
            drivers::ISpeedingCommand&    m_speedingControl;
            /* State machine state */
            uint8_t                       m_state;
        
    };
};

#endif
