#include "drivers/velocitycontrolduration.hpp"

namespace drivers{

    CVelocityControlDuration::CVelocityControlDuration(
        uint32_t f_period,
        drivers::ISteeringCommand& f_steeringControl,
        drivers::ISpeedingCommand& f_speedingControl
    )
    : utils::CTask(f_period)
    , m_steeringControl(f_steeringControl)
    , m_speedingControl(f_speedingControl)
    , m_isActive(false)
    , m_ticksRun(0.0)
    {
    };

    CVelocityControlDuration::~CVelocityControlDuration()
    {
    };

    // Command handler for setting speed and steering based on a received message.
    void CVelocityControlDuration::serialCallbackVCDCommand(const char* message, char* response)
    {
        float speed, steer, time;

        int parsed = sscanf(message, "%f;%f;%f", &speed, &time, &steer);

        m_targetTime = time;

        if(parsed == 3 && speed <= 50.0 && speed >= -50.0 && steer <= 23.2 && steer >= -23.2)
        {
            sprintf(response, "ack");

            m_isActive = true;
            m_ticksRun = 0.0;

            m_targetTime = time;

            m_steeringControl.setAngle(steer);
            m_speedingControl.setSpeed(-speed);
        }
        else
        {
            sprintf(response, "something went wrong");
        }
    };

    // Main run function for the task. Controls the speed and steering based on set parameters.
    void CVelocityControlDuration::_run()
    {
        if(!m_isActive) return;

        // If the accumulated ticks exceed the target time, stop the movement and deactivate the task.
        if(m_ticksRun >= m_targetTime+0.1)
        {
            m_speedingControl.setSpeed(0.0);
            m_steeringControl.setAngle(0.0);
            m_isActive = false;
        }
        else
        {
            // Otherwise, increment the tick counter.
            m_ticksRun += 0.1;
        }
    };
}

// TODO: Add your code here
