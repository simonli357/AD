#ifndef VELOCITYCONTROLDURATION_HPP
#define VELOCITYCONTROLDURATION_HPP

#include <mbed.h>
#include <utils/task.hpp>
#include <drivers/steeringmotor.hpp>
#include <drivers/speedingmotor.hpp>

namespace drivers{

    class CVelocityControlDuration : public utils::CTask {

        public:
            CVelocityControlDuration(
                uint32_t f_period,
                drivers::ISteeringCommand& f_steeringControl,
                drivers::ISpeedingCommand& f_speedingControl
            );

            ~CVelocityControlDuration();

            void serialCallbackVCDCommand(const char* message, char* response);

        private:

            virtual void _run();

            drivers::ISteeringCommand& m_steeringControl;
            drivers::ISpeedingCommand& m_speedingControl;

            bool m_isActive;

            float m_ticksRun;

            float m_targetTime;
    };
}

#endif // VELOCITYCONTROLDURATION_HPP
