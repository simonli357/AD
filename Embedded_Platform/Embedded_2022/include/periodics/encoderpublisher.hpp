#ifndef SENSOR_TASK_HPP
#define SENSOR_TASK_HPP

/* Standard C library for basic mathematical operations */
#include <math.h>
#include <utils/taskmanager/taskmanager.hpp>
#include <hardware/drivers/dcmotor.hpp>
#include <utils/serial/serialmonitor.hpp>
#include <signal/controllers/motorcontroller.hpp>
#include <signal/systemmodels/systemmodels.hpp>

#include <hardware/encoders/encoderinterfaces.hpp>

namespace periodics
{
    /**
    * @brief CEncoderPublisher class is subclass of utils::task::CTask, a class to publish periodically the encoder values. 
    * 
    */
    class CEncoderPublisher:public utils::task::CTask
    {
        public:
            /* Constructor */
            CEncoderPublisher(uint32_t            f_period
                        ,hardware::encoders::IEncoderGetter&    f_encoder
                        ,BufferedSerial&            f_serial);
            /* Serial callback implementation */
            void serialCallbackENCODERPUBcommand(char const * a, char * b);
        private:
            
            /* Run method */
            void _run();

            /** @brief Active flag  */
            bool                m_isActive;
            /** @brief Encoder getter interface  */
            hardware::encoders::IEncoderGetter&     m_encoder;
            /** @brief Serial communication obj.  */
            BufferedSerial&             m_serial;
    };
}; // namespace periodics


#endif