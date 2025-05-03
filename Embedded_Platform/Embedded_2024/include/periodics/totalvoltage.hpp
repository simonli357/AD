#ifndef TOTALVOLTAGE_HPP
#define TOTALVOLTAGE_HPP

#include <mbed.h>
#include <utils/task.hpp>

namespace periodics
{
   /**
    * @brief It is used for toggling a LED.
    * 
    */
    class CTotalVoltage : public utils::CTask
    {
        public:
            /* Construnctor */
            CTotalVoltage(
                uint32_t f_period,
                mbed::AnalogIn f_pin, 
                BufferedSerial& f_serial
            );
            /* Destructor */
            ~CTotalVoltage();
            /* Serial callback implementation */
            void TotalPublisherCommand(char const * a, char * b);
        private:
            /* Run method */
            virtual void        _run();
            /* ADC input pin for instand consume */
            mbed::AnalogIn      m_pin;  
            /* @brief Serial communication obj.  */
            BufferedSerial&          m_serial;
            /** @brief Active flag  */
            bool                m_isActive;

            float m_median;
    };
};

#endif // TOTALVOLTAGE_HPP