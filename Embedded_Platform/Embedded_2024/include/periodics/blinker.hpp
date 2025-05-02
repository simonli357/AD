#ifndef BLINKER_HPP
#define BLINKER_HPP

/* The mbed library */
#include <mbed.h>
/* Header file for the task manager library, which  applies periodically the fun function of it's children*/
#include <utils/task.hpp>


namespace periodics
{
   /**
    * @brief It is used for toggling a LED.
    * 
    */
    class CBlinker : public utils::CTask
    {
        public:
            /* Construnctor */
            CBlinker(
                uint32_t            f_period, 
                mbed::DigitalOut    f_led
            );
            /* Destructor */
            ~CBlinker();
        private:
            /* Run method */
            virtual void        _run();
            /* Digital output line connected to a LED */
            mbed::DigitalOut    m_led;    
    }; // class CBlinker
}; // namespace periodics

#endif // BLINKER_HPP