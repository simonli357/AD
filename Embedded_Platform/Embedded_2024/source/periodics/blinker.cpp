#include <periodics/blinker.hpp>


namespace periodics{
    /** \brief  Class constructor
     *
     *  It initializes the task and the state of the led. 
     *
     *  \param f_period       Toggling period of LED
     *  \param f_led          Digital output line to LED
     */
    CBlinker::CBlinker(
            uint32_t            f_period, 
            mbed::DigitalOut    f_led
        ) 
        : utils::CTask(f_period)
        , m_led(f_led) 
    {
        m_led = 1;
    }

    /** @brief  CBlinker class destructor
     */
    CBlinker::~CBlinker()
    {
    };

    /** \brief  Periodically applied method to change the LED's state
     * 
     */
    void CBlinker::_run()
    {
        printf("[Blinker run] LED state: %d\n", m_led.read());
        m_led = !m_led;
    }

}; // namespace periodics