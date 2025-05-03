#include <periodics/totalvoltage.hpp>

namespace periodics{
    /** \brief  Class constructor
     *
     *  It initializes the task and the state of the led. 
     *
     *  \param f_period       Toggling period of LED
     *  \param f_led          Digital output line to LED
     */
    CTotalVoltage::CTotalVoltage(
            uint32_t f_period, 
            mbed::AnalogIn f_pin, 
            BufferedSerial& f_serial) 
        : utils::CTask(f_period)
        , m_pin(f_pin)
        , m_serial(f_serial)
        , m_isActive(true)
        , m_median(0.0)
    {
    }

    /** @brief  CTotalVoltage class destructor
     */
    CTotalVoltage::~CTotalVoltage()
    {
    };

    /** \brief  Serial callback method to activate or deactivate the publisher. 
     * When the received integer value is bigger or equal to 1, then the publisher become 
     * active and send messages, otherwise is deactivated. 
     *
     * @param a                   input received string
     * @param b                   output reponse message
     * 
     */
    void CTotalVoltage::TotalPublisherCommand(char const * a, char * b) {
        int l_isActivate=0;
        uint32_t l_res = sscanf(a,"%d",&l_isActivate);
        if(l_res==1){
            m_isActive=(l_isActivate>=1);
            sprintf(b,"ack;;");
        }else{
            sprintf(b,"sintax error;;");
        }
    }

    /**
    * @brief Periodically reads the battery voltage from A1 pin and sends the scaled value over a serial connection.
    * 
    * When the function is active, it reads a 16-bit value from the pin, which is connected to a battery.
    * The reading is then scaled to represent the actual battery voltage using the provided scale factor: 
    * When the battery voltage is 7.96V, the pin reads a value of 58574.
    * 
    * After obtaining the scaled battery voltage, the function formats this value and sends it over the serial connection.
    */
    void CTotalVoltage::_run()
    {
        if(!m_isActive) return;
        char buffer[256];
        float l_rps = m_pin.read_u16()/7358.54;
        snprintf(buffer, sizeof(buffer), "@5:%.1f;;\r\n", l_rps);
        m_serial.write(buffer,strlen(buffer));
        // debug
        printf("CTotalVoltage::_run: voltage = %.1f\r\n", l_rps);
    }

}; // namespace periodics