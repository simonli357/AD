/**
 * Copyright (c) 2019, Bosch Engineering Center Cluj and BFMC organizers
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:

 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.

 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.

 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.

 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
*/

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
            UnbufferedSerial& f_serial) 
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
    }

}; // namespace periodics