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

#include <periodics/instantconsumption.hpp>


namespace periodics{
    /** \brief  Class constructor
     *
     *  It initializes the task and the state of the led. 
     *
     *  \param f_period       Toggling period of LED
     *  \param f_led          Digital output line to LED
     */
    CInstantConsumption::CInstantConsumption(
            uint32_t        f_period, 
            mbed::AnalogIn  f_pin, 
            UnbufferedSerial&      f_serial) 
        : utils::CTask(f_period)
        , m_pin(f_pin)
        , m_isActive(true)
        , m_serial(f_serial)
        , m_median(0.0)
    {
    }

    /** @brief  CInstantConsumption class destructor
     */
    CInstantConsumption::~CInstantConsumption()
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
    void CInstantConsumption::InstantPublisherCommand(char const * a, char * b) {
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
    * @brief Computes the average consumption rate per second.
    * 
    * The function is called every 0.2 seconds (5 times per second) to receive new consumption values. After every 5 calls (which corresponds to 1 second), 
    * it returns the average consumption for that second. If it hasn't yet accumulated 5 values, the function returns 0.0.
    * 
    * @param newValue The new consumption value to be added to the accumulation for the current second.
    * @return The average consumption rate for the last second after every 5th call; otherwise, returns 0.0.
    */
    float CInstantConsumption::calculateAverageInstantConsumption(float newValue)
    {
        static int i=0;
        i += 1;

        m_median += newValue;

        if(i==5)
        {
            i = 0;
            return m_median/5.0;
        }
        return 0.0;
    }

    /**
    * @brief Periodically checks and sends the scaled value from the A2 pin over a serial connection.
    * 
    * When the function is active, it reads a 16-bit value from the A2 pin and scales it using the provided scale factor:
    * For a 3.3V signal, the pin reads 65536, and for 1V it reads 19859.39.
    * 
    * After scaling the pin's reading, it calculates the average value using the `calculateMedian` method.
    * 
    * If there's a valid average for the past second, it formats the value and sends it over a serial connection.
    * 
    * Note: Despite the name `calculateMedian`, the function calculates the average.
    */
    void CInstantConsumption::_run()
    {
        if(!m_isActive) return;
        char buffer[256];
        float l_rps = m_pin.read_u16()/19859.39;
    
        float l_median = calculateAverageInstantConsumption(l_rps);

        if(l_median != 0.0)
        {
            snprintf(buffer, sizeof(buffer), "@6:%.3f;;\r\n", l_median*10);
            m_serial.write(buffer,strlen(buffer));
            m_median = 0.0;
        }
    }

}; // namespace periodics