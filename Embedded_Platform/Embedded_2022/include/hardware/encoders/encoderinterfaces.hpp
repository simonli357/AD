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

#ifndef ENDOCER_INTERFACE_HPP
#define ENDOCER_INTERFACE_HPP

#include <stdint.h>
namespace hardware::encoders{
    /**
     * @brief Rotary speed encoder interface class.
     *
     */
    class IEncoderGetter{
        public:
        /**
         * @brief Get the counted impulse in the last period. It can be positive or negative number, when it uses by quadrature encoder.
         * 
         * @return Rotation speed in a period
         */
        virtual int16_t  getCount() = 0;
        /**
         * @brief Get the rotation speed of the encoder based on the predefined period and encoder resolution. The result is returned in rotation per second (rps).
         * 
         * @return Counted value in a period
         */
        virtual float getSpeedRps() = 0;
        /**
         * @brief Get the Getter function for the traveled distance.
         * 
         * @return Counted value in a period
         */
        virtual float getTraveledDistance() = 0;
        /**
         * @brief Start the traveled distance measurement.
         * 
         * @return 
         */
        virtual void startDistMeasure();
        /**
         * @brief Stop the travel distange measurement.
         * 
         * @return 
         */
        virtual void stopDistMeasure();
        
        /**
         * @brief Get the encoder capacity. If it's false, than the encoder can give the orientation like a positive or a negative rotation speed, else it returns only the absolute value of the rotation speed. 
         * 
         * @return Capacity of measuring the rotation direction
         */
        virtual bool isAbs() = 0;
    };


    
    /**
     * @brief Rotary speed encoder interface class for accessing non-filtered values
     * 
     * This interface is used when a filter is attached to the encoder and the initial interface 'IEncoderGetter' returns the filtered values. In this case, this interface ensure the access to the non-filtered values. 
     */
    class IEncoderNonFilteredGetter{
        public:
            /**
             * @brief Get the non-filtered counted impulse in the last period. 
             * 
             * @return Non-filtered counted value.
             */
            virtual int16_t  getNonFilteredCount()=0;
            /**
             * @brief Get the non-filtered rotation speed [rotation per second]
             * 
             * @return Non-filtered rotation speed
             */
            virtual float getNonFilteredSpeedRps() = 0;
    };
}; // namepsace hardware::encoders;

#endif