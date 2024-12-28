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

/* Inclusion guard */
#ifndef SERIAL_MONITOR_HPP
#define SERIAL_MONITOR_HPP

/* The mbed library */
#include <mbed.h>
/* Header file for the task manager library, which  applies periodically the fun function of it's children*/
#include <utils/taskmanager.hpp>
/* Header file for the queue manager library*/
#include <utils/queue.hpp>

#include <map>
#include <array>
#include <string>
#include <functional>


namespace drivers
{
   /**
    * @brief Class Serial Monitor
    * 
    * It aims to decode the messages received from the other device and redirectionate to other functions the content of message. 
    * For decode it has a predefined structure with a header (key) part and a content part. The key has to be four character, the content is defined by user.
    * The message received has to start with the '#' special character, the responses have the same key and start with "@" character. The special characters 
    * notice the direction of the message. Examples of messages:
    * 
    *   "#KEY1:MESSAGECONTENT;;\r\n"
    * 
    *   "@KEY1:RESPONSECONTANT;;\r\n"
    * 
    * The key differs for each functionalities, so for each callback function.
    */
    class CSerialMonitor : public utils::CTask
    {
        public:
            typedef mbed::Callback<void(char const *, char *)> FCallback;
            typedef std::map<string,FCallback> CSerialSubscriberMap;

            /* Constructor */
            CSerialMonitor(
                UnbufferedSerial& f_serialPort,
                CSerialSubscriberMap f_serialSubscriberMap
            );
            /* Destructor */
            ~CSerialMonitor();
        private:
            /* Rx callback actions */
            void serialRxCallback();
            /* Tx callback actions */
            void serialTxCallback();
            /* Run method */
            virtual void _run();

            /** @brief Serial communication port */
            UnbufferedSerial& m_serialPort;
            /** @brief Rx buffer */
            utils::CQueue<char,255> m_RxBuffer;
            /** @brief Tx buffer */
            utils::CQueue<char,255> m_TxBuffer;
            /** @brief Data buffer */
            array<char,256> m_parseBuffer;
            /** @brief Parse iterator */
            array<char,256>::iterator m_parseIt;
            /** @brief Serial subscriber */
            CSerialSubscriberMap m_serialSubscriberMap;
    }; // class CSerialMonitor

}; // namespace drivers

#endif // SERIAL_MONITOR_HPP
