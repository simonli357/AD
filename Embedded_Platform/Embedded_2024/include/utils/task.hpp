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
#ifndef TASK_HPP
#define TASK_HPP

#include <mbed.h>

namespace utils
{
   /**
    * @brief It aims to the task functionality. The tasks will be applied periodically by the task manager, the period is defined in the contructor. 
    * 
    */
    class CTask
    {
        public:
            /* Constructor */
            CTask(
                uint32_t f_period
            );
            /* Destructor */
            virtual ~CTask();
            /* Run method */
            virtual void run();
            /** @brief  Timer callback */
            void timerCallback();
        protected:
            /** @brief  main application logic - It's a pure function for application logic and has to override in the derivered class to implement the appl.*/
            virtual void _run() = 0;
            /** @brief period of the task */
            const uint32_t m_period;
            /** @brief  ticks */
            uint32_t m_ticks;
            /** @brief  trigger flag */
            bool m_triggered;
    }; // class CTask

}; // namespace utils

#endif // TASK_HPP