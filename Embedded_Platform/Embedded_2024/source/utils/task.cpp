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


#include <utils/taskmanager.hpp>

namespace utils{

    /******************************************************************************/
    /** \brief  CTask class constructor
     *
     *  It initializes the period and other private value of the task. 
     *
     *  @param f_period      execution period
     */
    CTask::CTask(uint32_t f_period) 
        : m_period(f_period)
        , m_ticks(0)
        , m_triggered(false) 
    {
    }

    /** \brief  CTask class destructor
     *
     */
    CTask::~CTask() 
    {
    }

    /** @brief  Timer callback */
    void CTask::timerCallback()
    {
        m_ticks++;
        if (m_ticks >= m_period)
        {
            m_ticks = 0;
            m_triggered = true;
        }
    }

    /** \brief  Run method
     *
     *  It applies the '_run' method, which implements the task's functionality. It has to override in the derived class.  
     *  
     *  
     */
    void CTask::run()
    {
        if (m_triggered)
        {
            m_triggered = false;
            _run();
        }
    }// namespace CTask

}; // namespace utils