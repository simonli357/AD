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
    /** \brief  CTaskManager class constructor
     *
     *  Constructor method
     *
     *  @param f_taskList      list of tasks
     *  @param f_taskCount     number of tasks
     *  @param f_baseFreq      base frequency
     */
    CTaskManager::CTaskManager(
            utils::CTask** f_taskList, 
            uint32_t f_taskCount, 
            float f_baseFreq)
        : m_taskList(f_taskList)
        , m_taskCount(f_taskCount) 
    {
        m_ticker.attach(mbed::callback(this,&CTaskManager::timerCallback), f_baseFreq);
    }

    /** \brief  CTaskManager class destructor
     *  
     */
    CTaskManager::~CTaskManager() 
    {
        m_ticker.detach();
    }
    
    /** @brief  The main callback method aims to apply the subtasks' run method. */
    void CTaskManager::mainCallback()
    {
        for(uint32_t i = 0; i < m_taskCount; i++)
        {
            m_taskList[i]->run();
        }
    }

    /** @brief  Timer callback method applies the subtasks' callback function. */
    void CTaskManager::timerCallback()
    {
        for(uint32_t i = 0; i < m_taskCount; i++)
        {
            m_taskList[i]->timerCallback();
        }
    }

}; // namespace utils::task