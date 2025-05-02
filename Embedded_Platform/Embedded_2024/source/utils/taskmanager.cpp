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