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
        if (true)
        {
            // m_triggered = false;
            _run();
        }
    }// namespace CTask

}; // namespace utils