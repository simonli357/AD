#ifndef TASK_MANAGER_HPP
#define TASK_MANAGER_HPP

#include <mbed.h>
#include <utils/task.hpp>

namespace utils
{
   /**
    * @brief It aims to implement the task manager functionality. It controls and applies periodically each task. 
    * It has two main part, a ticker and the mainCallback method. The ticker method applies automatically 'timerCallback' method of each tasks, so
    * numerate separately the ticks from the functionalities of tasks. The mainCallback method aims to apply the application logic for each tasks, 
    * if the task's trigger flag has true state. 
    */
    class CTaskManager
    {
        public:
            CTaskManager(
                utils::CTask** f_taskList, 
                uint32_t f_taskCount, 
                float f_baseFreq
            );
            virtual ~CTaskManager();
            /** @brief  The main callback method aims to apply the subtasks' run method. */
            void mainCallback();
            /** @brief  Timer callback method applies the subtasks' callback function. */
            void timerCallback();
        private:
            /** @brief  List of tasks  */
            utils::CTask** m_taskList;
            /** @brief  number of tasks */
            uint32_t m_taskCount;
            /** @brief  Ticker for periodic applying the timer callback function  */
            Ticker m_ticker;
    }; // class CTaskManager

};

#endif