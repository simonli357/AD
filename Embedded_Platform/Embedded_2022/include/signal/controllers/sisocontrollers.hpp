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

#ifndef SISO_CONTROLLERS_H
#define SISO_CONTROLLERS_H

#include <cstdio>
#include <utils/linalg/linalg.h>
#include <signal/systemmodels/systemmodels.hpp>
#include <mbed.h>

namespace signal{
namespace controllers
{
    namespace siso
    {
        
        /**
         * @brief General interface class for the controller with single input and single output
         * 
         * @tparam T type of the variables (float, double)
         */
        template<class T>
        class IController{
            public:
                virtual T calculateControl(const T&)=0;
                virtual void clear()=0;
        };

        /**
         * @brief It generates a discrete transferfunction for realizing a proportional–integral–derivative controller, which is discretized by the Euler’s method.
         * 
         * @tparam T type of the variables (float, double) 
         */
        template<class T>
        class CPidController:public IController<T>
        {
            public:
                /* Discrete transferfunction type */

                /** @brief Type of the system model for a pid controller */
                
                using CPidSystemmodelType = signal::systemmodels::lti::siso::CDiscreteTransferFunction<T,3,3>;
                
                /* Constructor */
                CPidController(T              f_kp
                              ,T              f_ki
                              ,T              f_kd
                              ,T              f_tf
                              ,T              f_dt);
                CPidController(CPidSystemmodelType f_pid,T f_dt);

                /* Calculate the control signal based the input error. */
                T calculateControl(const T& f_input);
                /* Serial callback implementation */
                void serialCallbackTUNEPIDcommand(char const * a, char * b);

                /* Set to zero the previous values in the transferfunction  */
                void clear();
            private:
                /* Set the controller's parameters. */
                void setController(T              f_kp
                                  ,T              f_ki
                                  ,T              f_kd
                                  ,T              f_tf);

                /* Discrete transferfunction */
                CPidSystemmodelType     m_pidTf;


                /* Sampling time */
                T                 m_dt;

        };
        /* Include function definitions */
        #include "sisocontrollers.tpp"
    }; // namespace siso
}; // namespace controllers   

}; //namespace signal


#endif