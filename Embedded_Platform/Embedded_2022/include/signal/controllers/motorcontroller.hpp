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


/* Include guard */

#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP
#include<cmath>
#include<signal/controllers/sisocontrollers.hpp>

#include <hardware/encoders/encoderinterfaces.hpp>
#include <signal/controllers/converters.hpp>

#include <mbed.h>

namespace signal
{
    namespace controllers
    {

   /**
    * @brief It implements a controller with a single input and a single output. It needs an encoder getter interface to get the measured values, a controller 
    * to calculate the control signal. It can be completed with a converter to convert the measaurment unit of the control signal. 
    * 
    */
        class CMotorController
        {
            /* PID controller declaration*/
            template <class T>
            using  ControllerType=siso::IController<T>;

            public:
                /* Construnctor */
                CMotorController(hardware::encoders::IEncoderGetter&    f_encoder
                                ,ControllerType<double>&                f_pid
                                ,signal::controllers::IConverter*       f_converter=NULL
                                ,float                                  f_inf_dist = -3.5
                                ,float                                  f_sup_dist = 3.5
                                ,float                                  f_inf_ref = -225
                                ,float                                  f_sup_ref = 225);
                /* Set controller reference value */
                void setRef(double    f_RefRps);
                /* Set refference for distance */
                void setDist(double    f_RefDist);
                /* Get control value */
                double getSpeed();
                /* Clear PID parameters */
                void clearSpeed();
                /* Clear PID parameters */
                void clearDist();
                /* Control action */
                int8_t control();
                /* Distance action */
                int8_t doneDist();

                bool inRange(double f_RefRps);
                bool distInRange(double f_RefDist);

            private:
                /* PWM onverter */
                double converter(double f_u);

                /* Enconder object reference */
                hardware::encoders::IEncoderGetter&     m_encoder;
                /* PID object reference */
                ControllerType<double>&                 m_pid;
                /* Controller reference */
                double                                  m_RefRps;
                /* Distance reference */
                double                                  m_RefDist;
                /* Control value */
                double                                  m_u;
                /* Converter */
                signal::controllers::IConverter*        m_converter;
                uint8_t                                 m_nrHighPwm;
                /* Maximum High PWM Signal */
                const uint8_t                           m_maxNrHighPwm;


                /* Scaled PWM control signal limits */
                const float                                     m_control_sup;
                const float                                     m_control_inf;
                /* Absolute inferior limit of  reference to inactivate the controller in the case low reference and observation value. */
                const float                                     m_ref_abs_inf;
                /* Absolute inferior limits of measured speed to inactivate the controller in the case low reference and observation value. */
                const float                                     m_mes_abs_inf;
                /* Superior limits of measured speed to deactivate the controller for avoid over accelerating. */
                const float                                     m_mes_abs_sup;
                /* Reference allowed limits */
                const float                                     m_inf_dist;
                const float                                     m_sup_dist;
                const float                                     m_inf_ref;
                const float                                     m_sup_ref;

        };
    }; // namespace controllers

}; // namespace signal

#endif