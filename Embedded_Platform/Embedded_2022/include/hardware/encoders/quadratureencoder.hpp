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

#ifndef QUADRATURE_ENCODER_TASK_HPP
#define QUADRATURE_ENCODER_TASK_HPP

#include <hardware/encoders/encoderinterfaces.hpp>
#include <hardware/encoders/quadraturecounter.hpp>
#include <signal/filter/filter.hpp>

#include <rtos.h>

namespace hardware::encoders{

/**
 * @brief It implements a periodic task, which get the value from the counter and reset it to zero.
 * 
 */
class CQuadratureEncoder:public IEncoderGetter{
  public:
      CQuadratureEncoder(float,hardware::encoders::IQuadratureCounter_TIMX*,uint16_t);
      void startTimer();
    virtual void _run();
    virtual int16_t getCount();
    virtual float getSpeedRps();
    virtual float getTraveledDistance();
    virtual void startDistMeasure();
    virtual void stopDistMeasure();
    virtual bool isAbs(){return false;}
  protected:
      /** @brief Counter interface */
      ::hardware::encoders::IQuadratureCounter_TIMX *m_quadraturecounter;
      /** @brief Last counted value */
      int16_t           m_encoderCnt;
      /** @brief counted steps */
      double           m_encoderStps;
      /** @brief counted steps enabled */
      bool           m_encoderStpsEnable;
      /** @brief Sampling period */
      const float       m_taskperiod_s;
      /** @brief Resolution of encoder */
      const uint16_t    m_resolution;
      /** @brief Timer for periodically applying */
      LowPowerTicker m_timer;
};

/**
 * @brief It implements the same functionality than CQuadratureEncoderTask class. Additionally it filters the values. 
 * 
 */
class CQuadratureEncoderWithFilter: public CQuadratureEncoder, public IEncoderNonFilteredGetter{
    public:
      CQuadratureEncoderWithFilter(float,hardware::encoders::IQuadratureCounter_TIMX *, uint16_t,signal::filter::IFilter<float>&);
      
      virtual int16_t getCount();
      virtual float getSpeedRps();
      virtual int16_t  getNonFilteredCount();
      virtual float getNonFilteredSpeedRps();
    protected:
      virtual void _run();
      /** @brief Last filtered counted value */
      double m_encoderCntFiltered;
      /** @brief Filter interface */
      signal::filter::IFilter<float>& m_filter;

};

}; // namespace hardware::encoders

#endif