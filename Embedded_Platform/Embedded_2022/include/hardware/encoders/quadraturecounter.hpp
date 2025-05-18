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

#ifndef QUADRATURE_COUNTER__HPP
#define QUADRATURE_COUNTER__HPP

#include <mbed.h>

namespace hardware::encoders{
  /**
   * @brief  An interface to access the main functionality of timer for decoding a quadrature encoder. It's used to get and to reset the measured position of the encoder.
   * 
   */
  class IQuadratureCounter_TIMX{
      public:
        virtual int16_t getCount() = 0;
        virtual void reset() = 0;
  }; // class IQuadratureCounter_TIMX

  /**
   * @brief An interface for quadrature encoder based on timer TIM4. 
   * 
   * It's a singleton class for receiving and decoding the Quadrature signal. 
   * It returns the position of the encoder via method 'getCount' and sets the position to zero by method 'reset'.
   * This counter processes the quadrature signal received on PB6 and PB7 lines. 
   */
  class CQuadratureCounter_TIM4:public IQuadratureCounter_TIMX{
      
      
      /** 
       * @brief It uses to destroy the singleton object.
       */
      class CQuadratureCounter_TIM4_Destroyer{
      
      public:
          CQuadratureCounter_TIM4_Destroyer(){}
          ~CQuadratureCounter_TIM4_Destroyer();
          void SetSingleton(CQuadratureCounter_TIM4* s);
      private:
          CQuadratureCounter_TIM4* m_singleton;
      }; // class CQuadratureCounter_TIM4_Destroyer

    public:
      static CQuadratureCounter_TIM4 *Instance();
      int16_t getCount();
      void reset();
      virtual ~CQuadratureCounter_TIM4() {}
    protected:
      CQuadratureCounter_TIM4() {}
    
    private:
      void initialize();
      static CQuadratureCounter_TIM4* m_instance;
      static CQuadratureCounter_TIM4_Destroyer m_destroyer;
  }; //class CQuadratureCounter_TIM4

};// namespace hardware::encoders


#endif