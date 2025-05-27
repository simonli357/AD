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
   * @brief An interface for quadrature encoder based on timer TIM2. 
   * 
   * It's a singleton class for receiving and decoding the Quadrature signal. 
   * It returns the position of the encoder via method 'getCount' and sets the position to zero by method 'reset'.
   * This counter processes the quadrature signal received on PB6 and PB7 lines. 
   */
  class CQuadratureCounter_TIM2:public IQuadratureCounter_TIMX{
      
      
      /** 
       * @brief It uses to destroy the singleton object.
       */
      class CQuadratureCounter_TIM2_Destroyer{
      
      public:
          CQuadratureCounter_TIM2_Destroyer(){}
          ~CQuadratureCounter_TIM2_Destroyer();
          void SetSingleton(CQuadratureCounter_TIM2* s);
      private:
          CQuadratureCounter_TIM2* m_singleton;
      }; // class CQuadratureCounter_TIM2_Destroyer

    public:
      static CQuadratureCounter_TIM2 *Instance();
      int16_t getCount();
      void reset();
      virtual ~CQuadratureCounter_TIM2() {}
    protected:
      CQuadratureCounter_TIM2() {}
    
    private:
      void initialize();
      static CQuadratureCounter_TIM2* m_instance;
      static CQuadratureCounter_TIM2_Destroyer m_destroyer;
  }; //class CQuadratureCounter_TIM2

};// namespace hardware::encoders


#endif