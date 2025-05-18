#include <hardware/encoders/quadraturecounter.hpp>
#include "mbed.h"
#include "stm32l4xx_hal.h" 

namespace hardware::encoders{

    CQuadratureCounter_TIM4 *CQuadratureCounter_TIM4::m_instance = 0;
    CQuadratureCounter_TIM4::CQuadratureCounter_TIM4_Destroyer CQuadratureCounter_TIM4::m_destroyer;


    /**
     * @brief Setter function for the singelton object.
     * 
     * @param pSingObj - singleton object address
     */
    void CQuadratureCounter_TIM4::CQuadratureCounter_TIM4_Destroyer::SetSingleton(CQuadratureCounter_TIM4* pSingObj){
        m_singleton = pSingObj;
    }  

    /**
     * @brief Destroy the singelton object.
     * 
     */
    CQuadratureCounter_TIM4::CQuadratureCounter_TIM4_Destroyer::~CQuadratureCounter_TIM4_Destroyer(){
        delete m_singleton;
    }


    /**
     * @brief 
     * It verifies the existence of the singleton object. It creates a new instance when it's necessary and return the address of instance.
     * It initializes all parameter by appling method 'initialize'.
     * 
     * @return The address of the singleton object
     */
    CQuadratureCounter_TIM4* CQuadratureCounter_TIM4::Instance(){
        if(!CQuadratureCounter_TIM4::m_instance){
            CQuadratureCounter_TIM4::m_instance = new CQuadratureCounter_TIM4;
            m_instance->initialize();
            CQuadratureCounter_TIM4::m_destroyer.SetSingleton(m_instance);
        }
        return CQuadratureCounter_TIM4::m_instance;
    }


    /**
     * @brief Initialize the parameter of the object.
     * 
     * It configures all register, for timer TIM4 decodes the quadrature encoder signal.
     */
    void CQuadratureCounter_TIM4::initialize()
    {
        /* enable peripheral clocks */
        __HAL_RCC_GPIOB_CLK_ENABLE();
        __HAL_RCC_TIM4_CLK_ENABLE();

        /* --- PB6/PB7 → AF2 (TIM4_CH1 / TIM4_CH2) ------------------------------ */
        GPIO_InitTypeDef gpio{};
        gpio.Pin       = GPIO_PIN_6 | GPIO_PIN_7;
        gpio.Mode      = GPIO_MODE_AF_PP;          // alternate‑function, push‑pull
        gpio.Pull      = GPIO_PULLDOWN;            // matches your old PUPDR setting
        gpio.Speed     = GPIO_SPEED_FREQ_LOW;
        gpio.Alternate = GPIO_AF2_TIM4;            // AF‑mapping for TIM4 on L4S5
        HAL_GPIO_Init(GPIOB, &gpio);

        /* --- TIM4 encoder mode -------------------------------------------------- */
        TIM4->CR1   = TIM_CR1_CEN;                 // enable counter
        TIM4->SMCR  = TIM_ENCODERMODE_TI12;
        TIM4->CCMR1 = 0xF1F1;
        TIM4->CCER  = TIM_CCER_CC1E | TIM_CCER_CC2E;
        TIM4->PSC   = 0;
        TIM4->ARR   = 0xFFFF;
        TIM4->CNT   = 0;
    }

    /**
     * @brief Get the position of encoder. 
     * 
     * It returns the last counted value by the timer. 
     * 
     */
    int16_t CQuadratureCounter_TIM4::getCount(){
        return TIM4->CNT;
    }

    /**
     * @brief Reset the value of the counter to zero value.
     */
    void CQuadratureCounter_TIM4::reset(){
        TIM4->CNT = 0;
    }

}; // namespace hardware::encoders