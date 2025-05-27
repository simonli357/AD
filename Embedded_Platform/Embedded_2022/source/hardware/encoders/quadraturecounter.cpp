#include <hardware/encoders/quadraturecounter.hpp>
#include "mbed.h"
#include "stm32l4xx_hal.h" 

namespace hardware::encoders{

    CQuadratureCounter_TIM2 *CQuadratureCounter_TIM2::m_instance = 0;
    CQuadratureCounter_TIM2::CQuadratureCounter_TIM2_Destroyer CQuadratureCounter_TIM2::m_destroyer;


    /**
     * @brief Setter function for the singelton object.
     * 
     * @param pSingObj - singleton object address
     */
    void CQuadratureCounter_TIM2::CQuadratureCounter_TIM2_Destroyer::SetSingleton(CQuadratureCounter_TIM2* pSingObj){
        m_singleton = pSingObj;
    }  

    /**
     * @brief Destroy the singelton object.
     * 
     */
    CQuadratureCounter_TIM2::CQuadratureCounter_TIM2_Destroyer::~CQuadratureCounter_TIM2_Destroyer(){
        delete m_singleton;
    }


    /**
     * @brief 
     * It verifies the existence of the singleton object. It creates a new instance when it's necessary and return the address of instance.
     * It initializes all parameter by appling method 'initialize'.
     * 
     * @return The address of the singleton object
     */
    CQuadratureCounter_TIM2* CQuadratureCounter_TIM2::Instance(){
        if(!CQuadratureCounter_TIM2::m_instance){
            CQuadratureCounter_TIM2::m_instance = new CQuadratureCounter_TIM2;
            m_instance->initialize();
            CQuadratureCounter_TIM2::m_destroyer.SetSingleton(m_instance);
        }
        return CQuadratureCounter_TIM2::m_instance;
    }


    /**
     * @brief Initialize the parameter of the object.
     * 
     * It configures all register, for timer TIM2 decodes the quadrature encoder signal.
     */
    void CQuadratureCounter_TIM2::initialize()
    {
        /* enable peripheral clocks */
        __HAL_RCC_GPIOB_CLK_ENABLE();
        __HAL_RCC_TIM2_CLK_ENABLE();

        /* --- PB0/PB1 → AF1 (TIM2_CH1 / TIM2_CH2) ------------------------------ */
        GPIO_InitTypeDef gpio{};
        gpio.Pin       = GPIO_PIN_0 | GPIO_PIN_1;
        gpio.Mode      = GPIO_MODE_AF_PP;          // alternate‑function, push‑pull
        gpio.Pull      = GPIO_PULLDOWN;            // matches your old PUPDR setting
        gpio.Speed     = GPIO_SPEED_FREQ_LOW;
        gpio.Alternate = GPIO_AF1_TIM2;            // AF‑mapping for TIM2 on L4S5
        HAL_GPIO_Init(GPIOB, &gpio);

        /* --- TIM2 encoder mode -------------------------------------------------- */
        TIM2->CR1   = TIM_CR1_CEN;                 // enable counter
        TIM2->SMCR  = TIM_ENCODERMODE_TI12;
        TIM2->CCMR1 = 0xF1F1;
        TIM2->CCER  = TIM_CCER_CC1E | TIM_CCER_CC2E;
        TIM2->PSC   = 0;
        TIM2->ARR   = 0xFFFF;
        TIM2->CNT   = 0;
    }

    /**
     * @brief Get the position of encoder. 
     * 
     * It returns the last counted value by the timer. 
     * 
     */
    int16_t CQuadratureCounter_TIM2::getCount(){
        return TIM2->CNT;
    }

    /**
     * @brief Reset the value of the counter to zero value.
     */
    void CQuadratureCounter_TIM2::reset(){
        TIM2->CNT = 0;
    }

}; // namespace hardware::encoders