#pragma once
#include <stdbool.h>
#include "stm32f4xx_hal.h"

// STM32F405 encoder mode TI12: count both direct inputs on their normal
// polarity, unfiltered and undivided. DIR changes legitimately during motion.
static inline bool encoder_timer_sampling_valid(const TIM_TypeDef *t)
{
    const uint32_t cr1=TIM_CR1_CEN|TIM_CR1_OPM|TIM_CR1_CMS|TIM_CR1_CKD|TIM_CR1_ARPE;
    const uint32_t capture=TIM_CCER_CC1E|TIM_CCER_CC2E|TIM_CCER_CC1P|TIM_CCER_CC2P|
                           TIM_CCER_CC1NP|TIM_CCER_CC2NP;
    const uint32_t input=TIM_CCMR1_CC1S|TIM_CCMR1_IC1PSC|TIM_CCMR1_IC1F|
                         TIM_CCMR1_CC2S|TIM_CCMR1_IC2PSC|TIM_CCMR1_IC2F;
    return (t->CR1 & cr1)==TIM_CR1_CEN && (t->SMCR & (TIM_SMCR_SMS|TIM_SMCR_ECE))==TIM_ENCODERMODE_TI12 &&
        (t->CCER & capture)==(TIM_CCER_CC1E|TIM_CCER_CC2E) &&
        (t->CCMR1 & input)==(TIM_CCMR1_CC1S_0|TIM_CCMR1_CC2S_0) && t->ARR==65535 && t->PSC==0;
}
