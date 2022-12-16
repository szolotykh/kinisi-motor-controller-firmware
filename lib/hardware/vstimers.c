//------------------------------------------------------------
// File name: timers.c
//------------------------------------------------------------

#include "vstimers.h"

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim9;
TIM_HandleTypeDef htim10;
TIM_HandleTypeDef htim11;
TIM_HandleTypeDef htim12;

TIM_HandleTypeDef* get_timer_handeler(TIM_TypeDef * timTypeDef)
{
	if(timTypeDef == TIM1){
		return &htim1;
	}else if(timTypeDef == TIM2){
		return &htim2;
	}else if(timTypeDef == TIM3){
		return &htim3;
	}else if(timTypeDef == TIM4){
		return &htim4;
	}else if(timTypeDef == TIM5){
		return &htim5;
	}else if(timTypeDef == TIM6){
		return &htim6;
	}else if(timTypeDef == TIM7){
		return &htim7;
	}else if(timTypeDef == TIM8){
		return &htim8;
	}else if(timTypeDef == TIM9){
		return &htim9;
	}else if(timTypeDef == TIM10){
		return &htim10;
	}else if(timTypeDef == TIM11){
		return &htim11;
	}else if(timTypeDef == TIM12){
		return &htim12;
	}
	return NULL;
}

uint32_t get_alternate_function_mapping(TIM_TypeDef * timTypeDef)
{
    if(timTypeDef == TIM1){
    	return GPIO_AF1_TIM1;
    }else if(timTypeDef == TIM2){
    	return GPIO_AF1_TIM2;
    }else if(timTypeDef == TIM3){
    	return GPIO_AF2_TIM3;
    }else if(timTypeDef == TIM4){
    	return GPIO_AF2_TIM4;
    }else if(timTypeDef == TIM5){
    	return GPIO_AF2_TIM5;
    }else if(timTypeDef == TIM8){
    	return GPIO_AF3_TIM8;
    }else if(timTypeDef == TIM9){
    	return GPIO_AF3_TIM9;
    }else if(timTypeDef == TIM10){
    	return GPIO_AF3_TIM10;
    }else if(timTypeDef == TIM11){
    	return GPIO_AF3_TIM11;
    }else if(timTypeDef == TIM12){
    	return GPIO_AF9_TIM12;
    }else if(timTypeDef == TIM13){
        return GPIO_AF9_TIM13;
    }else if(timTypeDef == TIM14){
        return GPIO_AF9_TIM14;
    }

    // TODO: Add asser here.
    return 0;
}

void rcc_gpio_clk_enable(GPIO_TypeDef* port){
	if(port == GPIOA){
		__HAL_RCC_GPIOA_CLK_ENABLE();
	}else if (port == GPIOB){
		__HAL_RCC_GPIOB_CLK_ENABLE();
	}else if (port == GPIOC){
		__HAL_RCC_GPIOC_CLK_ENABLE();
	}else if (port == GPIOD){
		__HAL_RCC_GPIOD_CLK_ENABLE();
	}
}

void rcc_tim_clk_enable(TIM_TypeDef* timTypeDef){
	if(timTypeDef == TIM1){
		__HAL_RCC_TIM1_CLK_ENABLE();
	}else if(timTypeDef == TIM2){
		__HAL_RCC_TIM2_CLK_ENABLE();
	}else if(timTypeDef == TIM3){
		__HAL_RCC_TIM3_CLK_ENABLE();
	}else if(timTypeDef == TIM4){
		__HAL_RCC_TIM4_CLK_ENABLE();
	}else if(timTypeDef == TIM5){
		__HAL_RCC_TIM5_CLK_ENABLE();
	}else if(timTypeDef == TIM6){
		__HAL_RCC_TIM6_CLK_ENABLE();
	}else if(timTypeDef == TIM7){
		__HAL_RCC_TIM7_CLK_ENABLE();
	}else if(timTypeDef == TIM8){
		__HAL_RCC_TIM8_CLK_ENABLE();
	}else if(timTypeDef == TIM9){
		__HAL_RCC_TIM9_CLK_ENABLE();
	}else if(timTypeDef == TIM10){
		__HAL_RCC_TIM10_CLK_ENABLE();
	}else if(timTypeDef == TIM11){
		__HAL_RCC_TIM11_CLK_ENABLE();
	}else if(timTypeDef == TIM12){
		__HAL_RCC_TIM12_CLK_ENABLE();
	}
}