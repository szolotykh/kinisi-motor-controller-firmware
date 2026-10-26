#pragma once
#include "../../test_motor_control/stubs/stm32f4xx_hal.h"
#define GPIO_MODE_ANALOG 20
#define DISABLE 0
#define ADC1 ((void *)1)
#define ADC_CLOCK_SYNC_PCLK_DIV4 4
#define ADC_RESOLUTION_12B 12
#define ADC_EXTERNALTRIGCONVEDGE_NONE 0
#define ADC_SOFTWARE_START 0
#define ADC_DATAALIGN_RIGHT 0
#define ADC_EOC_SINGLE_CONV 1
#define ADC_SAMPLETIME_480CYCLES 480
#define __HAL_RCC_ADC1_CLK_ENABLE() test_adc_clock_enable()
void test_adc_clock_enable(void);
typedef struct {
    uint32_t ClockPrescaler, Resolution, ScanConvMode, ContinuousConvMode;
    uint32_t DiscontinuousConvMode, NbrOfDiscConversion, ExternalTrigConvEdge;
    uint32_t ExternalTrigConv, DataAlign, NbrOfConversion, DMAContinuousRequests, EOCSelection;
} ADC_InitTypeDef;
typedef struct { void *Instance; ADC_InitTypeDef Init; } ADC_HandleTypeDef;
typedef struct { uint32_t Channel, Rank, SamplingTime; } ADC_ChannelConfTypeDef;
HAL_StatusTypeDef HAL_ADC_Init(ADC_HandleTypeDef *);
HAL_StatusTypeDef HAL_ADC_DeInit(ADC_HandleTypeDef *);
HAL_StatusTypeDef HAL_ADC_ConfigChannel(ADC_HandleTypeDef *, ADC_ChannelConfTypeDef *);
HAL_StatusTypeDef HAL_ADC_Start(ADC_HandleTypeDef *);
HAL_StatusTypeDef HAL_ADC_Stop(ADC_HandleTypeDef *);
HAL_StatusTypeDef HAL_ADC_PollForConversion(ADC_HandleTypeDef *, uint32_t);
uint32_t HAL_ADC_GetValue(ADC_HandleTypeDef *);
