#pragma once
#include <stdint.h>
#include <stddef.h>
typedef enum { HAL_OK, HAL_ERROR } HAL_StatusTypeDef;
typedef enum { GPIO_PIN_RESET, GPIO_PIN_SET } GPIO_PinState;
typedef struct { uint32_t ODR, IDR; } GPIO_TypeDef;
typedef struct { uint32_t compare[4]; } TIM_TypeDef;
typedef struct { uint32_t unused; } SPI_TypeDef;
extern GPIO_TypeDef test_ports[5];
extern TIM_TypeDef test_timers[12];
#define GPIOA (&test_ports[0])
#define GPIOB (&test_ports[1])
#define GPIOC (&test_ports[2])
#define GPIOD (&test_ports[3])
#define GPIOE (&test_ports[4])
#define TIM1 (&test_timers[0])
#define TIM2 (&test_timers[1])
#define TIM3 (&test_timers[2])
#define TIM4 (&test_timers[3])
#define TIM5 (&test_timers[4])
#define TIM8 (&test_timers[7])
#define TIM12 (&test_timers[11])
#define TIM_CHANNEL_1 0
#define TIM_CHANNEL_2 4
#define TIM_CHANNEL_3 8
#define TIM_CHANNEL_4 12
#define HAL_TIM_STATE_READY 1
#define TIM_COUNTERMODE_UP 0
#define TIM_CLOCKDIVISION_DIV1 0
#define TIM_AUTORELOAD_PRELOAD_DISABLE 0
#define TIM_CLOCKSOURCE_INTERNAL 0
#define TIM_TRGO_RESET 0
#define TIM_MASTERSLAVEMODE_DISABLE 0
#define TIM_OCMODE_PWM1 1
#define TIM_OCPOLARITY_HIGH 0
#define TIM_OCNPOLARITY_HIGH 0
#define TIM_OCNPOLARITY_LOW 1
#define TIM_OCFAST_DISABLE 0
#define RCC_HCLK_DIV1 1
#define RCC_HCLK_DIV2 2
#define RCC_HCLK_DIV4 4
typedef struct { uint32_t APB1CLKDivider, APB2CLKDivider; } RCC_ClkInitTypeDef;
uint32_t HAL_RCC_GetPCLK1Freq(void);
uint32_t HAL_RCC_GetPCLK2Freq(void);
void HAL_RCC_GetClockConfig(RCC_ClkInitTypeDef *, uint32_t *);
#define GPIO_MODE_INPUT 0
#define GPIO_MODE_OUTPUT_PP 1
#define GPIO_MODE_AF_PP 2
#define GPIO_NOPULL 0
#define GPIO_SPEED_FREQ_LOW 0
#define GPIO_SPEED_FREQ_HIGH 1
#define GPIO_AF6_SPI3 6
#define SPI3 ((SPI_TypeDef *)3)
#define SPI_MODE_MASTER 1
#define SPI_DIRECTION_2LINES 2
#define SPI_DATASIZE_8BIT 8
#define SPI_POLARITY_LOW 0
#define SPI_PHASE_2EDGE 1
#define SPI_NSS_SOFT 1
#define SPI_BAUDRATEPRESCALER_64 64
#define SPI_FIRSTBIT_MSB 0
#define __HAL_RCC_SPI3_CLK_ENABLE() ((void)0)
#define __HAL_RCC_GPIOC_CLK_ENABLE() ((void)0)
typedef struct {
    uint32_t Prescaler, CounterMode, Period, ClockDivision, AutoReloadPreload;
} TIM_Base_InitTypeDef;
typedef struct {
    TIM_TypeDef *Instance;
    TIM_Base_InitTypeDef Init;
    int State;
    uint32_t started, startedN, noPreload;
    uint32_t polarityN[4];
} TIM_HandleTypeDef;
typedef struct { uint32_t Pin, Mode, Pull, Speed, Alternate; } GPIO_InitTypeDef;
typedef struct { uint32_t OCMode, OCPolarity, OCNPolarity, OCFastMode, Pulse; } TIM_OC_InitTypeDef;
typedef struct { uint32_t ClockSource; } TIM_ClockConfigTypeDef;
typedef struct { uint32_t MasterOutputTrigger, MasterSlaveMode; } TIM_MasterConfigTypeDef;
typedef struct {
    uint32_t Mode, Direction, DataSize, CLKPolarity, CLKPhase, NSS, BaudRatePrescaler, FirstBit;
} SPI_InitTypeDef;
typedef struct { SPI_TypeDef *Instance; SPI_InitTypeDef Init; } SPI_HandleTypeDef;
#define __HAL_TIM_SET_COMPARE(t,c,v) ((t)->Instance->compare[(c)/4]=(v))
#define __HAL_TIM_DISABLE_OCxPRELOAD(t,c) ((t)->noPreload |= 1u << ((c)/4))
void HAL_GPIO_WritePin(GPIO_TypeDef *, uint16_t, GPIO_PinState);
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *, uint16_t);
void HAL_GPIO_Init(GPIO_TypeDef *, GPIO_InitTypeDef *);
void HAL_Delay(uint32_t);
void Error_Handler(void);
int HAL_TIM_Base_GetState(TIM_HandleTypeDef *);
HAL_StatusTypeDef HAL_TIM_Base_Init(TIM_HandleTypeDef *);
HAL_StatusTypeDef HAL_TIM_ConfigClockSource(TIM_HandleTypeDef *, TIM_ClockConfigTypeDef *);
HAL_StatusTypeDef HAL_TIM_PWM_Init(TIM_HandleTypeDef *);
HAL_StatusTypeDef HAL_TIMEx_MasterConfigSynchronization(TIM_HandleTypeDef *, TIM_MasterConfigTypeDef *);
HAL_StatusTypeDef HAL_TIM_PWM_ConfigChannel(TIM_HandleTypeDef *, TIM_OC_InitTypeDef *, uint32_t);
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *, uint32_t);
HAL_StatusTypeDef HAL_TIMEx_PWMN_Start(TIM_HandleTypeDef *, uint32_t);
HAL_StatusTypeDef HAL_SPI_Init(SPI_HandleTypeDef *);
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *, uint8_t *, uint8_t *, uint16_t, uint32_t);
#define GPIO_PIN_0 (1u << 0)
#define GPIO_PIN_1 (1u << 1)
#define GPIO_PIN_2 (1u << 2)
#define GPIO_PIN_3 (1u << 3)
#define GPIO_PIN_4 (1u << 4)
#define GPIO_PIN_5 (1u << 5)
#define GPIO_PIN_6 (1u << 6)
#define GPIO_PIN_7 (1u << 7)
#define GPIO_PIN_8 (1u << 8)
#define GPIO_PIN_9 (1u << 9)
#define GPIO_PIN_10 (1u << 10)
#define GPIO_PIN_11 (1u << 11)
#define GPIO_PIN_12 (1u << 12)
#define GPIO_PIN_13 (1u << 13)
#define GPIO_PIN_14 (1u << 14)
#define GPIO_PIN_15 (1u << 15)
