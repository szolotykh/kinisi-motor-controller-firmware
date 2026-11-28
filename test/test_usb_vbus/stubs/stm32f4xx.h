#pragma once
#include <stdint.h>
#define GPIOA ((void *)1)
#define GPIO_PIN_9 (1u << 9)
#define GPIO_PIN_SET 1
#define OTG_FS_IRQn 67
uint32_t NVIC_GetEnableIRQ(int irq);
void HAL_NVIC_DisableIRQ(int irq);
void HAL_NVIC_EnableIRQ(int irq);
void HAL_NVIC_ClearPendingIRQ(int irq);
void __DSB(void);
void __ISB(void);
