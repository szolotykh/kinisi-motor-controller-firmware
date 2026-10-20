#pragma once
#include "../../test_motor_control/stubs/stm32f4xx_hal.h"
#define GPIO_MODE_AF_OD 3
#define GPIO_MODE_OUTPUT_OD 4
#define I2C_DUTYCYCLE_2 0
#define I2C_ADDRESSINGMODE_7BIT 1
#define I2C_DUALADDRESS_DISABLE 0
#define I2C_GENERALCALL_DISABLE 0
#define I2C_NOSTRETCH_DISABLE 0
#define I2C_MEMADD_SIZE_8BIT 1
void test_i2c_clock_enable(void);
#define __HAL_RCC_I2C3_CLK_ENABLE() test_i2c_clock_enable()
typedef struct {
    uint32_t ClockSpeed, DutyCycle, OwnAddress1, AddressingMode;
    uint32_t DualAddressMode, OwnAddress2, GeneralCallMode, NoStretchMode;
} I2C_InitTypeDef;
typedef struct { I2C_TypeDef *Instance; I2C_InitTypeDef Init; } I2C_HandleTypeDef;
HAL_StatusTypeDef HAL_I2C_Init(I2C_HandleTypeDef *);
HAL_StatusTypeDef HAL_I2C_DeInit(I2C_HandleTypeDef *);
HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *, uint16_t, uint16_t, uint16_t, uint8_t *, uint16_t, uint32_t);
HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *, uint16_t, uint16_t, uint16_t, uint8_t *, uint16_t, uint32_t);
uint32_t HAL_GetTick(void);
