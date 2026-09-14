//------------------------------------------------------------
// File name: stm32f4xx_hal.h
// Description: Minimal HAL interface for exercising the production I2C transport on a host.
//------------------------------------------------------------
#pragma once
#include <stdint.h>

typedef enum { HAL_OK, HAL_ERROR, HAL_BUSY } HAL_StatusTypeDef;
typedef enum { I2C2_EV_IRQn, I2C2_ER_IRQn, TEST_IRQ_COUNT } IRQn_Type;
typedef struct {
    uint32_t ClockSpeed, DutyCycle, OwnAddress1, AddressingMode, DualAddressMode;
    uint32_t OwnAddress2, GeneralCallMode, NoStretchMode;
} I2C_InitTypeDef;
typedef struct { void *Instance; I2C_InitTypeDef Init; } I2C_HandleTypeDef;
typedef struct { uint32_t Pin, Mode, Pull, Speed, Alternate; } GPIO_InitTypeDef;

#define I2C2 ((void *)(uintptr_t)2)
#define GPIOB ((void *)(uintptr_t)3)
#define GPIO_PIN_10 (1U << 10)
#define GPIO_PIN_11 (1U << 11)
#define GPIO_MODE_AF_OD 1U
#define GPIO_PULLUP 1U
#define GPIO_SPEED_FREQ_VERY_HIGH 3U
#define GPIO_AF4_I2C2 4U
#define I2C_DUTYCYCLE_2 0U
#define I2C_ADDRESSINGMODE_7BIT 0U
#define I2C_DUALADDRESS_DISABLE 0U
#define I2C_GENERALCALL_DISABLE 0U
#define I2C_NOSTRETCH_DISABLE 0U
#define I2C_FIRST_FRAME 1U
#define I2C_LAST_FRAME 2U
#define I2C_DIRECTION_TRANSMIT 0U
#define I2C_DIRECTION_RECEIVE 1U
#define __HAL_RCC_GPIOB_CLK_ENABLE() ((void)0)
#define __HAL_RCC_I2C2_CLK_ENABLE() ((void)0)

/** @brief Read the simulated interrupt mask. */
uint32_t __get_PRIMASK(void);
/** @brief Mask interrupts in the host fixture. */
void __disable_irq(void);
/** @brief Restore the simulated interrupt mask. */
void __set_PRIMASK(uint32_t mask);
/** @brief Record interrupt priorities configured by the transport. */
void HAL_NVIC_SetPriority(IRQn_Type irq, uint32_t priority, uint32_t subpriority);
/** @brief Enable a simulated NVIC interrupt line. */
void HAL_NVIC_EnableIRQ(IRQn_Type irq);
/** @brief Accept GPIO configuration without accessing hardware. */
void HAL_GPIO_Init(void *port, GPIO_InitTypeDef *config);
/** @brief Initialize the mock I2C peripheral. */
HAL_StatusTypeDef HAL_I2C_Init(I2C_HandleTypeDef *handle);
/** @brief Rearm listening after initialization or transaction completion. */
HAL_StatusTypeDef HAL_I2C_EnableListen_IT(I2C_HandleTypeDef *handle);
/** @brief Retain the transmit buffer while a mock master reads its bytes. */
HAL_StatusTypeDef HAL_I2C_Slave_Seq_Transmit_IT(I2C_HandleTypeDef *handle,
    uint8_t *data, uint16_t length, uint32_t options);
/** @brief Accept the receive setup used by the production address callback. */
HAL_StatusTypeDef HAL_I2C_Slave_Seq_Receive_IT(I2C_HandleTypeDef *handle,
    uint8_t *data, uint16_t length, uint32_t options);
/** @brief Model transfer-byte completion separately from the final read NACK. */
void HAL_I2C_EV_IRQHandler(I2C_HandleTypeDef *handle);
/** @brief Model HAL listen completion on final NACK and recovery on bus error. */
void HAL_I2C_ER_IRQHandler(I2C_HandleTypeDef *handle);
