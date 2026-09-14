//------------------------------------------------------------
// File name: stm32f4xx_hal.h
// Description: Declare minimal host-test substitutes for stm32f4xx_hal.h dependencies.
//------------------------------------------------------------
#pragma once
#include <stdint.h>
typedef struct { uint32_t CNT, SR; } test_timer_t;
/**
 * @brief Expose the fake timer, optionally injecting rollover between register reads.
 */
test_timer_t *test_timer(void);
#define TIM14 test_timer()
#define TIM_SR_UIF 1U
/**
 * @brief Return the fake HAL tick and require interrupts to be masked.
 */
uint32_t HAL_GetTick(void);
/**
 * @brief Read the fixture interrupt-mask value.
 */
uint32_t __get_PRIMASK(void);
/**
 * @brief Set the fixture interrupt mask.
 */
void __disable_irq(void);
/**
 * @brief Restore the supplied fixture interrupt-mask value.
 */
void __set_PRIMASK(uint32_t mask);
