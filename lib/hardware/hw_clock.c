//------------------------------------------------------------
// File name: hw_clock.c
// Description: Read a monotonic microsecond clock from TIM14 and the extended HAL tick.
//------------------------------------------------------------
#include "hw_clock.h"
#include "stm32f4xx_hal.h"

/**
 * @brief Read board monotonic time in microseconds, extending the 32-bit HAL tick.
 * @return Extended milliseconds plus the current TIM14 microsecond fraction.
 * @note Task-context use with the active 1 MHz TIM14/1 ms HAL timebase.
 * Call at least once per HAL tick wrap; IRQs must not span multiple tick overflows.
 */
uint64_t hw_clock_microseconds(void)
{
    static uint32_t previous_ms;
    static uint64_t epoch_ms;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    uint32_t ms = HAL_GetTick();
    uint32_t fraction = TIM14->CNT;
    if (TIM14->SR & TIM_SR_UIF) {
        ++ms; // Overflow occurred but HAL tick interrupt has not run yet.
        fraction = TIM14->CNT;
    }
    if (ms < previous_ms) epoch_ms += 1ULL << 32;
    previous_ms = ms;
    uint64_t result = (epoch_ms + ms) * 1000ULL + fraction;
    __set_PRIMASK(primask);
    return result;
}
