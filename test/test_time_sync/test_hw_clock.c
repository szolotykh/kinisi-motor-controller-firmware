//------------------------------------------------------------
// File name: test_hw_clock.c
// Description: Verify monotonic clock reads across timer and HAL tick rollover.
//------------------------------------------------------------
#include "hw_clock.h"
#include "stm32f4xx_hal.h"
#include <assert.h>
#include <stdio.h>

static test_timer_t timer;
static uint32_t tick, mask, accesses;
static int overflow_during_read;
/**
 * @brief Expose the fake timer, optionally injecting rollover between register reads.
 */
test_timer_t *test_timer(void)
{
    assert(mask == 1); // Both counter and tick must be sampled with IRQs masked.
    if (++accesses == 2 && overflow_during_read) {
        timer.CNT = 5; timer.SR = TIM_SR_UIF;
    }
    return &timer;
}
/**
 * @brief Return the fake HAL tick and require interrupts to be masked.
 */
uint32_t HAL_GetTick(void) { assert(mask == 1); return tick; }
/**
 * @brief Read the fixture interrupt-mask value.
 */
uint32_t __get_PRIMASK(void) { return mask; }
/**
 * @brief Set the fixture interrupt mask.
 */
void __disable_irq(void) { mask = 1; }
/**
 * @brief Restore the supplied fixture interrupt-mask value.
 */
void __set_PRIMASK(uint32_t value) { mask = value; }
/**
 * @brief Run this file's assertions and return zero when all checks pass.
 */
int main(void)
{
    tick = 12; timer.CNT = 345;
    assert(hw_clock_microseconds() == 12345 && mask == 0);
    timer.CNT = 999; accesses = 0; overflow_during_read = 1;
    assert(hw_clock_microseconds() == 13005 && mask == 0);
    overflow_during_read = 0;
    // HAL has now serviced the pending overflow; time must not double count it.
    tick = 13; timer.SR = 0; timer.CNT = 10;
    assert(hw_clock_microseconds() == 13010);
    tick = UINT32_MAX; timer.CNT = 999;
    assert(hw_clock_microseconds() == (uint64_t)UINT32_MAX * 1000 + 999);
    // Pending overflow exactly at the 32-bit HAL millisecond wrap.
    timer.CNT = 3; timer.SR = TIM_SR_UIF; mask = 1;
    assert(hw_clock_microseconds() == (1ULL << 32) * 1000 + 3 && mask == 1);
    tick = 0; timer.CNT = 9; timer.SR = 0;
    assert(hw_clock_microseconds() == (1ULL << 32) * 1000 + 9);
    puts("Hardware clock counter race, pending IRQ, tick wrap and interrupt-mask restoration passed");
    return 0;
}
