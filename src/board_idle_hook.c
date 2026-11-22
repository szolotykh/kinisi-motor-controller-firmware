// Apply shallow Cortex-M sleep only in the idle task. Interrupts, SysTick
// and peripheral clocks stay live, so normal scheduling and I/O can wake it.
#include "hw_config.h"
#if BOARD_HAS_BOARD_IDLE
#include "connection.h"
void vApplicationIdleHook(void)
{
    if (connection_board_idle() &&
        !(SCB->SCR & (SCB_SCR_SLEEPDEEP_Msk | SCB_SCR_SLEEPONEXIT_Msk))) {
        __DSB();
        __WFI();
        __ISB();
    }
}
#endif
