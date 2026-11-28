// Apply shallow Cortex-M sleep only in the idle task. Interrupts, SysTick
// and peripheral clocks stay live, so normal scheduling and I/O can wake it.
#include "board_revision.h"
#if BOARD_HAS_BOARD_IDLE
#include "connection.h"
#include "cmsis_os.h"
#include "stm32f4xx_hal.h"
void vApplicationIdleHook(void)
{
    if (!connection_board_idle()) return;
    // Only the command task changes the idle latch. Serialize its final check
    // with the WFI, leaving interrupts and SysTick enabled throughout the wait.
    vTaskSuspendAll();
    if (connection_board_idle() &&
        !(SCB->SCR & (SCB_SCR_SLEEPDEEP_Msk | SCB_SCR_SLEEPONEXIT_Msk))) {
        __DSB();
        __WFI();
        __ISB();
    }
    (void)xTaskResumeAll();
}
#endif
