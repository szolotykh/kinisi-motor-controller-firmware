// Run the production hook with observable Cortex-M instructions and scheduler.
#include "FreeRTOSConfig.h"
#include "connection.h"
#include "stm32f4xx_hal.h"
#include <assert.h>
#include <stdbool.h>
#if configUSE_IDLE_HOOK
static struct { unsigned SCR; } control;
#define SCB (&control)
#define SCB_SCR_SLEEPDEEP_Msk 4u
#define SCB_SCR_SLEEPONEXIT_Msk 2u
static unsigned calls;
static bool suspended, preempt_before_suspend;
void vTaskSuspendAll(void)
{
    assert(!suspended);
    if (preempt_before_suspend) connection_set_board_idle(false, 1000);
    suspended = true;
}
int xTaskResumeAll(void) { assert(suspended); suspended = false; return 0; }
static void interrupt_ends_wfi(void)
{
    assert(suspended && connection_board_idle() && !control.SCR && !__get_PRIMASK());
    ++calls; // Represents a peripheral or tick interrupt returning to the idle task.
}
#define __DSB() ((void)0)
#define __WFI() interrupt_ends_wfi()
#define __ISB() ((void)0)
#include "../../src/board_idle_hook.c"
#endif
void check_idle_hook(bool expected)
{
#if configUSE_IDLE_HOOK
    unsigned before = calls;
    control.SCR = SCB_SCR_SLEEPDEEP_Msk;
    vApplicationIdleHook();
    control.SCR = SCB_SCR_SLEEPONEXIT_Msk;
    vApplicationIdleHook();
    assert(calls == before && !suspended);
    control.SCR = 0;
    vApplicationIdleHook();
    vApplicationIdleHook(); // An interrupt does not logically exit idle.
    assert(calls == before + 2 * expected && !suspended && connection_board_idle() == expected);
#else
    assert(!expected);
#endif
}

// Simulate dispatch of an explicit wake immediately before the idle hook
// suspends task switches. A resumed, active board must skip this wait.
void check_idle_preemption(void)
{
#if configUSE_IDLE_HOOK
    assert(!connection_board_idle());
    connection_set_board_idle(true, 1000);
    unsigned before = calls;
    preempt_before_suspend = true;
    vApplicationIdleHook();
    preempt_before_suspend = false;
    assert(!suspended && !connection_board_idle() && before == calls);
#endif
}
