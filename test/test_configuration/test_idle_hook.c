// Run the production RTOS hook against instrumented Cortex-M instructions.
#include <assert.h>
#include <stdbool.h>
#if BOARD_HAS_BOARD_IDLE
static struct { unsigned SCR; } control;
#define SCB (&control)
#define SCB_SCR_SLEEPDEEP_Msk 4u
#define SCB_SCR_SLEEPONEXIT_Msk 2u
static unsigned calls;
#define __DSB() ((void)0)
#define __WFI() (++calls)
#define __ISB() ((void)0)
#include "../../src/board_idle_hook.c"
#endif
void check_idle_hook(bool expected)
{
#if BOARD_HAS_BOARD_IDLE
    unsigned before = calls;
    control.SCR = SCB_SCR_SLEEPDEEP_Msk;
    vApplicationIdleHook();
    assert(calls == before);
    control.SCR = 0;
    vApplicationIdleHook();
    assert(calls == before + expected);
#endif
}
