#include "os_interface.h"
#include "cmsis_os.h"

static void freertos_kernel_init(void) {
    osKernelInitialize();
}

static void freertos_kernel_start(void) {
    osKernelStart();
}

static thread_handle_t freertos_create_thread(void (*thread_func)(void*), const char* name, void* arg) {
    // A NULL attr makes cmsis_os2 fall back to configMINIMAL_STACK_SIZE
    // (128 words = 512 bytes). That is too small for the command-handler task:
    // the GET_MOTOR_CONTROLLER_STATE (RUN) path goes through xSemaphoreTake,
    // builds a 57-byte state struct and calls CDC_Transmit_FS, which overflowed
    // the 512-byte stack and hard-faulted the MCU (stack-overflow checking is
    // off, so it failed silently). Give created threads a comfortable stack.
    const osThreadAttr_t attr = {
        .name = name,
        .stack_size = 2048,
    };
    osThreadId_t thread = osThreadNew((osThreadFunc_t)thread_func, arg, &attr);
    return (thread_handle_t)thread;
}

static void freertos_delete_thread(thread_handle_t thread) {
    osThreadTerminate((osThreadId_t)thread);
}

static void freertos_delay_ms(uint32_t ms) {
    osDelay(ms);
}

static uint32_t freertos_get_tick_ms(void) {
    return osKernelGetTickCount();
}

static const os_interface_t freertos_interface = {
    .kernel_init = freertos_kernel_init,
    .kernel_start = freertos_kernel_start,
    .create_thread = freertos_create_thread,
    .delete_thread = freertos_delete_thread,
    .delay_ms = freertos_delay_ms,
    .get_tick_ms = freertos_get_tick_ms
};

const os_interface_t* get_os_interface(void) {
    return &freertos_interface;
}