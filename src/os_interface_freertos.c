#include "os_interface.h"
#include "cmsis_os.h"

static void freertos_kernel_init(void) {
    osKernelInitialize();
}

static void freertos_kernel_start(void) {
    osKernelStart();
}

static thread_handle_t freertos_create_thread(void (*thread_func)(void*), const char* name, void* arg) {
    osThreadId_t thread = osThreadNew((osThreadFunc_t)thread_func, arg, NULL);
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