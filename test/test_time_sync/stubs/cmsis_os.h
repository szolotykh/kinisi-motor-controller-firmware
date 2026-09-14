//------------------------------------------------------------
// File name: cmsis_os.h
// Description: Declare minimal host-test substitutes for cmsis_os.h dependencies.
//------------------------------------------------------------
#pragma once
#include <stdint.h>
#include <stddef.h>
typedef void *osThreadId_t;
typedef int osThreadState_t;
typedef int osPriority_t;
typedef uint32_t TickType_t;
enum { osThreadError = -1, osPriorityNormal = 0 };
typedef struct { const char *name; uint32_t stack_size; osPriority_t priority; } osThreadAttr_t;
/**
 * @brief Report whether the mocked task handle exists.
 */
osThreadState_t osThreadGetState(osThreadId_t thread);
/**
 * @brief Capture the production task entry point and argument without launching a thread.
 */
osThreadId_t osThreadNew(void (*task)(void *), void *arg, const osThreadAttr_t *attr);
/**
 * @brief Provide the fixture's initial RTOS tick value.
 */
TickType_t xTaskGetTickCount(void);
/**
 * @brief Stop the fixture after one complete production odometry-task iteration.
 */
void vTaskDelayUntil(TickType_t *last, TickType_t period);
#define pdMS_TO_TICKS(ms) (ms)
#define portMAX_DELAY UINT32_MAX
