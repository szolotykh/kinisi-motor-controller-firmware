//------------------------------------------------------------
// File name: semphr.h
// Description: Declare minimal host-test substitutes for semphr.h dependencies.
//------------------------------------------------------------
#pragma once
#include <stdint.h>
typedef void *SemaphoreHandle_t;
/**
 * @brief Return the fixture mutex token.
 */
SemaphoreHandle_t xSemaphoreCreateMutex(void);
/**
 * @brief Assert exclusive access and mark the fixture mutex locked.
 */
int xSemaphoreTake(SemaphoreHandle_t mutex, uint32_t timeout);
/**
 * @brief Assert mutex ownership and release the fixture lock.
 */
void xSemaphoreGive(SemaphoreHandle_t mutex);
