//------------------------------------------------------------
// File name: commands_manager.h
// Description: Expose command queues, task startup, and USB disconnect notification.
//------------------------------------------------------------
#pragma once

#include <message_queue.h>
#include <stdint.h>

extern message_queue_t CommandQueue;
extern message_queue_t I2CCommandQueue;

/**
 * @brief Create the command task using the configured operating-system interface.
 */
void commands_manager_start(void);


// IRQ-safe notification; connection cleanup runs on the command task.
/**
 * @brief Flag a USB disconnect for deferred session cleanup.
 * @note IRQ-safe notification; the command task performs queue and clock reset.
 */
void commands_manager_usb_disconnected(void);
