#pragma once

#include <stdint.h>

typedef void* thread_handle_t;

// OS abstraction interface
typedef struct os_interface {
    // Kernel management
    void (*kernel_init)(void);
    void (*kernel_start)(void);
    
    // Thread management
    thread_handle_t (*create_thread)(void (*thread_func)(void*), const char* name, void* arg);
    void (*delete_thread)(thread_handle_t thread);
    
    // Time management
    void (*delay_ms)(uint32_t ms);
    uint32_t (*get_tick_ms)(void);
} os_interface_t;

// Get the OS interface implementation
const os_interface_t* get_os_interface(void);