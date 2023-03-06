//------------------------------------------------------------
// File name: controllers_manager.h
//------------------------------------------------------------
#pragma once

#include "hw_motor.h"
#include "hw_encoder.h"
#include <pid_controller.h>
#include <cmsis_os.h>
typedef struct controller_info_t
{
    enum {
        STOP,
        RUN,
        STOPPING
    } state;
    motorIndex mIndex;
    encoder_index_t eIndex;
    pid_controller_t controller;
} controller_info_t;

typedef struct controllers_manager
{
    osThreadId_t threadHandler;
} controllers_manager_t;

typedef struct controllers_manager_input
{
    double TargetVelocity[4];
    controller_info_t ControllerInfo[4];
} controllers_manager_input_t;

void controllers_manager_init(controllers_manager_t* controllers_manager, controllers_manager_input_t* controllers_manager_input);

int controllers_manager_is_not_init(const controllers_manager_t* controllers_manager);

