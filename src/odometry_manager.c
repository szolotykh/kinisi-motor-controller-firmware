//------------------------------------------------------------
// File name: odometry_manager.c
//------------------------------------------------------------
#include <odometry_manager.h>
#include <cmsis_os.h>
#include <hw_encoder.h>
#include <math.h>
#include <semphr.h>
#include <platform.h>
#include <hw_config.h>
#include <platform.h>

#define ODOMETRY_UPDATE_INTERVAL 50

// Odometry manager state
typedef struct
{
    uint8_t is_initialized[NUMBER_ENCODERS];
    uint16_t encoder_previous_value[NUMBER_ENCODERS];
    double odometry[NUMBER_ENCODERS]; // in Radians
    double odometry_change[NUMBER_ENCODERS]; // in Radians
    osThreadId_t thread_handler;
    uint32_t update_interval; // in ms
    SemaphoreHandle_t odometry_mutex;

    // Platform odometry
    platform_odometry_t platform_odometry;
} odometry_manager_state_t;

static odometry_manager_state_t odometry_manager_state = {
    .is_initialized = {0},
    .encoder_previous_value = {0},
    .odometry = {0},
    .thread_handler = NULL,
    .update_interval = ODOMETRY_UPDATE_INTERVAL,
    .odometry_mutex = NULL
};

//------------------------------------------------------------
void odometry_manager_task(void *argument)
{
    odometry_manager_state_t *state = (odometry_manager_state_t *)argument;

    const TickType_t xFrequency = pdMS_TO_TICKS(state->update_interval);
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (1)
    {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        // Obtain odometry mutex
        if (xSemaphoreTake(state->odometry_mutex, portMAX_DELAY))
        {
            // Update odometry for each encoder
            for(int i = 0; i < NUMBER_ENCODERS; i++)
            {
                if(state->is_initialized[i])
                {
                    uint16_t encoder_value = get_encoder_value(i);
                    double resolution = encoder_get_resolution(i)/ (2 * M_PI); // Ticks per radians
                    uint16_t encoder_values_change_raw = encoder_value - state->encoder_previous_value[i];

                    // Check for overflow and adjust
                    double encoder_values_change = encoder_values_change_raw;
                    if (encoder_values_change_raw > 32768) { // Half of UINT16_MAX, detecting large backward movement (underflow)
                        encoder_values_change = -(65536 - encoder_values_change_raw); // Adjust for underflow
                    }

                    state->odometry_change[i] = encoder_values_change / resolution;
                    state->odometry[i] = state->odometry[i] + state->odometry_change[i];

                    state->encoder_previous_value[i] = encoder_value;
                }
            }

            // Update odometry for the platform if it is initialized
            if (platform_is_odometry_enabled())
            {
                uint8_t motor_indexes[NUMBER_MOTORS];
                double velocities[NUMBER_MOTORS];
                for (int i = 0; i < NUMBER_MOTORS; i++)
                {
                    motor_indexes[i] = i;
                    velocities[i] = state->odometry_change[i];
                }
                
                platform_odometry_t odometry_change = platform_update_odometry(motor_indexes, velocities, NUMBER_MOTORS);
                state->platform_odometry.x += odometry_change.x * cosf(state->platform_odometry.t) - odometry_change.y * sinf(state->platform_odometry.t);
                state->platform_odometry.y += odometry_change.x * sinf(state->platform_odometry.t) + odometry_change.y * cosf(state->platform_odometry.t);
                state->platform_odometry.t += odometry_change.t;
            }
            osDelay(1);
            xSemaphoreGive(state->odometry_mutex);
        }
    }
}

//------------------------------------------------------------
uint8_t odometry_manager_is_not_initialized()
{
    osThreadState_t status = osThreadGetState(odometry_manager_state.thread_handler);
    return status == osThreadError;
}

//------------------------------------------------------------
void odometry_manager_initialize()
{
    if(odometry_manager_is_not_initialized())
    {
        odometry_manager_state.odometry_mutex = xSemaphoreCreateMutex();

        const osThreadAttr_t CommandsTask_attributes = {
            .name = "CommandsTask",
            .stack_size = 128 * 8,
            .priority = (osPriority_t) osPriorityNormal,
        };
        odometry_manager_state.thread_handler = osThreadNew(odometry_manager_task, &odometry_manager_state, &CommandsTask_attributes);
    }
}

//------------------------------------------------------------
void encoder_start_odometry(uint8_t encoder_index)
{
    // Initialize odometry manager if it is not initialized
    odometry_manager_initialize();

    if(encoder_is_initialized(encoder_index) == 0)
    {
        return;
    }

    // Obtain odometry mutex
    if (xSemaphoreTake(odometry_manager_state.odometry_mutex, portMAX_DELAY))
    {
        odometry_manager_state.is_initialized[encoder_index] = 1;
        odometry_manager_state.odometry[encoder_index] = 0;
        // TODO: Should set previous value on first run of the update task
        odometry_manager_state.encoder_previous_value[encoder_index] = get_encoder_value(encoder_index);
        xSemaphoreGive(odometry_manager_state.odometry_mutex);
    }
}

//------------------------------------------------------------
void encoder_reset_odometry(uint8_t encoder_index)
{
    // Return if odometry manager is not initialized
    if (odometry_manager_is_not_initialized())
    {
        return;
    }

    // Obtain odometry mutex
    if (xSemaphoreTake(odometry_manager_state.odometry_mutex, portMAX_DELAY))
    {
        odometry_manager_state.odometry[encoder_index] = 0;
        xSemaphoreGive(odometry_manager_state.odometry_mutex);
    }
}

//------------------------------------------------------------
double encoder_get_odometry(uint8_t encoder_index)
{
    double odometry = 0;

    // Check if odometry manager is initialized
    if (!odometry_manager_is_not_initialized())
    {
        // Obtain odometry mutex
        if (xSemaphoreTake(odometry_manager_state.odometry_mutex, portMAX_DELAY))
        {
            odometry = odometry_manager_state.odometry[encoder_index];
            xSemaphoreGive(odometry_manager_state.odometry_mutex);
        }
    }
    
    return odometry;
}

//------------------------------------------------------------
void encoder_stop_odometry(uint8_t encoder_index)
{
    // Return if odometry manager is not initialized
    if (odometry_manager_is_not_initialized())
    {
        return;
    }

    // Obtain odometry mutex
    if (xSemaphoreTake(odometry_manager_state.odometry_mutex, portMAX_DELAY))
    {
        odometry_manager_state.is_initialized[encoder_index] = 0;
        odometry_manager_state.odometry[encoder_index] = 0;
        odometry_manager_state.encoder_previous_value[encoder_index] = 0;
        xSemaphoreGive(odometry_manager_state.odometry_mutex);
    }
}

//------------------------------------------------------------
platform_odometry_t odometry_manager_get_platform_odometry(){
    platform_odometry_t platform_odometry = {0};

    // Check if odometry manager is initialized
    if (!odometry_manager_is_not_initialized())
    {
        // Obtain odometry mutex
        if (xSemaphoreTake(odometry_manager_state.odometry_mutex, portMAX_DELAY))
        {
            platform_odometry = odometry_manager_state.platform_odometry;
            xSemaphoreGive(odometry_manager_state.odometry_mutex);
        }
    }

    return platform_odometry;
}

//------------------------------------------------------------
void odometry_manager_reset_platform_odometry(){
    // Return if odometry manager is not initialized
    if (odometry_manager_is_not_initialized())
    {
        return;
    }

    // Obtain odometry mutex
    if (xSemaphoreTake(odometry_manager_state.odometry_mutex, portMAX_DELAY))
    {
        odometry_manager_state.platform_odometry.x = 0;
        odometry_manager_state.platform_odometry.y = 0;
        odometry_manager_state.platform_odometry.t = 0;
        xSemaphoreGive(odometry_manager_state.odometry_mutex);
    }
}