//------------------------------------------------------------
// File name: odometry_manager.c
//------------------------------------------------------------
#include <odometry_manager.h>
#include <cmsis_os.h>
#include <hw_encoder.h>
#include <math.h>
#include <semphr.h>

#define NUMBER_ENCODERS 4
#define ODOMETRY_UPDATE_INTERVAL 50

// Odometry manager state
typedef struct
{
    uint8_t is_initialized[NUMBER_ENCODERS];
    uint16_t encoder_previous_value[NUMBER_ENCODERS];
    double odometry[NUMBER_ENCODERS]; // in Radians
    osThreadId_t thread_handler;
    uint32_t update_interval; // in ms
    SemaphoreHandle_t odometry_mutex;
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
        if (xSemaphoreTake(state->odometry_mutex, portMAX_DELAY)) {
            // Update odometry
            for(int i = 0; i < NUMBER_ENCODERS; i++)
            {
                if(state->is_initialized[i])
                {
                    uint16_t encoder_value = get_encoder_value(i);
                    double resolution = encoder_get_resolution(i)/ (2 * M_PI); // Ticks per radians
                    double encoder_values_change = encoder_value - state->encoder_previous_value[i];
                    state->odometry[i] = state->odometry[i] + encoder_values_change / resolution;

                    state->encoder_previous_value[i] = encoder_value;
                }
            }
            osDelay(1);
            xSemaphoreGive(state->odometry_mutex);
        }
    }
}

//------------------------------------------------------------
void odometry_manager_initialize()
{
    odometry_manager_state.odometry_mutex = xSemaphoreCreateMutex();

    const osThreadAttr_t CommandsTask_attributes = {
        .name = "CommandsTask",
        .stack_size = 128 * 8,
        .priority = (osPriority_t) osPriorityNormal,
    };
    odometry_manager_state.thread_handler = osThreadNew(odometry_manager_task, &odometry_manager_state, &CommandsTask_attributes);
}

//------------------------------------------------------------
uint8_t odometry_manager_is_not_initialized()
{
    osThreadState_t status = osThreadGetState(odometry_manager_state.thread_handler);
    return status == osThreadError;
}

//------------------------------------------------------------
void encoder_start_odometry(uint8_t encoder_index)
{
    if(odometry_manager_is_not_initialized())
    {
        odometry_manager_initialize();
    }

    if(encoder_is_initialized(encoder_index) == 0)
    {
        return;
    }

    // Obtain odometry mutex
    if (xSemaphoreTake(odometry_manager_state.odometry_mutex, portMAX_DELAY)) {
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
    // Obtain odometry mutex
    if (xSemaphoreTake(odometry_manager_state.odometry_mutex, portMAX_DELAY)) {
        odometry_manager_state.odometry[encoder_index] = 0;
        xSemaphoreGive(odometry_manager_state.odometry_mutex);
    }
}

//------------------------------------------------------------
double encoder_get_odometry(uint8_t encoder_index)
{
    double odometry = 0;
    // Obtain odometry mutex
    if (xSemaphoreTake(odometry_manager_state.odometry_mutex, portMAX_DELAY)) {
        odometry = odometry_manager_state.odometry[encoder_index];
        xSemaphoreGive(odometry_manager_state.odometry_mutex);
    }
    return odometry;
}

//------------------------------------------------------------
void encoder_stop_odometry(uint8_t encoder_index)
{
    // Obtain odometry mutex
    if (xSemaphoreTake(odometry_manager_state.odometry_mutex, portMAX_DELAY)) {
        odometry_manager_state.is_initialized[encoder_index] = 0;
        odometry_manager_state.odometry[encoder_index] = 0;
        odometry_manager_state.encoder_previous_value[encoder_index] = 0;
        xSemaphoreGive(odometry_manager_state.odometry_mutex);
    }
}

//------------------------------------------------------------