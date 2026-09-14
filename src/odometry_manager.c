//------------------------------------------------------------
// File name: odometry_manager.c
// Description: Integrate encoder samples and store odometry with its acquisition timestamp.
//------------------------------------------------------------
#include <odometry_manager.h>
#include <cmsis_os.h>
#include <hw_encoder.h>
#include <math.h>
#include <semphr.h>
#include <platform.h>
#include <hw_config.h>
#include <platform.h>
#include <odometry_integrator.h>
#include <loop_frequency.h>
#include <hw_clock.h>
#include <protocol.h>

// Default odometry-task period in ms. 20 ms = 50 Hz: fast enough for good pose
// integration while driving+turning and to match a typical ROS EKF/Nav2 odom
// rate. Runtime-configurable via SET_ODOMETRY_FREQUENCY.
#define ODOMETRY_UPDATE_INTERVAL 20

// Add near the top of the file
static const hw_encoder_interface_t* encoder = NULL;

// Odometry manager state
typedef struct
{
    uint8_t is_initialized[NUMBER_ENCODERS];
    uint16_t encoder_previous_value[NUMBER_ENCODERS];
    double odometry[NUMBER_ENCODERS]; // in Radians
    double odometry_change[NUMBER_ENCODERS]; // in Radians
    uint64_t encoder_sample_us[NUMBER_ENCODERS];
    uint64_t platform_sample_us;
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
/**
 * @brief Acquire encoder batches, integrate odometry, and store timestamps under one mutex.
 */
void odometry_manager_task(void *argument)
{
    odometry_manager_state_t *state = (odometry_manager_state_t *)argument;

    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (1)
    {
        // Recompute the period each iteration so SET_ODOMETRY_FREQUENCY takes
        // effect at runtime (the interval is a shared state variable).
        const TickType_t xFrequency = pdMS_TO_TICKS(state->update_interval);
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        // Obtain odometry mutex
        if (xSemaphoreTake(state->odometry_mutex, portMAX_DELAY))
        {
            // Timestamp the encoder acquisition batch, before integration.
            uint64_t sample_us = hw_clock_microseconds();
            // Update odometry for each encoder
            for(int i = 0; i < NUMBER_ENCODERS; i++)
            {
                if(state->is_initialized[i])
                {
                    uint16_t encoder_value = encoder->get_value(i);
                    state->odometry_change[i] = odometry_integrator_wheel_delta(
                        state->encoder_previous_value[i], encoder_value, encoder->get_resolution(i));
                    state->odometry[i] = state->odometry[i] + state->odometry_change[i];

                    state->encoder_previous_value[i] = encoder_value;
                    state->encoder_sample_us[i] = sample_us;
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

                // odometry_change is expressed in the ROBOT BODY frame. Rotate
                // it into the world frame by the current heading and accumulate
                // (see odometry_integrator_accumulate). Without this, driving
                // while turning corrupts x/y (a small circle unrolls into
                // metres of phantom translation).
                state->platform_odometry = odometry_integrator_accumulate(
                    state->platform_odometry, odometry_change);
                state->platform_sample_us = sample_us;
            }
            xSemaphoreGive(state->odometry_mutex);
        }
    }
}

//------------------------------------------------------------
/**
 * @brief Check whether the odometry task has not been created.
 */
uint8_t odometry_manager_is_not_initialized()
{
    osThreadState_t status = osThreadGetState(odometry_manager_state.thread_handler);
    return status == osThreadError;
}

//------------------------------------------------------------
/**
 * @brief Create the odometry mutex/task on first use and bind the encoder interface.
 */
void odometry_manager_initialize()
{
    if(odometry_manager_is_not_initialized())
    {
        encoder = get_encoder_interface();
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
/**
 * @brief Start accumulation from the current counter and invalidate the prior encoder sample.
 */
void encoder_start_odometry(uint8_t encoder_index)
{
    // Initialize odometry manager if it is not initialized
    odometry_manager_initialize();

    if(encoder->is_initialized(encoder_index) == 0)
    {
        return;
    }

    // Obtain odometry mutex
    if (xSemaphoreTake(odometry_manager_state.odometry_mutex, portMAX_DELAY))
    {
        odometry_manager_state.is_initialized[encoder_index] = 1;
        odometry_manager_state.odometry[encoder_index] = 0;
        odometry_manager_state.encoder_sample_us[encoder_index] = 0;
        // TODO: Should set previous value on first run of the update task
        odometry_manager_state.encoder_previous_value[encoder_index] = encoder->get_value(encoder_index);
        xSemaphoreGive(odometry_manager_state.odometry_mutex);
    }
}

//------------------------------------------------------------
/**
 * @brief Zero the selected encoder angle and invalidate its cached sample timestamp.
 */
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
        odometry_manager_state.encoder_sample_us[encoder_index] = 0;
        xSemaphoreGive(odometry_manager_state.odometry_mutex);
    }
}

//------------------------------------------------------------
/**
 * @brief Read the cached encoder angle in radians under the odometry mutex.
 */
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
/**
 * @brief Disable the selected encoder accumulator and clear its angle and sample timestamp.
 */
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
        odometry_manager_state.encoder_sample_us[encoder_index] = 0;
        odometry_manager_state.encoder_previous_value[encoder_index] = 0;
        xSemaphoreGive(odometry_manager_state.odometry_mutex);
    }
}

//------------------------------------------------------------
/**
 * @brief Read the cached platform pose under the odometry mutex.
 */
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
/**
 * @brief Zero accumulated platform pose and invalidate its cached sample timestamp.
 */
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
        odometry_manager_state.platform_sample_us = 0;
        xSemaphoreGive(odometry_manager_state.odometry_mutex);
    }
}

//------------------------------------------------------------
// Set the global odometry-task frequency (Hz), quantized to the 1 ms tick.
// No-op if frequency_hz is 0.
/**
 * @brief Set the global odometry update frequency, quantized to the millisecond task tick.
 */
void odometry_manager_set_frequency(uint16_t frequency_hz)
{
    // Clamp to the supported range; 0 stays 0 (invalid) and is ignored below.
    frequency_hz = loop_frequency_clamp_hz(frequency_hz);
    if (frequency_hz == 0)
    {
        return; // Ignore invalid frequency
    }

    // Convert to the RTOS tick period (quantized to the 1 ms tick).
    uint32_t period_ms = loop_frequency_hz_to_period_ms(frequency_hz);

    // If the task/mutex has not been created yet, just record the interval; the
    // task reads it when it starts.
    if (odometry_manager_is_not_initialized())
    {
        odometry_manager_state.update_interval = period_ms;
        return;
    }

    if (xSemaphoreTake(odometry_manager_state.odometry_mutex, portMAX_DELAY))
    {
        odometry_manager_state.update_interval = period_ms;
        xSemaphoreGive(odometry_manager_state.odometry_mutex);
    }
}

//------------------------------------------------------------
// Get the current global odometry-task frequency in Hz.
/**
 * @brief Return the effective odometry task frequency in hertz.
 */
uint16_t odometry_manager_get_frequency()
{
    return loop_period_ms_to_frequency_hz(odometry_manager_state.update_interval);
}

// Copy the value and its acquisition time under the same lock.
/**
 * @brief Copy an encoder angle and its acquisition time under the odometry mutex.
 * @param index Encoder index; must be within the configured encoder count.
 * @param angle Receives the cached angle in radians after acquiring the mutex.
 * @param sample_us Receives the cached monotonic acquisition time in microseconds.
 * @return RESPONSE_OK, INVALID_ARGUMENT, ODOMETRY_NOT_INITIALIZED, SAMPLE_NOT_AVAILABLE, or INTERNAL_ERROR.
 * @note Output values are a valid measurement only when RESPONSE_OK is returned.
 */
uint8_t encoder_get_odometry_sample(uint8_t index, double *angle, uint64_t *sample_us)
{
    if (index >= NUMBER_ENCODERS) return RESPONSE_INVALID_ARGUMENT;
    if (!odometry_manager_state.odometry_mutex) return RESPONSE_ODOMETRY_NOT_INITIALIZED;
    if (!xSemaphoreTake(odometry_manager_state.odometry_mutex, portMAX_DELAY)) return RESPONSE_INTERNAL_ERROR;
    uint8_t error = !odometry_manager_state.is_initialized[index] ? RESPONSE_ODOMETRY_NOT_INITIALIZED :
        !odometry_manager_state.encoder_sample_us[index] ? RESPONSE_SAMPLE_NOT_AVAILABLE : RESPONSE_OK;
    *angle = odometry_manager_state.odometry[index];
    *sample_us = odometry_manager_state.encoder_sample_us[index];
    xSemaphoreGive(odometry_manager_state.odometry_mutex);
    return error;
}

/**
 * @brief Copy the cached platform pose and acquisition time under the odometry mutex.
 * @param pose Receives position in meters and heading in radians.
 * @param sample_us Receives monotonic acquisition time in microseconds.
 * @return RESPONSE_OK, ODOMETRY_NOT_INITIALIZED, SAMPLE_NOT_AVAILABLE, or INTERNAL_ERROR.
 * @note Output values are a valid measurement only when RESPONSE_OK is returned.
 */
uint8_t odometry_manager_get_platform_sample(platform_odometry_t *pose, uint64_t *sample_us)
{
    if (!odometry_manager_state.odometry_mutex) return RESPONSE_ODOMETRY_NOT_INITIALIZED;
    if (!xSemaphoreTake(odometry_manager_state.odometry_mutex, portMAX_DELAY)) return RESPONSE_INTERNAL_ERROR;
    uint8_t error = !platform_is_odometry_enabled() ? RESPONSE_ODOMETRY_NOT_INITIALIZED :
        !odometry_manager_state.platform_sample_us ? RESPONSE_SAMPLE_NOT_AVAILABLE : RESPONSE_OK;
    *pose = odometry_manager_state.platform_odometry;
    *sample_us = odometry_manager_state.platform_sample_us;
    xSemaphoreGive(odometry_manager_state.odometry_mutex);
    return error;
}

/**
 * @brief Clear the platform sample timestamp while preserving accumulated pose.
 * @note Restart uses this to require a fresh sample; no-op before manager creation.
 */
void odometry_manager_invalidate_platform_sample(void)
{
    if (!odometry_manager_state.odometry_mutex) return;
    if (xSemaphoreTake(odometry_manager_state.odometry_mutex, portMAX_DELAY)) {
        odometry_manager_state.platform_sample_us = 0;
        xSemaphoreGive(odometry_manager_state.odometry_mutex);
    }
}
