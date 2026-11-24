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
#include "platform_fusion.h"
#include "feedback_source.h"

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
#if BOARD_HAS_FEEDBACK_SOURCE_SELECTOR
    // Split long odometry waits for passive shaft monitoring. Only a completed
    // full period samples odometry and invokes either estimator.
    uint32_t phase_ms, remaining_ms;
    feedback_frame_t feedback_frame;
    uint32_t feedback_sequence;
    bool feedback_valid;
    uint8_t feedback_reason;
    platform_odometry_t feedback_twist;
#endif
    osThreadId_t thread_handler;
    uint32_t update_interval; // in ms
    SemaphoreHandle_t odometry_mutex;

    // Platform odometry
    platform_odometry_t platform_odometry;
} odometry_manager_state_t;

#if BOARD_HAS_FUSION_EXPERIMENT
typedef struct { uint64_t seen, quiet_since, moved; uint16_t count; } rest_sample_t;
static rest_sample_t rest[NUMBER_ENCODERS];

static bool record_count(rest_sample_t *r, uint16_t count, uint64_t at)
{
    bool moved = r->seen && count != r->count;
    if (moved) r->moved = at;
    if (moved || !r->seen || at <= r->seen || at - r->seen > 60000) r->quiet_since = at;
    r->count = count;
    r->seen = at;
    return moved;
}
#if BOARD_HAS_FEEDBACK_SOURCE_SELECTOR
static rest_sample_t switch_rest[NUMBER_ENCODERS];
static void sample_switch_rest(uint64_t at)
{
    for (uint8_t i = 0; i < NUMBER_ENCODERS; ++i) {
        if (encoder->is_initialized(i)) record_count(&switch_rest[i],encoder->get_value(i),at);
        else switch_rest[i] = (rest_sample_t){0};
    }
}
#endif
#endif

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
        uint32_t wait_ms = state->update_interval;
#if BOARD_HAS_FEEDBACK_SOURCE_SELECTOR
        if (!state->remaining_ms || state->phase_ms != wait_ms) {
            state->phase_ms = wait_ms;
            state->remaining_ms = wait_ms;
        }
        wait_ms = state->remaining_ms > 20 ? 20 : state->remaining_ms;
#endif
        const TickType_t xFrequency = pdMS_TO_TICKS(wait_ms);
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        // Obtain odometry mutex
        if (xSemaphoreTake(state->odometry_mutex, portMAX_DELAY))
        {
            // Timestamp the encoder acquisition batch, before integration.
            uint64_t sample_us = hw_clock_microseconds();
#if BOARD_HAS_FEEDBACK_SOURCE_SELECTOR
            state->remaining_ms -= wait_ms;
            if (state->remaining_ms) {
                sample_switch_rest(sample_us);
                xSemaphoreGive(state->odometry_mutex);
                continue;
            }
#endif
#if BOARD_HAS_FUSION_EXPERIMENT
            const uint32_t epoch = platform_fusion_epoch();
#if BOARD_HAS_FEEDBACK_SOURCE_SELECTOR
            const uint32_t passive_epoch = passive_fusion_epoch();
            const bool watch_standalone = epoch || passive_epoch;
            const bool observe_wheels = true; // Required for the encoder selector's validity.
#else
            const bool observe_wheels = epoch != 0;
            const bool watch_standalone = observe_wheels;
#endif
            bool valid = platform_is_odometry_enabled();
            bool stationary = true, moved = false;
            platform_odometry_t delta = {0};
            uint64_t previous = state->platform_sample_us;
#endif
            // Update odometry for each encoder
            for(int i = 0; i < NUMBER_ENCODERS; i++)
            {
#if BOARD_HAS_FUSION_EXPERIMENT
                if (observe_wheels && platform_owns_motor(i) && (!state->is_initialized[i] ||
                    !encoder->is_initialized(i) || !isfinite(encoder->get_resolution(i)) ||
                    encoder->get_resolution(i) <= 0)) valid = false;
#endif
                if(state->is_initialized[i])
                {
                    uint16_t encoder_value = encoder->get_value(i);
#if BOARD_HAS_FEEDBACK_SOURCE_SELECTOR
                    if (encoder->is_initialized(i)) record_count(&switch_rest[i],encoder_value,sample_us);
                    else switch_rest[i] = (rest_sample_t){0};
#endif
#if BOARD_HAS_FUSION_EXPERIMENT
                    moved |= record_count(&rest[i], encoder_value, sample_us);
#endif
                    state->odometry_change[i] = odometry_integrator_wheel_delta(
                        state->encoder_previous_value[i], encoder_value, encoder->get_resolution(i));
                    state->odometry[i] = state->odometry[i] + state->odometry_change[i];

                    state->encoder_previous_value[i] = encoder_value;
                    state->encoder_sample_us[i] = sample_us;
                }
#if BOARD_HAS_FUSION_EXPERIMENT
                else if (encoder->is_initialized(i) && (watch_standalone || BOARD_HAS_FEEDBACK_SOURCE_SELECTOR)) {
                    uint16_t count = encoder->get_value(i);
#if BOARD_HAS_FEEDBACK_SOURCE_SELECTOR
                    record_count(&switch_rest[i],count,sample_us);
#endif
                    if (watch_standalone) moved |= record_count(&rest[i],count,sample_us);
                    else rest[i] = (rest_sample_t){0};
                } else {
                    rest[i] = (rest_sample_t){0};
#if BOARD_HAS_FEEDBACK_SOURCE_SELECTOR
                    switch_rest[i] = (rest_sample_t){0};
#endif
                }
                if (rest[i].seen && sample_us - rest[i].quiet_since < 100000) stationary = false;
#endif
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
#if BOARD_HAS_FUSION_EXPERIMENT
                delta = odometry_change;
#if BOARD_HAS_FEEDBACK_SOURCE_SELECTOR
                uint64_t limit = (uint64_t)state->update_interval * 3000U;
                bool coherent = valid && previous && sample_us > previous && sample_us - previous <= limit &&
                    isfinite(delta.x) && isfinite(delta.y) && isfinite(delta.t) &&
                    isfinite(state->platform_odometry.x) && isfinite(state->platform_odometry.y) &&
                    isfinite(state->platform_odometry.t);
                if (coherent) {
                    double hz = 1e6 / (double)(sample_us - previous);
                    state->feedback_twist = (platform_odometry_t){delta.x*hz,delta.y*hz,delta.t*hz};
                    ++state->feedback_sequence;
                    feedback_anchor(&state->feedback_frame,FEEDBACK_SOURCE_ENCODER,sample_us,state->platform_odometry);
                } else if (state->feedback_valid) feedback_invalidate(FEEDBACK_SOURCE_ENCODER);
                state->feedback_valid = coherent;
                state->feedback_reason = !valid ? FEEDBACK_REASON_UNAVAILABLE :
                    !previous ? FEEDBACK_REASON_WAITING_SAMPLE :
                    sample_us <= previous || sample_us-previous > limit ? FEEDBACK_REASON_STALE :
                    FEEDBACK_REASON_UNAVAILABLE;
#endif
#endif
            }
#if BOARD_HAS_FUSION_EXPERIMENT
            platform_odometry_t pose = state->platform_odometry;
#endif
            xSemaphoreGive(state->odometry_mutex);
#if BOARD_HAS_FUSION_EXPERIMENT
#if BOARD_HAS_FEEDBACK_SOURCE_SELECTOR
            if (passive_epoch) passive_fusion_observe(passive_epoch, previous, sample_us, delta, valid, stationary, moved);
#endif
            if (epoch) platform_fusion_observe(epoch, previous, sample_us, delta, pose, valid, stationary, moved);
#endif
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
#if BOARD_HAS_FUSION_EXPERIMENT
            .stack_size = 3072, // BMI088 acquisition and software double-precision estimator.
#else
            .stack_size = 128 * 8,
#endif
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
#if BOARD_HAS_FEEDBACK_SOURCE_SELECTOR
    passive_fusion_reset();
#endif
    // Return if odometry manager is not initialized
    if (odometry_manager_is_not_initialized())
    {
        return;
    }

    // Obtain odometry mutex
    if (xSemaphoreTake(odometry_manager_state.odometry_mutex, portMAX_DELAY))
    {
        // Consume pending wheel counts before establishing the new platform
        // origin. Preserve encoder totals; pre-reset travel must not reappear
        // as platform motion on the following odometry tick.
        for (uint8_t i = 0; i < NUMBER_ENCODERS; ++i) {
            if (!platform_owns_motor(i) || !odometry_manager_state.is_initialized[i]) continue;
            uint16_t value = encoder->get_value(i);
            odometry_manager_state.odometry[i] += odometry_integrator_wheel_delta(
                odometry_manager_state.encoder_previous_value[i], value, encoder->get_resolution(i));
            odometry_manager_state.encoder_previous_value[i] = value;
            odometry_manager_state.odometry_change[i] = 0;
            odometry_manager_state.encoder_sample_us[i] = 0;
        }
        odometry_manager_state.platform_odometry.x = 0;
        odometry_manager_state.platform_odometry.y = 0;
        odometry_manager_state.platform_odometry.t = 0;
        odometry_manager_state.platform_sample_us = 0;
#if BOARD_HAS_FEEDBACK_SOURCE_SELECTOR
        odometry_manager_state.feedback_valid = false;
        feedback_invalidate(FEEDBACK_SOURCE_ENCODER);
#endif
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
#if BOARD_HAS_FEEDBACK_SOURCE_SELECTOR
        if (odometry_manager_state.update_interval != period_ms) {
            // A wider deadline cannot renew an acquisition produced at the old rate.
            odometry_manager_state.feedback_valid = false;
            odometry_manager_state.feedback_reason = FEEDBACK_REASON_WAITING_SAMPLE;
            feedback_invalidate(FEEDBACK_SOURCE_ENCODER);
        }
#endif
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

/** @brief Expose the actual calculation period for subscription rate validation. */
uint32_t odometry_manager_get_period_ms(void)
{
    return odometry_manager_state.update_interval;
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
#if BOARD_HAS_FEEDBACK_SOURCE_SELECTOR
    passive_fusion_reset();
#endif
    if (!odometry_manager_state.odometry_mutex) return;
    if (xSemaphoreTake(odometry_manager_state.odometry_mutex, portMAX_DELAY)) {
        odometry_manager_state.platform_sample_us = 0;
#if BOARD_HAS_FEEDBACK_SOURCE_SELECTOR
        odometry_manager_state.feedback_valid = false;
        feedback_invalidate(FEEDBACK_SOURCE_ENCODER);
#endif
        xSemaphoreGive(odometry_manager_state.odometry_mutex);
    }
}

#if BOARD_HAS_FUSION_EXPERIMENT
bool odometry_manager_recent_motion(uint64_t now)
{
    SemaphoreHandle_t mutex = odometry_manager_state.odometry_mutex;
    if (!mutex || !xSemaphoreTake(mutex, portMAX_DELAY)) return false;
    bool moving = false;
    for (uint8_t i = 0; i < NUMBER_ENCODERS; ++i) {
        if (!encoder->is_initialized(i)) continue;
        rest_sample_t *r = &rest[i];
        if (r->seen && now >= r->seen && now - r->seen <= 60000 &&
            encoder->get_value(i) != r->count) moving = true;
        if (r->moved && now >= r->moved && now - r->moved < 100000) moving = true;
    }
    xSemaphoreGive(mutex);
    return moving;
}
#endif

#if BOARD_HAS_FEEDBACK_SOURCE_SELECTOR
bool odometry_feedback_at_rest(uint64_t now)
{
    const hw_encoder_interface_t *hw = get_encoder_interface();
    bool any = false;
    for (uint8_t i=0; i<NUMBER_ENCODERS; ++i) any |= hw->is_initialized(i) != 0;
    if (!any) return true; // Only the operator and output/ownership gates can observe rest without encoders.
    // When odometry has never run, start only the passive counter sampler.
    // A later request can succeed after measured quiet time.
    odometry_manager_initialize();
    odometry_manager_state_t *state = &odometry_manager_state;
    if (!state->odometry_mutex || !xSemaphoreTake(state->odometry_mutex,portMAX_DELAY)) return false;
    bool quiet = true;
    for (uint8_t i=0; i<NUMBER_ENCODERS; ++i) {
        if (!hw->is_initialized(i)) continue;
        const rest_sample_t *r = &switch_rest[i];
        if (!r->seen || now < r->seen || now - r->seen > 60000 ||
            r->seen - r->quiet_since < 100000 || hw->get_value(i) != r->count) quiet = false;
    }
    xSemaphoreGive(state->odometry_mutex);
    return quiet;
}

void odometry_feedback_sample(feedback_snapshot_t *out)
{
    odometry_manager_state_t *s = &odometry_manager_state;
    if (!platform_is_odometry_enabled() || !s->odometry_mutex) { out->reason=FEEDBACK_REASON_NOT_STARTED; return; }
    if (!xSemaphoreTake(s->odometry_mutex,portMAX_DELAY)) { out->reason=FEEDBACK_REASON_UNAVAILABLE; return; }
    uint64_t now=hw_clock_microseconds(), stamp=s->platform_sample_us;
    if (!stamp) out->reason=FEEDBACK_REASON_WAITING_SAMPLE;
    else if (!s->feedback_valid) out->reason=s->feedback_reason;
    else if (now < stamp || now-stamp > (uint64_t)s->update_interval * 3000U) {
        s->feedback_valid=false;
        s->feedback_reason=out->reason=FEEDBACK_REASON_STALE;
        feedback_invalidate(FEEDBACK_SOURCE_ENCODER);
    }
    else feedback_snapshot(out,&s->feedback_frame,FEEDBACK_SOURCE_ENCODER,s->feedback_sequence,stamp,
                           s->platform_odometry,s->feedback_twist);
    xSemaphoreGive(s->odometry_mutex);
}
#endif
