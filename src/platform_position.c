#include "platform_position.h"
#include "platform.h"
#include "platform_fusion.h"
#include "platform_common.h"
#include "odometry_manager.h"
#include "hw_clock.h"
#include "protocol.h"
#include <cmsis_os.h>
#include <semphr.h>
#include <math.h>

static SemaphoreHandle_t mutex;
static uint8_t initialized, active;
static position_settings_t linear_settings, angular_settings;
static platform_odometry_t target_pose;
static position_platform_state_t pid_state;
static uint64_t previous_update;

static void reset_pid(void)
{
    pid_state = (position_platform_state_t){0};
    previous_update = 0;
}

static uint8_t positive(double v) { return isfinite(v) && v > 0; }

static uint8_t geometry_valid(void)
{
    if (platform.set_platform_target_velocity == differential_platform_set_target_velocity)
        return positive(platform.properties.differential.wheel_diameter) &&
            positive(platform.properties.differential.wheel_base);
    if (platform.set_platform_target_velocity == omni_platform_set_target_velocity)
        return positive(platform.properties.omni.wheel_diameter) &&
            positive(platform.properties.omni.robot_radius);
    if (platform.set_platform_target_velocity == mecanum_platform_set_target_velocity)
        return positive(platform.properties.mecanum.wheel_diameter) &&
            positive(platform.properties.mecanum.length) && positive(platform.properties.mecanum.width);
    return 0;
}

static void zero_velocity(void)
{
    if (platform.is_controller_initialized && platform.set_platform_target_velocity)
        platform.set_platform_target_velocity((platform_velocity_t){0});
}

#if BOARD_HAS_FUSION_EXPERIMENT
#include "platform_fusion_internal.h"
#if BOARD_HAS_FEEDBACK_SOURCE_SELECTOR
#include "passive_fusion_internal.h"
#endif
#endif

void platform_position_cancel(void)
{
    if (!mutex || !xSemaphoreTake(mutex, portMAX_DELAY)) return;
#if BOARD_HAS_FUSION_EXPERIMENT
    if (experiment.mode) clear_target();
#endif
    if (initialized) zero_velocity();
    initialized = active = 0;
    reset_pid();
    xSemaphoreGive(mutex);
}

uint8_t platform_position_initialize(position_settings_t linear, position_settings_t angular)
{
    if (!position_settings_valid(linear) || !position_settings_valid(angular))
        return RESPONSE_INVALID_ARGUMENT;
    if (!platform.is_initialized) return RESPONSE_PLATFORM_NOT_INITIALIZED;
    if (!platform.is_controller_initialized) return RESPONSE_CONTROLLER_NOT_INITIALIZED;
    if (!geometry_valid()) return RESPONSE_INVALID_ARGUMENT;
    const hw_encoder_interface_t *encoders = get_encoder_interface();
    for (uint8_t i = 0; i < 4; ++i)
        if (platform.motor_mask & (1U << i)) {
            if (!controllers_manager_is_running(i)) return RESPONSE_CONTROLLER_NOT_INITIALIZED;
            if (!encoders->is_initialized(i) || !positive(encoders->get_resolution(i)))
                return RESPONSE_ENCODER_NOT_INITIALIZED;
        }
    if (!mutex) mutex = xSemaphoreCreateMutex();
    if (!mutex || !xSemaphoreTake(mutex, portMAX_DELAY)) return RESPONSE_INTERNAL_ERROR;
#if BOARD_HAS_FUSION_EXPERIMENT
    if (experiment.mode) clear_target();
#endif
    linear_settings = linear;
    angular_settings = angular;
    reset_pid();
    initialized = 1;
    active = 0;
    zero_velocity();
    // Preserve an existing odometry frame; start feedback automatically if needed.
    if (!platform_is_odometry_enabled()) platform_start_odometry();
    xSemaphoreGive(mutex);
    return RESPONSE_OK;
}

static uint8_t feedback(platform_odometry_t *pose)
{
#if BOARD_HAS_FUSION_EXPERIMENT
    if (experiment.mode) {
        if (!usable()) return RESPONSE_SAMPLE_NOT_AVAILABLE;
        *pose = experiment.mode == FUSION_MODE_ENCODER_CORRECTION ? experiment.pose :
            (platform_odometry_t){experiment.estimator.x, experiment.estimator.y, experiment.estimator.yaw};
        return RESPONSE_OK;
    }
#endif
    uint64_t sample;
    uint8_t error = odometry_manager_get_platform_sample(pose, &sample);
    if (error != RESPONSE_OK) return error;
    uint64_t now = hw_clock_microseconds();
    // Allow three actual odometry periods; detect a stopped or stalled producer.
    if (sample > now || now - sample > (uint64_t)odometry_manager_get_period_ms() * 3000U)
        return RESPONSE_SAMPLE_NOT_AVAILABLE;
    if (!isfinite(pose->x) || !isfinite(pose->y) || !isfinite(pose->t))
        return RESPONSE_SAMPLE_NOT_AVAILABLE;
    return RESPONSE_OK;
}

uint8_t platform_position_set(platform_odometry_t target)
{
    if (!isfinite(target.x) || !isfinite(target.y) || !isfinite(target.t))
        return RESPONSE_INVALID_ARGUMENT;
    if (!mutex) return RESPONSE_CONTROLLER_NOT_INITIALIZED;
    if (!xSemaphoreTake(mutex, portMAX_DELAY)) return RESPONSE_INTERNAL_ERROR;
    platform_odometry_t pose;
    uint8_t error = !initialized || !platform.is_controller_initialized ?
        RESPONSE_CONTROLLER_NOT_INITIALIZED : feedback(&pose);
#if BOARD_HAS_FUSION_EXPERIMENT
    if (experiment.mode && !fresh_request()) error = RESPONSE_SAMPLE_NOT_AVAILABLE;
#endif
    if (error == RESPONSE_OK) {
        if (!active || target.x != target_pose.x || target.y != target_pose.y || target.t != target_pose.t) {
            reset_pid();
#if BOARD_HAS_FUSION_EXPERIMENT
            reset_correction();
#endif
        }
#if BOARD_HAS_FUSION_EXPERIMENT
        if (experiment.mode) {
            controllers_manager_feedback_hold(platform.motor_mask, false);
            experiment.target_live = true;
        }
#endif
        target_pose = target; active = 1;
    }
    xSemaphoreGive(mutex);
    return error;
}

uint8_t platform_position_reset(void)
{
    if (!mutex) return RESPONSE_CONTROLLER_NOT_INITIALIZED;
    if (!xSemaphoreTake(mutex, portMAX_DELAY)) return RESPONSE_INTERNAL_ERROR;
    uint8_t error = initialized ? RESPONSE_OK : RESPONSE_CONTROLLER_NOT_INITIALIZED;
    if (error == RESPONSE_OK) {
        active = 0;
        reset_pid();
        zero_velocity();
#if BOARD_HAS_FUSION_EXPERIMENT
        if (experiment.mode) { restart_batch(true); experiment.health = FUSION_HEALTH_WAITING; }
#endif
        odometry_manager_reset_platform_odometry();
        target_pose = (platform_odometry_t){0};
    }
    xSemaphoreGive(mutex);
    return error;
}

void platform_position_update(void)
{
    if (!mutex || !xSemaphoreTake(mutex, portMAX_DELAY)) return;
#if BOARD_HAS_FUSION_EXPERIMENT
    if (experiment.mode && experiment.target_live) {
        bool running = platform.is_controller_initialized;
        for (uint8_t i = 0; running && i < 4; ++i)
            if (platform_owns_motor(i) && !controllers_manager_is_running(i)) running = false;
        if (!running) reject_feedback(FUSION_FAULT_INHIBITED);
        else if (usable() && !active) apply_correction(experiment.direct);
    }
#endif
    if (active) {
        platform_odometry_t pose;
        uint8_t healthy = platform.is_controller_initialized && feedback(&pose) == RESPONSE_OK;
        for (uint8_t i = 0; healthy && i < 4; ++i)
            if ((platform.motor_mask & (1U << i)) && !controllers_manager_is_running(i)) healthy = 0;
        if (!healthy) {
            // Feedback failure latches idle; recovery requires a new target.
            zero_velocity();
            active = 0;
            reset_pid();
        } else {
            uint64_t now = hw_clock_microseconds();
            double dt = previous_update && now > previous_update ? (now - previous_update) / 1000000.0 : 0;
            previous_update = now;
            platform_velocity_t velocity = position_platform_pid_velocity(
                linear_settings, angular_settings, &pid_state, pose, target_pose,
                platform.set_platform_target_velocity == differential_platform_set_target_velocity, dt);
#if BOARD_HAS_FUSION_EXPERIMENT
            if (experiment.mode) {
                bool reached = !velocity.x && !velocity.y && !velocity.t;
                controllers_manager_feedback_hold(platform.motor_mask, reached);
                if (reached) reset_correction();
                else apply_correction(velocity);
            }
            else
#endif
                platform.set_platform_target_velocity(velocity);
        }
    }
    xSemaphoreGive(mutex);
}
