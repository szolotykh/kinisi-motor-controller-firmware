#include "controller_configuration.h"
#include "board_revision.h"
#include "protocol.h"
#include "platform_common.h"

static uint8_t read_mecanum_orientation(uint8_t index, double *value)
{
    if (index || !value) return RESPONSE_INVALID_ARGUMENT;
    if (!platform.is_initialized || platform.type != PLATFORM_MECANUM)
        return RESPONSE_PLATFORM_NOT_INITIALIZED;
    *value = platform.properties.mecanum.roller_orientation;
    return RESPONSE_OK;
}

#ifdef MP_V4
#include "config_store.h"
#include "hw_encoder.h"
#include "hw_motor.h"
#include "platform.h"
#include <math.h>
#include "feedback_source.h"
#include "imu_mount_wire.h"
#include <stddef.h>

// Fixed-size payload for the v0.4 STM32, little-endian with binary64 geometry.
// Explicit padding and masks keep timers, outputs and runtime objects out of FRAM.
typedef struct {
    double diameter, track_or_length, width_or_radius;
    double resolution[4];
    uint32_t frequency;
    uint8_t type, motors, motor_reverse, encoders, encoder_reverse;
    uint8_t feedback_preference, payload_revision; // Both bytes were reserved zero in v1.
    uint8_t roller_orientation, mount_reference, reserved[3]; // Reserved zero through v4.
} saved_setup_t;
_Static_assert(sizeof(saved_setup_t) == 72 && sizeof(double) == 8, "v1 configuration payload layout");
typedef struct {
    saved_setup_t base;
    imu_mount_configuration mount;
} mounted_setup_t;
// Store only defined bytes. Keep the old prefix and avoid trailing ABI padding.
enum { SETUP_ORIENTED = 5, MOUNT_NONE = 0, MOUNT_CHIP = 1, MOUNT_BOARD = 2 };
enum { MOUNTED_LENGTH = sizeof(saved_setup_t) + sizeof(imu_mount_configuration) };
_Static_assert(offsetof(mounted_setup_t, mount) == 72 && MOUNTED_LENGTH == 91,
               "v3 configuration payload layout");

static uint8_t bit(uint8_t mask, unsigned index)
{
    return (mask >> index) & 1U;
}

static void capture(saved_setup_t *setup)
{
    *setup = (saved_setup_t){0};
    if (platform.is_initialized) {
        setup->type = platform.type;
        switch (platform.type) {
        case PLATFORM_DIFFERENTIAL:
            setup->diameter = platform.properties.differential.wheel_diameter;
            setup->track_or_length = platform.properties.differential.wheel_base;
            break;
        case PLATFORM_OMNI:
            setup->diameter = platform.properties.omni.wheel_diameter;
            setup->width_or_radius = platform.properties.omni.robot_radius;
            break;
        case PLATFORM_MECANUM:
            setup->diameter = platform.properties.mecanum.wheel_diameter;
            setup->track_or_length = platform.properties.mecanum.length;
            setup->width_or_radius = platform.properties.mecanum.width;
            setup->roller_orientation = platform.properties.mecanum.roller_orientation;
            break;
        default: break;
        }
    }
    const hw_motor_interface_t *motor = get_motor_interface();
    const hw_encoder_interface_t *encoder = get_encoder_interface();
    for (unsigned i = 0; i < 4; ++i) {
        if (motor->is_initialized(i)) {
            setup->motors |= 1U << i;
            setup->motor_reverse |= motor->is_reversed(i) << i;
        }
        if (encoder->is_initialized(i)) {
            setup->encoders |= 1U << i;
            setup->resolution[i] = encoder->get_resolution(i);
            setup->encoder_reverse |= hw_encoder_is_reversed(i) << i;
        } else if (platform.is_initialized) {
            setup->encoder_reverse |= platform.encoder_reverse_mask & (1U << i);
        }
    }
#if BOARD_HAS_FEEDBACK_SOURCE_SELECTOR
    setup->feedback_preference = feedback_requested();
    setup->payload_revision = 2;
#endif
    setup->frequency = hw_motor_get_pwm_frequency();
}

static bool positive(double number)
{
    return isfinite(number) && number > 0;
}

static uint8_t platform_motors(uint8_t type)
{
    switch (type) {
    case PLATFORM_DIFFERENTIAL: return BMOTOR0 | BMOTOR1;
    case PLATFORM_OMNI: return BMOTOR0 | BMOTOR1 | BMOTOR2;
    case PLATFORM_MECANUM: return BMOTOR0 | BMOTOR1 | BMOTOR2 | BMOTOR3;
    default: return 0;
    }
}

static bool valid(const saved_setup_t *s)
{
    if (s->payload_revision == 0) {
        if (s->feedback_preference != FEEDBACK_SOURCE_ENCODER) return false;
    } else if ((s->payload_revision != 2 && s->payload_revision != 3 && s->payload_revision != 4 &&
                s->payload_revision != SETUP_ORIENTED) ||
               s->feedback_preference > FEEDBACK_SOURCE_FUSED) return false;
    if (s->payload_revision == SETUP_ORIENTED) {
        if (s->type != PLATFORM_MECANUM || s->roller_orientation != MECANUM_ROLLER_MIRRORED ||
            s->mount_reference > MOUNT_BOARD ||
            !mecanum_geometry_valid(s->track_or_length, s->width_or_radius, s->diameter, 0,
                                    s->roller_orientation)) return false;
    } else if (s->roller_orientation || s->mount_reference) return false;
    for (unsigned i = 0; i < sizeof(s->reserved); ++i) if (s->reserved[i]) return false;
    if (!hw_motor_pwm_frequency_allowed(s->frequency)) return false;
    uint8_t wheels = platform_motors(s->type);
    if ((s->motors & wheels) != wheels ||
        (s->motors | s->encoders) > 15 || (s->motor_reverse & ~s->motors) ||
        (s->encoder_reverse & ~(s->encoders | wheels))) return false;
    for (unsigned i = 0; i < 4; ++i) {
        if (bit(s->encoders, i) ? !positive(s->resolution[i]) : s->resolution[i] != 0) return false;
    }
    if (s->type == PLATFORM_NONE)
        return (s->payload_revision == 3 || s->payload_revision == 4) && s->diameter == 0 && s->track_or_length == 0 &&
               s->width_or_radius == 0;
    return wheels && positive(s->diameter) &&
        (s->type == PLATFORM_OMNI ? s->track_or_length == 0 : positive(s->track_or_length)) &&
        (s->type == PLATFORM_DIFFERENTIAL ? s->width_or_radius == 0 : positive(s->width_or_radius));
}

uint8_t controller_configuration_save(void)
{
    imu_mount_saved_t saved = imu_mount_saved();
    imu_mount_config_t mount = saved.entered;
    if (!platform.is_initialized && !mount.configured) return RESPONSE_PLATFORM_NOT_INITIALIZED;
    mounted_setup_t setup = {0};
    capture(&setup.base);
    uint16_t length = sizeof(setup.base);
    if (mount.configured) {
        if (!imu_mount_valid(&mount)) return RESPONSE_INVALID_ARGUMENT;
        // Both revisions store the exact entered coordinates. V3 refers to U601,
        // v4 to the PCB underside, using the same 19-byte numeric layout.
        setup.base.payload_revision = saved.reference == IMU_MOUNT_BOARD_UNDERSIDE ? 4 : 3;
        setup.mount = imu_mount_pack(mount);
        length = MOUNTED_LENGTH;
    }
    // Standard configurations retain their original record format. V5 uses two
    // previously reserved bytes and explicitly identifies the mount reference.
    if (setup.base.roller_orientation == MECANUM_ROLLER_MIRRORED) {
        if (mount.configured)
            setup.base.mount_reference = saved.reference == IMU_MOUNT_BOARD_UNDERSIDE ? MOUNT_BOARD : MOUNT_CHIP;
        setup.base.payload_revision = SETUP_ORIENTED;
    }
    if (!valid(&setup.base)) return RESPONSE_INVALID_ARGUMENT;
    return config_store_save(&setup, length) ? RESPONSE_OK : RESPONSE_INTERNAL_ERROR;
}

uint8_t controller_configuration_read(uint8_t field, uint8_t index, double *value)
{
    if (field == 7) return read_mecanum_orientation(index, value);
    if (!value || field > 6 || (field < 4 ? index != 0 : index >= 4)) return RESPONSE_INVALID_ARGUMENT;
    saved_setup_t setup;
    capture(&setup);
    if (field >= 1 && field <= 3 && setup.type == PLATFORM_NONE) return RESPONSE_PLATFORM_NOT_INITIALIZED;
    switch (field) {
    case 0: *value = setup.type; break;
    case 1: *value = setup.diameter; break;
    case 2: *value = setup.track_or_length; break;
    case 3: *value = setup.width_or_radius; break;
    case 4: *value = setup.resolution[index]; break;
    case 5: *value = bit(setup.encoder_reverse, index); break;
    case 6: *value = bit(setup.motor_reverse, index); break;
    }
    return RESPONSE_OK;
}

void controller_configuration_restore(void)
{
    mounted_setup_t record = {0};
    uint16_t length = config_store_load_latest(&record, MOUNTED_LENGTH);
    saved_setup_t s = record.base;
    imu_mount_config_t mount = {0};
    bool mounted = s.payload_revision == 3 || s.payload_revision == 4 ||
                   (s.payload_revision == SETUP_ORIENTED && s.mount_reference != MOUNT_NONE);
    bool board_reference = s.payload_revision == 4 ||
                           (s.payload_revision == SETUP_ORIENTED && s.mount_reference == MOUNT_BOARD);
    if (length == MOUNTED_LENGTH && mounted) {
        mount = imu_mount_unpack(record.mount);
        if (record.mount.configured != 1 || !imu_mount_valid(&mount)) return;
    } else if (length != sizeof(s) || mounted) return;
    if (!valid(&s)) return;
    const hw_motor_interface_t *motor = get_motor_interface();
    uint8_t wheels = platform_motors(s.type);
    // Validate standalone motor drivers first. Wheel initialization itself
    // disables the bridges and rejects a partially initialized platform.
    for (unsigned i = 0; i < 4; ++i) {
        if (bit(s.motors & ~wheels, i)) {
            motor->initialize(i, bit(s.motor_reverse, i));
            if (!motor->is_initialized(i)) goto rollback;
        }
    }
    const uint8_t m = s.motor_reverse, e = s.encoder_reverse;
    // A zero common resolution defers encoder setup until the independent
    // per-channel scales below. This also preserves encoder-free platforms.
    switch (s.type) {
    case PLATFORM_DIFFERENTIAL:
        initialize_differential_platform(bit(m,0), bit(m,1), bit(e,0), bit(e,1),
            s.diameter, s.track_or_length, 0);
        break;
    case PLATFORM_OMNI:
        initialize_omni_platform(bit(m,0), bit(m,1), bit(m,2), bit(e,0), bit(e,1), bit(e,2),
            s.diameter, s.width_or_radius, 0);
        break;
    case PLATFORM_MECANUM:
        initialize_mecanum_platform_oriented(bit(m,0), bit(m,1), bit(m,2), bit(m,3),
            bit(e,0), bit(e,1), bit(e,2), bit(e,3), s.track_or_length, s.width_or_radius, s.diameter, 0,
            s.roller_orientation);
        break;
    }
    if ((s.type != PLATFORM_NONE && !platform.is_initialized) ||
        !hw_motor_set_pwm_frequency(s.frequency)) goto rollback;
    const hw_encoder_interface_t *encoder = get_encoder_interface();
    for (unsigned i = 0; i < 4; ++i) {
        if (!bit(s.encoders, i)) continue;
        if (!hw_encoder_try_initialize(i, s.resolution[i], bit(e,i)) ||
            !encoder->is_initialized(i) || encoder->get_resolution(i) != s.resolution[i] ||
            hw_encoder_is_reversed(i) != bit(e,i)) goto rollback;
    }
#if BOARD_HAS_FEEDBACK_SOURCE_SELECTOR
    feedback_restore_preference(s.feedback_preference);
#endif
    if (mount.configured && !imu_mount_restore_reference(&mount,
        board_reference ? IMU_MOUNT_BOARD_UNDERSIDE : IMU_MOUNT_CHIP)) goto rollback;
    return;
rollback:
#if BOARD_HAS_FEEDBACK_SOURCE_SELECTOR
    feedback_restore_preference(FEEDBACK_SOURCE_ENCODER);
#endif
    hw_motor_discard_restored_setup();
    hw_encoder_discard_restored_setup();
    platform = (platform_t){0};
}
#else
void controller_configuration_restore(void) {}
uint8_t controller_configuration_save(void) { return RESPONSE_SENSOR_NOT_AVAILABLE; }
uint8_t controller_configuration_read(uint8_t field, uint8_t index, double *value)
{
    if (field == 7) return read_mecanum_orientation(index, value);
    return RESPONSE_SENSOR_NOT_AVAILABLE;
}
#endif
