//------------------------------------------------------------
// File name: odometry_integrator.c
//------------------------------------------------------------
#include <odometry_integrator.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

//------------------------------------------------------------
double odometry_integrator_wheel_delta(uint16_t previous_value,
                                       uint16_t current_value,
                                       double resolution)
{
    // Ticks per radian.
    double ticks_per_rad = resolution / (2.0 * M_PI);
    if (ticks_per_rad == 0.0)
    {
        return 0.0;
    }

    // uint16 subtraction wraps mod 65536, so this is the raw counter change.
    uint16_t change_raw = current_value - previous_value;

    // A raw change above half range is really a small negative (backward)
    // movement that underflowed the unsigned counter.
    double change = change_raw;
    if (change_raw > 32768) // Half of UINT16_MAX
    {
        change = -(65536.0 - change_raw);
    }

    return change / ticks_per_rad;
}

//------------------------------------------------------------
platform_odometry_t odometry_integrator_accumulate(platform_odometry_t pose,
                                                    platform_odometry_t body_delta)
{
    // Rotate the body-frame translation increment into the world frame using
    // the heading at the midpoint of the tick. See header for rationale.
    double heading_mid = pose.t + body_delta.t / 2.0;
    double cos_h = cos(heading_mid);
    double sin_h = sin(heading_mid);

    pose.x += body_delta.x * cos_h - body_delta.y * sin_h;
    pose.y += body_delta.x * sin_h + body_delta.y * cos_h;
    pose.t += body_delta.t;

    return pose;
}
