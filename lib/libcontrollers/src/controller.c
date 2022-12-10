//------------------------------------------------------------
// File name: controller.c
//------------------------------------------------------------
#include <stdlib.h>
#include "controller.h"

signed char verify_range(signed char c);
int sing(int c);

mecanum_velocity_t get_mecanum_velocities(signed char x, signed char y, signed char t)
{
    // Adjust if needed velocity input components
    int nx = verify_range(x);
    int ny = verify_range(y);
    int nt = verify_range(t);

    // Normalize velocity
    int l = abs(nx) + abs(ny) + abs(nt);
    nx = sing(nx) * nx * nx / l;
    ny = sing(ny) * ny * ny / l;
    nt = sing(nt) * nt * nt / l;

    mecanum_velocity_t velocities = {
        .motor0 = -(nx + ny + nt),
        .motor1 = -(nx - ny + nt),
        .motor2 = nx + ny - nt,
        .motor3 = nx - ny - nt
        };

    return velocities;
};

signed char verify_range(signed char c)
{
    if(c > 100) return 100;
    if(c < -100) return -100;
    return c;
}

int sing(int c)
{
    return (c > 0) - (c < 0);
}