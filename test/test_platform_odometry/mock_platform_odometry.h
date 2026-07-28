//------------------------------------------------------------
// File name: mock_platform_odometry.h
//------------------------------------------------------------
//
// Minimal hardware/controller stubs so the real platform_*.c and
// odometry_integrator.c can be linked into the odometry pipeline test.
//------------------------------------------------------------

#pragma once

// Install the mock hw interfaces (motor/encoder). Call from setUp.
void mock_odometry_install_interfaces(void);
