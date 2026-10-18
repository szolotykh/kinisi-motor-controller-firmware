# Board v0.4 firmware support

## Motor control configuration

Board motor definitions live in [v0.4.json](../boards/v0.4.json) and generate
[hw_config.h](../include/hw_config.h); their structure and
`motor_control_model_t` enum live in [hw_config_structs.h](../lib/hardware/hw_config_structs.h).
Each motor's `controlModel` is `MOTOR_CONTROL_PWM` (V1-V3) or
`MOTOR_CONTROL_ENABLE_DIRECTION` (V4). PWM uses `pwmChannel1` and
`pwmChannel2`; enable/direction uses the `enable` PWM channel and the
`direction` GPIO. The `complementary` flag selects a timer's N output.

Each board also selects `MOTOR_CONTROL_MODEL` using the corresponding
numeric `MOTOR_CONTROL_MODEL_*` selector. These selectors support C `#if`
checks: enum names alone cannot be evaluated by the preprocessor.
`hw_motor.c` compiles only the selected speed, stop/brake and initialization
path, and rejects motor entries whose enum differs from the board selection.
All motors on a board must use that selected model. DRV8873 SPI wake-up and
configuration are separate in `hw_drv8873.c`; its motor pin definitions come
from the same board configuration.

Run `python test/test_motor_control/run_tests.py` with a host GCC or Clang
available (or set `CC`) to test all four profiles without attached hardware.

## Board 0.4.0 basic support

Select the `v0_4` PlatformIO environment or build with `pio run -e v0_4`
for the STM32F405VGT6 board.
The original `genericSTM32F405RG` environment continues to select MP_V3.
INIT reports board version 0.4.0 for the new environment.

MP_V4 supports four DRV8873S motors using SPI3 and TIM1 EN PWM,
PH direction, DISABLE coast/stop, and EN-low brake. All motors on V1-V4 use
100 kHz PWM with 840 duty steps. Each timer prescaler is derived from its
APB1 or APB2 timer clock; V4 uses TIM1 for all four motors.
V4 explicitly programs IC1 SR=000 (53.2 V/us) and verifies readback for all
four drivers. The reset SR=100 (10.8 V/us) has longer switching delays
relative to the 10 us PWM period, limiting useful duty range at 100 kHz.
`motors.driver.slew_rate_v_per_us` in the JSON generates `DRV8873_SLEW_RATE_BITS`.
See [board configuration](../boards/README.md) for generation and validation.
Physical low-duty behavior, switching noise and motor startup still need
board measurements; software duty is not a guarantee of bridge output duty.
Failed motor initialization or a fault-blocked nonzero speed command returns
INTERNAL_ERROR instead of silently acknowledging success. Channels 1-3 use
complementary outputs with inverted polarity; motor 0 uses regular channel 4.
Startup holds every driver disabled and asleep. Motor initialization wakes its
driver, writes and reads back PH/EN mode, and starts zero-duty PWM while
remaining disabled. SPI failure leaves that motor uninitialized and disabled.
Explicit motor reinitialization coasts the channel, cycles sleep to clear latched faults, and
reprograms/verifies its driver without restarting its PWM timer. Stop before
removing motor power; after restoring it while USB remains connected,
reinitialize the motor or platform before driving. Automatic detection and
reconfiguration after a power interruption are not implemented. A speed
command with an asserted nFAULT coasts the motor. Current regulation retains
the driver's factory default (6.5 A nominal); this is not a board current rating.
Zero speed coasts; explicit brake uses the PH/EN high-side brake state.

The four encoder mappings, eight external GPIOs, and PB14 status LED follow
the new board. Sensor/FRAM support, current ADC measurements, asynchronous
fault reporting, and UART support are outside this initial motor-control scope.
Fault inputs are checked on speed commands, not monitored by a background task.

Each motor's `current_sense` definition explicitly selects `"type": "adc"`.
The generator requires this type and emits `CURRENT_SENSE_ADC` in the C
configuration. Missing or unsupported types are rejected; I2C sensing requires
a separate implementation before it can be selected. The ADC configuration
currently describes the circuit only; it does not enable sampling or telemetry.

Each motor's `hardware_current_limit.supported_thresholds_ma` lists the nominal
hardware regulation settings in milliamps: `[4000, 5400, 6500, 7000]`.
The generator emits this list and its count alongside `enabled` and `threshold_ma`
in the C configuration. These describe hardware capability and the selected
configuration; they do not expose a client command. The selected threshold must
be in the list. Until IC4 programming is implemented, validation only accepts
`enabled: true` and `threshold_ma: 6500`, matching the driver's factory default.

Each motor also declares `overcurrent_protection` separately from current
regulation. Its `supported_modes` are `latched_shutdown`, `automatic_retry`,
`report_only`, and `disabled`; `mode` selects `latched_shutdown`, the factory
default. The generator emits a typed mode, the supported-mode array and its
count. This remains metadata: IC2 protection-mode programming is not implemented,
so validation rejects other selected modes instead of silently ignoring them.
The modes describe hardware capability, not available client commands. Legacy
board profiles leave this metadata unavailable; that does not establish whether
their drivers have internal protection.

Before powered use, verify SPI readback, physical EN duty/polarity at zero and
100%, motor direction/reversal, coast/brake, encoder counts, and fault behavior
on all four channels. Compilation and host tests do not verify these electrical
behaviors. Driver configuration follows the
[TI DRV8873 datasheet](https://www.ti.com/lit/ds/symlink/drv8873.pdf).

Motor PID and platform initialization also return INTERNAL_ERROR when a
driver cannot initialize. Failed platform setup stops its selected wheels and
invalidates the platform rather than enabling a partially initialized base.

## Validation

Host tests cover V1-V4 motor behavior, 100 kHz timer setup, SPI failures and
reinitialization, PID failure cleanup, and platform rollback. Build both
`genericSTM32F405RG` and `v0_4` when changing the shared motor implementation.
USB communication, status LED blinking, and motor 0 movement were observed
on the v0.4 prototype. Encoder counts and complete four-channel electrical
validation remain outstanding; the reinitialization/failure-cleanup changes
have been verified in host tests and builds, not flashed to the prototype.
