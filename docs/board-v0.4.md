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
the new board. FRAM support, current ADC measurements, asynchronous
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

## Board temperature

The v0.4 TMP102 (U602) is sensor 0 at the 7-bit address `0x48` (decimal 72).
Its I2C3 bus uses PA8/SCL and PC9/SDA at 100 kHz with external pull-ups. This
is separate from the I2C2 client transport. The FRAM shares I2C3 at a different
address; temperature support does not access it. The sensor's ALERT pin is
unconnected, so this feature provides polled readings, without automatic motor
shutdown or temperature events.

`temperature_sensors` in `boards/v0.4.json` describes the sensor type, bus and
address. The generator emits its C configuration and count. An omitted or empty
array means no temperature sensors (as on v0.1-v0.3). Bus definitions must be
used and their pins must not conflict with other resources.

Startup configures continuous 4 Hz conversion in 12-bit mode and checks the
configuration readback. Signed readings have 0.0625 degree C resolution, per the
[TMP102 datasheet](https://www.ti.com/lit/ds/symlink/tmp102.pdf). They measure the
sensor's local board temperature, not an individual motor or driver junction.

Protocol **2.4.0** adds `GET_TEMPERATURE` (`0x50`). Send one `uint8_t` sensor index;
the response contains a little-endian 64-bit `double` in degrees Celsius. For
example, after INIT, the complete request `04 50 01 00 00` reads sensor 0 using
message ID 1. A successful reply starts `0B 50 01 00` followed by the eight-byte
temperature. Existing protocol 2.0-2.3 clients remain compatible.

`SENSOR_NOT_AVAILABLE` (16) means that index is not defined on the selected
board. `SAMPLE_NOT_AVAILABLE` (10) means the first 300 ms after initialization
or recovery have not elapsed; retry later. `INTERNAL_ERROR` (6) reports an I2C
failure or invalid configuration/data. A failed read never returns a cached
temperature as success. Startup failure does not block boot, and subsequent
reads retry initialization. Sensor transactions have bounded HAL timeouts;
the command does not wait for a conversion. Reads faster than 4 Hz can repeat
the latest conversion.

An I2C transfer or initialization failure invalidates every temperature sensor
on the shared bus. The next read attempts bus recovery: release the peripheral,
send up to nine open-drain SCL pulses and a STOP, then reset and reinitialize
I2C3. Recovery uses a 50 ms time budget. If either line remains low, the request
returns an error with both outputs released; later reads can retry. Successful
recovery requires fresh sensor configuration and the normal 300 ms conversion
wait. The external I2C2 client transport is not reset.

Run `python test/test_temperature/run_tests.py` for host checks. A firmware build
and host tests do not verify the physical sensor; confirm readings on the board
after flashing.

## Motion sensor (IMU)

`imu_sensors` in `boards/v0.4.json` defines BMI088 sensor 0. The generator emits
the sensor count, type, SPI bus and GPIOs. Older boards omit this array and return
`SENSOR_NOT_AVAILABLE` without accessing SPI2.

SPI2 uses PB13/SCK, PC2/MISO and PC3/MOSI, mode 0, APB1 divided by 64
(656.25 kHz with the current clocks). PD10 selects the accelerometer and PD12
selects the gyroscope. These are independent of the motor drivers on SPI3.
PD11/accel INT2 and PD9/gyro INT4 are recorded as wiring metadata. This driver
polls data registers; it does not configure EXTI, interrupts, FIFO or the physical
gyro INT3 to accel INT1 synchronization connection.

The driver follows the [Bosch BMI088 datasheet, revision 1.9](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi088-ds001.pdf):
it selects the accelerometer's SPI interface, verifies both chip IDs, resets both
devices, enables acceleration sampling, and configures 100 Hz sampling. The
accelerometer range is ±6 g with its normal filter; the gyroscope range is
±500 degrees/s with 32 Hz bandwidth. Each three-axis burst is coherent for its
sensor. Acceleration and angular velocity samples are independent and are not
guaranteed to describe the same instant.

Protocol **2.5.0** adds `GET_IMU` (`0x51`), with one `uint8_t` sensor index. The
48-byte reply contains six little-endian doubles, in this order:
`acceleration_x`, `acceleration_y`, `acceleration_z` (m/s², including gravity),
`angular_velocity_x`, `angular_velocity_y`, `angular_velocity_z` (rad/s).
Coordinates are the BMI088's native axes, without board/platform remapping,
bias calibration or orientation estimation. Faster reads can repeat samples.

Initialization is automatic and nonfatal if the IMU cannot be reached. Setup
honors power/reset/write delays, then readings return `SAMPLE_NOT_AVAILABLE`
during a 50 ms settling interval. SPI transfers have 10 ms timeouts. Reads verify
identity and relevant configuration before converting samples; failures return
`INTERNAL_ERROR` without publishing partial or stale values. A subsequent request
retries setup, which can spend about 50 ms in reset/configuration delays before
starting the settling interval. Existing client protocols remain compatible.

Run `python test/test_imu/run_tests.py` for host checks. On 2026-10-17, the v0.4
image was flashed with readback verification, and the board reported protocol
2.5.0 over USB. Five IMU reads returned acceleration magnitudes of 9.896–9.926
m/s² and angular velocities below 0.008 rad/s on every axis; the TMP102 read
24.5 °C. This verifies communication and plausible readings near rest. Physical
axis direction, deliberate rotation response and calibration remain unverified.

## Validation

Host tests cover V1-V4 motor behavior, 100 kHz timer setup, SPI failures and
reinitialization, PID failure cleanup, and platform rollback. Build both
`genericSTM32F405RG` and `v0_4` when changing the shared motor implementation.
USB communication, status LED blinking, and motor 0 movement were observed
on the v0.4 prototype. Encoder counts and complete four-channel electrical
validation remain outstanding; the reinitialization/failure-cleanup changes
have been verified in host tests and builds, not flashed to the prototype.
