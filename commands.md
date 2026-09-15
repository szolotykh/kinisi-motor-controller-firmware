# Kinisi motor controller commands

**Protocol 2.1.0** · Generated from [commands.json](commands.json)

[Command index](#command-index) · [Wire format](#wire-format) · [Error codes](#error-codes)

> **Compatibility:** Protocol 2.x and protocol 1.x clients are incompatible.

## Command index

| Command | Code | Direction | Response |
| --- | --- | --- | --- |
| [`INITIALIZE_MOTOR`](#initialize_motor-0x01) | `0x01` | Client → controller | ACK |
| [`SET_MOTOR_SPEED`](#set_motor_speed-0x02) | `0x02` | Client → controller | ACK |
| [`STOP_MOTOR`](#stop_motor-0x03) | `0x03` | Client → controller | ACK |
| [`BRAKE_MOTOR`](#brake_motor-0x04) | `0x04` | Client → controller | ACK |
| [`INITIALIZE_MOTOR_CONTROLLER`](#initialize_motor_controller-0x05) | `0x05` | Client → controller | ACK |
| [`SET_MOTOR_TARGET_SPEED`](#set_motor_target_speed-0x06) | `0x06` | Client → controller | ACK |
| [`RESET_MOTOR_CONTROLLER`](#reset_motor_controller-0x07) | `0x07` | Client → controller | ACK |
| [`GET_MOTOR_CONTROLLER_STATE`](#get_motor_controller_state-0x08) | `0x08` | Client → controller | `motor_controller_state` |
| [`DELETE_MOTOR_CONTROLLER`](#delete_motor_controller-0x09) | `0x09` | Client → controller | ACK |
| [`SET_CONTROLLER_FREQUENCY`](#set_controller_frequency-0x0a) | `0x0A` | Client → controller | ACK |
| [`GET_CONTROLLER_FREQUENCY`](#get_controller_frequency-0x0b) | `0x0B` | Client → controller | `uint16_t` |
| [`INITIALIZE_ENCODER`](#initialize_encoder-0x11) | `0x11` | Client → controller | ACK |
| [`GET_ENCODER_VALUE`](#get_encoder_value-0x12) | `0x12` | Client → controller | `uint16_t` |
| [`START_ENCODER_ODOMETRY`](#start_encoder_odometry-0x13) | `0x13` | Client → controller | ACK |
| [`RESET_ENCODER_ODOMETRY`](#reset_encoder_odometry-0x14) | `0x14` | Client → controller | ACK |
| [`STOP_ENCODER_ODOMETRY`](#stop_encoder_odometry-0x15) | `0x15` | Client → controller | ACK |
| [`GET_ENCODER_ODOMETRY`](#get_encoder_odometry-0x16) | `0x16` | Client → controller | `encoder_odometry_sample` |
| [`SET_ODOMETRY_FREQUENCY`](#set_odometry_frequency-0x17) | `0x17` | Client → controller | ACK |
| [`GET_ODOMETRY_FREQUENCY`](#get_odometry_frequency-0x18) | `0x18` | Client → controller | `uint16_t` |
| [`INITIALIZE_GPIO_PIN`](#initialize_gpio_pin-0x20) | `0x20` | Client → controller | ACK |
| [`SET_GPIO_PIN_STATE`](#set_gpio_pin_state-0x21) | `0x21` | Client → controller | ACK |
| [`GET_GPIO_PIN_STATE`](#get_gpio_pin_state-0x22) | `0x22` | Client → controller | `uint8_t` |
| [`TOGGLE_GPIO_PIN_STATE`](#toggle_gpio_pin_state-0x23) | `0x23` | Client → controller | ACK |
| [`SET_STATUS_LED_STATE`](#set_status_led_state-0x25) | `0x25` | Client → controller | ACK |
| [`TOGGLE_STATUS_LED_STATE`](#toggle_status_led_state-0x26) | `0x26` | Client → controller | ACK |
| [`INITIALIZE_MECANUM_PLATFORM`](#initialize_mecanum_platform-0x30) | `0x30` | Client → controller | ACK |
| [`INITIALIZE_OMNI_PLATFORM`](#initialize_omni_platform-0x31) | `0x31` | Client → controller | ACK |
| [`INITIALIZE_DIFFERENTIAL_PLATFORM`](#initialize_differential_platform-0x32) | `0x32` | Client → controller | ACK |
| [`SET_PLATFORM_VELOCITY`](#set_platform_velocity-0x40) | `0x40` | Client → controller | ACK |
| [`START_PLATFORM_CONTROLLER`](#start_platform_controller-0x41) | `0x41` | Client → controller | ACK |
| [`SET_PLATFORM_TARGET_VELOCITY`](#set_platform_target_velocity-0x42) | `0x42` | Client → controller | ACK |
| [`GET_PLATFORM_CURRENT_VELOCITY`](#get_platform_current_velocity-0x43) | `0x43` | Client → controller | `platform_velocity` |
| [`STOP_PLATFORM_CONTROLLER`](#stop_platform_controller-0x44) | `0x44` | Client → controller | ACK |
| [`START_PLATFORM_ODOMETRY`](#start_platform_odometry-0x45) | `0x45` | Client → controller | ACK |
| [`RESET_PLATFORM_ODOMETRY`](#reset_platform_odometry-0x46) | `0x46` | Client → controller | ACK |
| [`STOP_PLATFORM_ODOMETRY`](#stop_platform_odometry-0x47) | `0x47` | Client → controller | ACK |
| [`GET_PLATFORM_ODOMETRY`](#get_platform_odometry-0x48) | `0x48` | Client → controller | `platform_odometry_sample` |
| [`BRAKE_PLATFORM`](#brake_platform-0x49) | `0x49` | Client → controller | ACK |
| [`COAST_PLATFORM`](#coast_platform-0x4a) | `0x4A` | Client → controller | ACK |
| [`INIT`](#init-0x70) | `0x70` | Client → controller | `init_response` + [`READY`](#ready-0x73) |
| [`ERROR`](#error-0x7f) | `0x7F` | Controller → client | — |
| [`TIME_SYNC_REQUEST`](#time_sync_request-0x71) | `0x71` | Controller → client | [`TIME_SYNC_RESPONSE`](#time_sync_response-0x72) |
| [`TIME_SYNC_RESPONSE`](#time_sync_response-0x72) | `0x72` | Client → controller | No ACK |
| [`READY`](#ready-0x73) | `0x73` | Controller → client | — |
| [`SET_TIME_SYNC_INTERVAL`](#set_time_sync_interval-0x74) | `0x74` | Client → controller | ACK |
| [`GET_TIME_STATUS`](#get_time_status-0x75) | `0x75` | Client → controller | `time_status` |
| [`PING`](#ping-0x76) | `0x76` | Client → controller | ACK |
| [`SET_HEARTBEAT_CONFIG`](#set_heartbeat_config-0x77) | `0x77` | Client → controller | ACK |
| [`GET_HEARTBEAT_CONFIG`](#get_heartbeat_config-0x78) | `0x78` | Client → controller | `heartbeat_config` |
| [`SUBSCRIBE_ODOMETRY`](#subscribe_odometry-0x79) | `0x79` | Client → controller | ACK |
| [`UNSUBSCRIBE_ODOMETRY`](#unsubscribe_odometry-0x7a) | `0x7a` | Client → controller | ACK |
| [`ENCODER_ODOMETRY_EVENT`](#encoder_odometry_event-0x7b) | `0x7b` | Controller → client | — |
| [`PLATFORM_ODOMETRY_EVENT`](#platform_odometry_event-0x7c) | `0x7c` | Controller → client | — |
| [`POLL_TELEMETRY`](#poll_telemetry-0x7d) | `0x7d` | Client → controller | ACK |

## Wire format

```text
[length: uint8][command: uint8][message_id: uint16][payload]
```

Multi-byte fields are little-endian. Length counts all bytes after itself. Parameter tables list payload fields in wire order and exclude the shared header.

- **ACK:** an empty response that echoes the command and message ID.
- **Errors:** [`ERROR`](#error-0x7f) echoes the message ID and identifies the failed command.
- **Initialization:** [`INIT`](#init-0x70) returns identity, then [`READY`](#ready-0x73) after clock setup.
- **Time sync:** [`TIME_SYNC_RESPONSE`](#time_sync_response-0x72) has no success acknowledgment.

See [response framing](docs/responses.md) and [time synchronization](docs/time-sync.md) for details.

## Commands

### INITIALIZE_MOTOR (0x01)

**Client → controller** · **Payload:** 2 bytes

This command initializes a motor and prepares it for use. Rejected with MOTOR_OWNED if the motor is currently owned by an active platform (one of its wheels), so platform wheels are not reconfigured out from under the platform.

#### Parameters

| Parameter | Type | Bytes | Range | Description |
| --- | --- | ---: | --- | --- |
| `motor_index` | `uint8_t` | 1 | 0 to 3 | The index of the motor to initialize. |
| `is_reversed` | `bool` | 1 | — | Whether or not the motor is reversed. |

#### Response

**ACK** — empty payload; echoes the command and message ID.

#### Errors

[`INVALID_LENGTH`](#error-invalid-length) (4) · [`INVALID_ARGUMENT`](#error-invalid-argument) (2) · [`INTERNAL_ERROR`](#error-internal-error) (6) · [`MOTOR_OWNED`](#error-motor-owned) (5) · [`INIT_REQUIRED`](#error-init-required) (15)

[Back to command index](#command-index)

---

### SET_MOTOR_SPEED (0x02)

**Client → controller** · **Payload:** 9 bytes

This command sets the speed of the specified motor in PWM. Rejected with MOTOR_OWNED if the motor is currently owned by an active platform (one of its wheels); use the platform velocity commands to drive platform wheels.

#### Parameters

| Parameter | Type | Bytes | Range | Description |
| --- | --- | ---: | --- | --- |
| `motor_index` | `uint8_t` | 1 | 0 to 3 | The index of the motor to set the speed for. |
| `pwm` | `double` | 8 | -100.0 to 100.0 | The speed of the motor. |

#### Response

**ACK** — empty payload; echoes the command and message ID.

#### Errors

[`INVALID_LENGTH`](#error-invalid-length) (4) · [`INVALID_ARGUMENT`](#error-invalid-argument) (2) · [`INTERNAL_ERROR`](#error-internal-error) (6) · [`MOTOR_OWNED`](#error-motor-owned) (5) · [`MOTOR_NOT_INITIALIZED`](#error-motor-not-initialized) (13) · [`INIT_REQUIRED`](#error-init-required) (15)

[Back to command index](#command-index)

---

### STOP_MOTOR (0x03)

**Client → controller** · **Payload:** 1 bytes

Coasts the motor to a stop: both H-bridge outputs are driven low, leaving the motor terminals open (high impedance) so it free-wheels and spins down gradually under its own friction. This also stops that motor's closed-loop speed controller if one is running (started via INITIALIZE_MOTOR_CONTROLLER), so the PID loop cannot re-drive the motor; to command the motor by target speed again you must re-initialize its controller. This is a single-motor command and is ignored if the motor is currently owned by an active platform (one of its wheels); to stop a platform, use STOP_PLATFORM_CONTROLLER, COAST_PLATFORM or BRAKE_PLATFORM instead. Use STOP_MOTOR for a soft, low-stress stop; use BRAKE_MOTOR when you need the motor to hold position and stop quickly.

#### Parameters

| Parameter | Type | Bytes | Range | Description |
| --- | --- | ---: | --- | --- |
| `motor_index` | `uint8_t` | 1 | 0 to 3 | The index of the motor to set the speed for. |

#### Response

**ACK** — empty payload; echoes the command and message ID.

#### Errors

[`INVALID_LENGTH`](#error-invalid-length) (4) · [`INVALID_ARGUMENT`](#error-invalid-argument) (2) · [`INTERNAL_ERROR`](#error-internal-error) (6) · [`MOTOR_OWNED`](#error-motor-owned) (5)

[Back to command index](#command-index)

---

### BRAKE_MOTOR (0x04)

**Client → controller** · **Payload:** 1 bytes

Actively brakes the motor (short brake): both H-bridge outputs are driven high, shorting the motor terminals together so the motor's own back-EMF resists rotation and it stops quickly and holds position. This also stops that motor's closed-loop speed controller if one is running (started via INITIALIZE_MOTOR_CONTROLLER), so the PID loop cannot re-drive the motor; to command the motor by target speed again you must re-initialize its controller. This is a single-motor command and is ignored if the motor is currently owned by an active platform (one of its wheels); to brake a platform, use BRAKE_PLATFORM (or STOP_PLATFORM_CONTROLLER / COAST_PLATFORM) instead. Use BRAKE_MOTOR for a fast, holding stop; use STOP_MOTOR to let the motor coast freely instead.

#### Parameters

| Parameter | Type | Bytes | Range | Description |
| --- | --- | ---: | --- | --- |
| `motor_index` | `uint8_t` | 1 | 0 to 3 | The index of the motor to set the speed for. |

#### Response

**ACK** — empty payload; echoes the command and message ID.

#### Errors

[`INVALID_LENGTH`](#error-invalid-length) (4) · [`INVALID_ARGUMENT`](#error-invalid-argument) (2) · [`INTERNAL_ERROR`](#error-internal-error) (6) · [`MOTOR_OWNED`](#error-motor-owned) (5)

[Back to command index](#command-index)

---

### INITIALIZE_MOTOR_CONTROLLER (0x05)

**Client → controller** · **Payload:** 44 bytes

This command sets the controller for the specified motor. Rejected with MOTOR_OWNED if the motor is currently owned by an active platform (one of its wheels), so it cannot create a competing controller on a platform wheel.

#### Parameters

| Parameter | Type | Bytes | Range | Description |
| --- | --- | ---: | --- | --- |
| `motor_index` | `uint8_t` | 1 | 0 to 3 | The index of the motor to set the controller for. |
| `is_reversed` | `bool` | 1 | — | Whether or not the motor is reversed. |
| `encoder_index` | `uint8_t` | 1 | 0 to 3 | The index of the encoder to use for the controller. |
| `is_encoder_reversed` | `bool` | 1 | — | Reverses the encoder counting direction, independently of the motor's is_reversed. For closed-loop control the encoder must report a positive measured speed when a positive speed is commanded (negative feedback); if the controller runs away, flip this flag. |
| `encoder_resolution` | `double` | 8 | — | Encoder resolution in ticks per revolution. The value can not be negative or zero. |
| `kp` | `double` | 8 | — | Proportional constant of PID |
| `ki` | `double` | 8 | — | Integral constant of PID |
| `kd` | `double` | 8 | — | Derivative constant of PID |
| `integral_limit` | `double` | 8 | — | Integral limit of PID controller. The value can not be negative or zero. If the value is zero or negative, the integral limit is disabled. |

#### Response

**ACK** — empty payload; echoes the command and message ID.

#### Errors

[`INVALID_LENGTH`](#error-invalid-length) (4) · [`INVALID_ARGUMENT`](#error-invalid-argument) (2) · [`INTERNAL_ERROR`](#error-internal-error) (6) · [`MOTOR_OWNED`](#error-motor-owned) (5) · [`INIT_REQUIRED`](#error-init-required) (15)

[Back to command index](#command-index)

---

### SET_MOTOR_TARGET_SPEED (0x06)

**Client → controller** · **Payload:** 9 bytes

This command sets the target speed for the specified motor in radians. Rejected with MOTOR_OWNED if the motor is currently owned by an active platform (one of its wheels); use SET_PLATFORM_TARGET_VELOCITY to drive platform wheels.

#### Parameters

| Parameter | Type | Bytes | Range | Description |
| --- | --- | ---: | --- | --- |
| `motor_index` | `uint8_t` | 1 | 0 to 3 | The index of the motor to set the target velocity for. |
| `speed` | `double` | 8 | — | The speed of the motor. |

#### Response

**ACK** — empty payload; echoes the command and message ID.

#### Errors

[`INVALID_LENGTH`](#error-invalid-length) (4) · [`INVALID_ARGUMENT`](#error-invalid-argument) (2) · [`INTERNAL_ERROR`](#error-internal-error) (6) · [`MOTOR_OWNED`](#error-motor-owned) (5) · [`CONTROLLER_NOT_INITIALIZED`](#error-controller-not-initialized) (14) · [`INIT_REQUIRED`](#error-init-required) (15)

[Back to command index](#command-index)

---

### RESET_MOTOR_CONTROLLER (0x07)

**Client → controller** · **Payload:** 1 bytes

This command resets the closed-loop controller for the specified motor: it clears the accumulated PID state (integrator windup, derivative history and internal output) and re-zeros the target speed, while keeping the controller running with its existing tuning (kp/ki/kd). Use it to recover from integrator windup or to bring a motor cleanly to a stop without deleting and re-initializing the controller. No effect if no controller is running for that motor, and ignored if the motor is currently owned by an active platform (one of its wheels).

#### Parameters

| Parameter | Type | Bytes | Range | Description |
| --- | --- | ---: | --- | --- |
| `motor_index` | `uint8_t` | 1 | 0 to 3 | The index of the motor to reset the controller for. |

#### Response

**ACK** — empty payload; echoes the command and message ID.

#### Errors

[`INVALID_LENGTH`](#error-invalid-length) (4) · [`INVALID_ARGUMENT`](#error-invalid-argument) (2) · [`INTERNAL_ERROR`](#error-internal-error) (6) · [`MOTOR_OWNED`](#error-motor-owned) (5) · [`INIT_REQUIRED`](#error-init-required) (15)

[Back to command index](#command-index)

---

### GET_MOTOR_CONTROLLER_STATE (0x08)

**Client → controller** · **Payload:** 1 bytes

This command gets the state of the controller for the specified motor.

#### Parameters

| Parameter | Type | Bytes | Range | Description |
| --- | --- | ---: | --- | --- |
| `motor_index` | `uint8_t` | 1 | 0 to 3 | The index of the motor to get the state for. |

#### Response

**Controller → client** · `motor_controller_state` · **Payload:** 57 bytes

The state of the controller for the specified motor.

| Parameter | Type | Bytes | Description |
| --- | --- | ---: | --- |
| `motor_index` | `int8_t` | 1 | Index of the motor with the controller. |
| `kp` | `double` | 8 | Proportional constant of PID |
| `ki` | `double` | 8 | Integral constant of PID |
| `kd` | `double` | 8 | Derivative constant of PID |
| `target_speed` | `double` | 8 | The target speed of the motor. |
| `current_speed` | `double` | 8 | The current speed of the motor. |
| `error` | `double` | 8 | The error of the motor. |
| `output` | `double` | 8 | The output of the motor. |

#### Errors

[`INVALID_LENGTH`](#error-invalid-length) (4) · [`INVALID_ARGUMENT`](#error-invalid-argument) (2) · [`INTERNAL_ERROR`](#error-internal-error) (6) · [`INIT_REQUIRED`](#error-init-required) (15)

[Back to command index](#command-index)

---

### DELETE_MOTOR_CONTROLLER (0x09)

**Client → controller** · **Payload:** 1 bytes

This command deletes the controller for the specified motor. Rejected with MOTOR_OWNED if the motor is currently owned by an active platform (one of its wheels); use STOP_PLATFORM_CONTROLLER to stop the platform controller instead.

#### Parameters

| Parameter | Type | Bytes | Range | Description |
| --- | --- | ---: | --- | --- |
| `motor_index` | `uint8_t` | 1 | 0 to 3 | The index of the motor to delete the controller for. |

#### Response

**ACK** — empty payload; echoes the command and message ID.

#### Errors

[`INVALID_LENGTH`](#error-invalid-length) (4) · [`INVALID_ARGUMENT`](#error-invalid-argument) (2) · [`INTERNAL_ERROR`](#error-internal-error) (6) · [`MOTOR_OWNED`](#error-motor-owned) (5) · [`INIT_REQUIRED`](#error-init-required) (15)

[Back to command index](#command-index)

---

### SET_CONTROLLER_FREQUENCY (0x0A)

**Client → controller** · **Payload:** 2 bytes

This command sets the global update frequency (in Hz) of the closed-loop motor controller task. All motor controllers share a single control loop, so this frequency is global and affects every currently running controller as well as any created afterwards; the PID sampling time is updated to match. The requested value is clamped to the supported range of 1 to 1000 Hz (the 1000 Hz maximum is bounded by the 1 ms RTOS tick). The value is then quantized to the 1 ms RTOS tick (period_ms = 1000 / frequency), so effective frequencies are 1000/N Hz. A value of 0 is invalid and ignored. Defaults to 10 Hz (100 ms) at start-up.

#### Parameters

| Parameter | Type | Bytes | Range | Description |
| --- | --- | ---: | --- | --- |
| `frequency` | `uint16_t` | 2 | 1 to 1000 | The controller update frequency in Hz. Valid range 1 to 1000 Hz; values outside are clamped, and 0 is ignored. |

#### Response

**ACK** — empty payload; echoes the command and message ID.

#### Errors

[`INVALID_LENGTH`](#error-invalid-length) (4) · [`INVALID_ARGUMENT`](#error-invalid-argument) (2) · [`INTERNAL_ERROR`](#error-internal-error) (6) · [`INIT_REQUIRED`](#error-init-required) (15)

[Back to command index](#command-index)

---

### GET_CONTROLLER_FREQUENCY (0x0B)

**Client → controller** · **Payload:** 0 bytes

This command retrieves the current global update frequency (in Hz) of the closed-loop motor controller task.

#### Parameters

No payload parameters.

#### Response

**Controller → client** · `uint16_t` · **Payload:** 2 bytes

The current controller update frequency in Hz (1 to 1000 Hz).

| Parameter | Type | Bytes | Description |
| --- | --- | ---: | --- |
| `frequency` | `uint16_t` | 2 | The current controller update frequency in Hz (1 to 1000 Hz). |

#### Errors

[`INVALID_LENGTH`](#error-invalid-length) (4) · [`INVALID_ARGUMENT`](#error-invalid-argument) (2) · [`INTERNAL_ERROR`](#error-internal-error) (6) · [`INIT_REQUIRED`](#error-init-required) (15)

[Back to command index](#command-index)

---

### INITIALIZE_ENCODER (0x11)

**Client → controller** · **Payload:** 10 bytes

This command initializes an encoder and prepares it for use.

#### Parameters

| Parameter | Type | Bytes | Range | Description |
| --- | --- | ---: | --- | --- |
| `encoder_index` | `uint8_t` | 1 | 0 to 3 | The index of the encoder to initialize. |
| `encoder_resolution` | `double` | 8 | — | Encoder resolution in ticks per revolution. The value can not be negative or zero. |
| `is_reversed` | `bool` | 1 | — | Whether or not the encoder is reversed. |

#### Response

**ACK** — empty payload; echoes the command and message ID.

#### Errors

[`INVALID_LENGTH`](#error-invalid-length) (4) · [`INVALID_ARGUMENT`](#error-invalid-argument) (2) · [`INTERNAL_ERROR`](#error-internal-error) (6) · [`INIT_REQUIRED`](#error-init-required) (15)

[Back to command index](#command-index)

---

### GET_ENCODER_VALUE (0x12)

**Client → controller** · **Payload:** 1 bytes

This command retrieves the current value of the encoder.

#### Parameters

| Parameter | Type | Bytes | Range | Description |
| --- | --- | ---: | --- | --- |
| `encoder_index` | `uint8_t` | 1 | 0 to 3 | The index of the encoder to retrieve the value for. |

#### Response

**Controller → client** · `uint16_t` · **Payload:** 2 bytes

The current value of the encoder.

| Parameter | Type | Bytes | Description |
| --- | --- | ---: | --- |
| `encoderValue` | `uint16_t` | 2 | The current value of the encoder. |

#### Errors

[`INVALID_LENGTH`](#error-invalid-length) (4) · [`INVALID_ARGUMENT`](#error-invalid-argument) (2) · [`INTERNAL_ERROR`](#error-internal-error) (6) · [`ENCODER_NOT_INITIALIZED`](#error-encoder-not-initialized) (11) · [`INIT_REQUIRED`](#error-init-required) (15)

[Back to command index](#command-index)

---

### START_ENCODER_ODOMETRY (0x13)

**Client → controller** · **Payload:** 1 bytes

This command starts the odometry calculation for the specified encoder.

#### Parameters

| Parameter | Type | Bytes | Range | Description |
| --- | --- | ---: | --- | --- |
| `encoder_index` | `uint8_t` | 1 | 0 to 3 | The index of the encoder to start the odometry calculation for. |

#### Response

**ACK** — empty payload; echoes the command and message ID.

#### Errors

[`INVALID_LENGTH`](#error-invalid-length) (4) · [`INVALID_ARGUMENT`](#error-invalid-argument) (2) · [`INTERNAL_ERROR`](#error-internal-error) (6) · [`INIT_REQUIRED`](#error-init-required) (15) · [`CLOCK_NOT_READY`](#error-clock-not-ready) (7) · [`ENCODER_NOT_INITIALIZED`](#error-encoder-not-initialized) (11)

[Back to command index](#command-index)

---

### RESET_ENCODER_ODOMETRY (0x14)

**Client → controller** · **Payload:** 1 bytes

This command resets the odometry calculation for the specified encoder.

#### Parameters

| Parameter | Type | Bytes | Range | Description |
| --- | --- | ---: | --- | --- |
| `encoder_index` | `uint8_t` | 1 | 0 to 3 | The index of the encoder to reset the odometry calculation for. |

#### Response

**ACK** — empty payload; echoes the command and message ID.

#### Errors

[`INVALID_LENGTH`](#error-invalid-length) (4) · [`INVALID_ARGUMENT`](#error-invalid-argument) (2) · [`INTERNAL_ERROR`](#error-internal-error) (6) · [`INIT_REQUIRED`](#error-init-required) (15)

[Back to command index](#command-index)

---

### STOP_ENCODER_ODOMETRY (0x15)

**Client → controller** · **Payload:** 1 bytes

This command stops the odometry calculation for the specified encoder.

#### Parameters

| Parameter | Type | Bytes | Range | Description |
| --- | --- | ---: | --- | --- |
| `encoder_index` | `uint8_t` | 1 | 0 to 3 | The index of the encoder to stop the odometry calculation for. |

#### Response

**ACK** — empty payload; echoes the command and message ID.

#### Errors

[`INVALID_LENGTH`](#error-invalid-length) (4) · [`INVALID_ARGUMENT`](#error-invalid-argument) (2) · [`INTERNAL_ERROR`](#error-internal-error) (6) · [`INIT_REQUIRED`](#error-init-required) (15)

[Back to command index](#command-index)

---

### GET_ENCODER_ODOMETRY (0x16)

**Client → controller** · **Payload:** 1 bytes

This command retrieves the odometry of the specified encoder.

#### Parameters

| Parameter | Type | Bytes | Range | Description |
| --- | --- | ---: | --- | --- |
| `encoder_index` | `uint8_t` | 1 | 0 to 3 | The index of the encoder to retrieve the odometry for. |

#### Response

**Controller → client** · `encoder_odometry_sample` · **Payload:** 18 bytes

Timestamped odometry measurement. See errors for missing INIT, clock readiness, odometry initialization, and sample availability.

| Parameter | Type | Bytes | Description |
| --- | --- | ---: | --- |
| `timestamp_us` | `uint64_t` | 8 | Measurement timestamp in selected clock domain. |
| `clock_mode` | `uint8_t` | 1 | 0 uptime, 1 Unix wall time. |
| `clock_quality` | `uint8_t` | 1 | 1 valid, 2 stale. |
| `angle` | `double` | 8 | Encoder odometry radians. |

#### Errors

[`INVALID_LENGTH`](#error-invalid-length) (4) · [`INVALID_ARGUMENT`](#error-invalid-argument) (2) · [`INTERNAL_ERROR`](#error-internal-error) (6) · [`INIT_REQUIRED`](#error-init-required) (15) · [`CLOCK_NOT_READY`](#error-clock-not-ready) (7) · [`ODOMETRY_NOT_INITIALIZED`](#error-odometry-not-initialized) (9) · [`SAMPLE_NOT_AVAILABLE`](#error-sample-not-available) (10)

[Back to command index](#command-index)

---

### SET_ODOMETRY_FREQUENCY (0x17)

**Client → controller** · **Payload:** 2 bytes

This command sets the global update frequency (in Hz) of the odometry task. A single odometry task integrates all encoder and platform odometry, so this frequency is global. The requested value is clamped to the supported range of 1 to 1000 Hz (the 1000 Hz maximum is bounded by the 1 ms RTOS tick). The value is then quantized to the 1 ms RTOS tick (period_ms = 1000 / frequency), so effective frequencies are 1000/N Hz. A value of 0 is invalid and ignored. Defaults to 20 Hz (50 ms) at start-up. Rejected if the resulting calculation period exceeds half of any active subscription interval on either transport.

#### Parameters

| Parameter | Type | Bytes | Range | Description |
| --- | --- | ---: | --- | --- |
| `frequency` | `uint16_t` | 2 | 1 to 1000 | The odometry update frequency in Hz. Valid range 1 to 1000 Hz; values outside are clamped, and 0 is ignored. |

#### Response

**ACK** — empty payload; echoes the command and message ID.

#### Errors

[`INVALID_LENGTH`](#error-invalid-length) (4) · [`INVALID_ARGUMENT`](#error-invalid-argument) (2) · [`INTERNAL_ERROR`](#error-internal-error) (6) · [`INIT_REQUIRED`](#error-init-required) (15)

[Back to command index](#command-index)

---

### GET_ODOMETRY_FREQUENCY (0x18)

**Client → controller** · **Payload:** 0 bytes

This command retrieves the current global update frequency (in Hz) of the odometry task.

#### Parameters

No payload parameters.

#### Response

**Controller → client** · `uint16_t` · **Payload:** 2 bytes

The current odometry update frequency in Hz (1 to 1000 Hz).

| Parameter | Type | Bytes | Description |
| --- | --- | ---: | --- |
| `frequency` | `uint16_t` | 2 | The current odometry update frequency in Hz (1 to 1000 Hz). |

#### Errors

[`INVALID_LENGTH`](#error-invalid-length) (4) · [`INVALID_ARGUMENT`](#error-invalid-argument) (2) · [`INTERNAL_ERROR`](#error-internal-error) (6) · [`INIT_REQUIRED`](#error-init-required) (15)

[Back to command index](#command-index)

---

### INITIALIZE_GPIO_PIN (0x20)

**Client → controller** · **Payload:** 2 bytes

This command initializes a digital pin and prepares it for use.

#### Parameters

| Parameter | Type | Bytes | Description |
| --- | --- | ---: | --- |
| `pin_number` | `uint8_t` | 1 | The number of the pin to initialize. |
| `mode` | `uint8_t` | 1 | Set digital pin as input or output. Modes: 0 = INPUT_PULLDOWN, 1 = INPUT_PULLUP, 2 = INPUT_NOPULL, 3 = OUTPUT. |

#### Response

**ACK** — empty payload; echoes the command and message ID.

#### Errors

[`INVALID_LENGTH`](#error-invalid-length) (4) · [`INVALID_ARGUMENT`](#error-invalid-argument) (2) · [`INTERNAL_ERROR`](#error-internal-error) (6) · [`INIT_REQUIRED`](#error-init-required) (15)

[Back to command index](#command-index)

---

### SET_GPIO_PIN_STATE (0x21)

**Client → controller** · **Payload:** 2 bytes

This command sets the specified pin to a state.

#### Parameters

| Parameter | Type | Bytes | Description |
| --- | --- | ---: | --- |
| `pin_number` | `uint8_t` | 1 | The number of the pin to set to a state. |
| `state` | `uint8_t` | 1 | The state of the pin. 0 = LOW, 1 = HIGH. |

#### Response

**ACK** — empty payload; echoes the command and message ID.

#### Errors

[`INVALID_LENGTH`](#error-invalid-length) (4) · [`INVALID_ARGUMENT`](#error-invalid-argument) (2) · [`INTERNAL_ERROR`](#error-internal-error) (6) · [`INIT_REQUIRED`](#error-init-required) (15)

[Back to command index](#command-index)

---

### GET_GPIO_PIN_STATE (0x22)

**Client → controller** · **Payload:** 1 bytes

This command gets the state of the specified pin.

#### Parameters

| Parameter | Type | Bytes | Description |
| --- | --- | ---: | --- |
| `pin_number` | `uint8_t` | 1 | The number of the pin to get the state for. |

#### Response

**Controller → client** · `uint8_t` · **Payload:** 1 bytes

The state of the pin. 0 = LOW, 1 = HIGH.

| Parameter | Type | Bytes | Description |
| --- | --- | ---: | --- |
| `state` | `uint8_t` | 1 | The state of the pin. 0 = LOW, 1 = HIGH. |

#### Errors

[`INVALID_LENGTH`](#error-invalid-length) (4) · [`INVALID_ARGUMENT`](#error-invalid-argument) (2) · [`INTERNAL_ERROR`](#error-internal-error) (6) · [`INIT_REQUIRED`](#error-init-required) (15)

[Back to command index](#command-index)

---

### TOGGLE_GPIO_PIN_STATE (0x23)

**Client → controller** · **Payload:** 1 bytes

This command toggles the specified pin.

#### Parameters

| Parameter | Type | Bytes | Description |
| --- | --- | ---: | --- |
| `pin_number` | `uint8_t` | 1 | The number of the pin to toggle. |

#### Response

**ACK** — empty payload; echoes the command and message ID.

#### Errors

[`INVALID_LENGTH`](#error-invalid-length) (4) · [`INVALID_ARGUMENT`](#error-invalid-argument) (2) · [`INTERNAL_ERROR`](#error-internal-error) (6) · [`INIT_REQUIRED`](#error-init-required) (15)

[Back to command index](#command-index)

---

### SET_STATUS_LED_STATE (0x25)

**Client → controller** · **Payload:** 1 bytes

This command sets the status LED to a state.

#### Parameters

| Parameter | Type | Bytes | Description |
| --- | --- | ---: | --- |
| `state` | `uint8_t` | 1 | The state of the status LED. 0 = OFF, 1 = ON. |

#### Response

**ACK** — empty payload; echoes the command and message ID.

#### Errors

[`INVALID_LENGTH`](#error-invalid-length) (4) · [`INVALID_ARGUMENT`](#error-invalid-argument) (2) · [`INTERNAL_ERROR`](#error-internal-error) (6) · [`INIT_REQUIRED`](#error-init-required) (15)

[Back to command index](#command-index)

---

### TOGGLE_STATUS_LED_STATE (0x26)

**Client → controller** · **Payload:** 0 bytes

This command toggles the status LED.

#### Parameters

No payload parameters.

#### Response

**ACK** — empty payload; echoes the command and message ID.

#### Errors

[`INVALID_LENGTH`](#error-invalid-length) (4) · [`INVALID_ARGUMENT`](#error-invalid-argument) (2) · [`INTERNAL_ERROR`](#error-internal-error) (6) · [`INIT_REQUIRED`](#error-init-required) (15)

[Back to command index](#command-index)

---

### INITIALIZE_MECANUM_PLATFORM (0x30)

**Client → controller** · **Payload:** 40 bytes

This command initializes a mecanum (4-wheel) platform and prepares it for use. It uses motor and encoder indices 0, 1, 2 and 3 (one per wheel), which correspond to the is_reversed_0..3 and is_encoder_reversed_0..3 parameters. All four motor slots are occupied by this platform.

#### Parameters

| Parameter | Type | Bytes | Description |
| --- | --- | ---: | --- |
| `is_reversed_0` | `bool` | 1 | Determines if motor 0 is reversed. |
| `is_reversed_1` | `bool` | 1 | Determines if motor 1 is reversed. |
| `is_reversed_2` | `bool` | 1 | Determines if motor 2 is reversed. |
| `is_reversed_3` | `bool` | 1 | Determines if motor 3 is reversed. |
| `is_encoder_reversed_0` | `bool` | 1 | Reverses encoder 0 counting direction, independently of motor 0's is_reversed_0. Set so the closed-loop feedback is negative (flip if the wheel runs away when given a target speed). |
| `is_encoder_reversed_1` | `bool` | 1 | Reverses encoder 1 counting direction, independently of motor 1's is_reversed_1. Set so the closed-loop feedback is negative (flip if the wheel runs away when given a target speed). |
| `is_encoder_reversed_2` | `bool` | 1 | Reverses encoder 2 counting direction, independently of motor 2's is_reversed_2. Set so the closed-loop feedback is negative (flip if the wheel runs away when given a target speed). |
| `is_encoder_reversed_3` | `bool` | 1 | Reverses encoder 3 counting direction, independently of motor 3's is_reversed_3. Set so the closed-loop feedback is negative (flip if the wheel runs away when given a target speed). |
| `length` | `double` | 8 | Length of the platform in meters. |
| `width` | `double` | 8 | Width of the platform in meters. |
| `wheels_diameter` | `double` | 8 | Diameter of the robot wheels in meters. |
| `encoder_resolution` | `double` | 8 | Encoder resolution in ticks per revolution. The value can not be negative. If platform does not have encoders, the value should be set to zero. |

#### Response

**ACK** — empty payload; echoes the command and message ID.

#### Errors

[`INVALID_LENGTH`](#error-invalid-length) (4) · [`INVALID_ARGUMENT`](#error-invalid-argument) (2) · [`INTERNAL_ERROR`](#error-internal-error) (6) · [`INIT_REQUIRED`](#error-init-required) (15)

[Back to command index](#command-index)

---

### INITIALIZE_OMNI_PLATFORM (0x31)

**Client → controller** · **Payload:** 30 bytes

This command initializes an omni (3-wheel) platform and prepares it for use. It uses motor and encoder indices 0, 1 and 2 (one per wheel), which correspond to the is_reversed_0..2 and is_encoder_reversed_0..2 parameters. Motor index 3 is not used by this platform and stays free for other purposes.

#### Parameters

| Parameter | Type | Bytes | Description |
| --- | --- | ---: | --- |
| `is_reversed_0` | `bool` | 1 | Determines if motor 0 is reversed. |
| `is_reversed_1` | `bool` | 1 | Determines if motor 1 is reversed. |
| `is_reversed_2` | `bool` | 1 | Determines if motor 2 is reversed. |
| `is_encoder_reversed_0` | `bool` | 1 | Reverses encoder 0 counting direction, independently of motor 0's is_reversed_0. Set so the closed-loop feedback is negative (flip if the wheel runs away when given a target speed). |
| `is_encoder_reversed_1` | `bool` | 1 | Reverses encoder 1 counting direction, independently of motor 1's is_reversed_1. Set so the closed-loop feedback is negative (flip if the wheel runs away when given a target speed). |
| `is_encoder_reversed_2` | `bool` | 1 | Reverses encoder 2 counting direction, independently of motor 2's is_reversed_2. Set so the closed-loop feedback is negative (flip if the wheel runs away when given a target speed). |
| `wheels_diameter` | `double` | 8 | Diameter of the robot wheels in millimeters. |
| `robot_radius` | `double` | 8 | Distance berween the center of the robot and the center of the wheels in millimeters. |
| `encoder_resolution` | `double` | 8 | Encoder resolution in ticks per revolution. The value can not be negative. If platform does not have encoders, the value should be set to zero. |

#### Response

**ACK** — empty payload; echoes the command and message ID.

#### Errors

[`INVALID_LENGTH`](#error-invalid-length) (4) · [`INVALID_ARGUMENT`](#error-invalid-argument) (2) · [`INTERNAL_ERROR`](#error-internal-error) (6) · [`INIT_REQUIRED`](#error-init-required) (15)

[Back to command index](#command-index)

---

### INITIALIZE_DIFFERENTIAL_PLATFORM (0x32)

**Client → controller** · **Payload:** 28 bytes

This command initializes a differential (2-wheel) platform and prepares it for use. It uses motor and encoder index 0 for the left wheel and index 1 for the right wheel, which correspond to the is_reversed_0/1 and is_encoder_reversed_0/1 parameters. Motor indices 2 and 3 are not used by this platform and stay free for other purposes.

#### Parameters

| Parameter | Type | Bytes | Description |
| --- | --- | ---: | --- |
| `is_reversed_0` | `bool` | 1 | Determines if motor 0 is reversed. |
| `is_reversed_1` | `bool` | 1 | Determines if motor 1 is reversed. |
| `is_encoder_reversed_0` | `bool` | 1 | Reverses encoder 0 counting direction, independently of motor 0's is_reversed_0. Set so the closed-loop feedback is negative (flip if the wheel runs away when given a target speed). |
| `is_encoder_reversed_1` | `bool` | 1 | Reverses encoder 1 counting direction, independently of motor 1's is_reversed_1. Set so the closed-loop feedback is negative (flip if the wheel runs away when given a target speed). |
| `wheel_diameter` | `double` | 8 | Diameter of the robot wheels in meters. |
| `wheel_base` | `double` | 8 | Distance between the two wheels in meters. |
| `encoder_resolution` | `double` | 8 | Encoder resolution in ticks per revolution. The value can not be negative. If platform does not have encoders, the value should be set to zero. |

#### Response

**ACK** — empty payload; echoes the command and message ID.

#### Errors

[`INVALID_LENGTH`](#error-invalid-length) (4) · [`INVALID_ARGUMENT`](#error-invalid-argument) (2) · [`INTERNAL_ERROR`](#error-internal-error) (6) · [`INIT_REQUIRED`](#error-init-required) (15)

[Back to command index](#command-index)

---

### SET_PLATFORM_VELOCITY (0x40)

**Client → controller** · **Payload:** 24 bytes

This command sets the velocity for the platform in PWM.

#### Parameters

| Parameter | Type | Bytes | Range | Description |
| --- | --- | ---: | --- | --- |
| `x` | `double` | 8 | -100.0 to 100.0 | X component of platform velocity in PWM |
| `y` | `double` | 8 | -100.0 to 100.0 | Y component of platform velocity in PWM |
| `t` | `double` | 8 | -100.0 to 100.0 | Theta component of platform velocity in PWM |

#### Response

**ACK** — empty payload; echoes the command and message ID.

#### Errors

[`INVALID_LENGTH`](#error-invalid-length) (4) · [`INVALID_ARGUMENT`](#error-invalid-argument) (2) · [`INTERNAL_ERROR`](#error-internal-error) (6) · [`PLATFORM_NOT_INITIALIZED`](#error-platform-not-initialized) (12) · [`INIT_REQUIRED`](#error-init-required) (15)

[Back to command index](#command-index)

---

### START_PLATFORM_CONTROLLER (0x41)

**Client → controller** · **Payload:** 32 bytes

This command sets the controller for the platform.

#### Parameters

| Parameter | Type | Bytes | Description |
| --- | --- | ---: | --- |
| `kp` | `double` | 8 | Proportional constant of PID |
| `ki` | `double` | 8 | Integral constant of PID |
| `kd` | `double` | 8 | Derivative constant of PID |
| `integral_limit` | `double` | 8 | Integral limit of PID controller. The value can not be negative or zero. If the value is zero or negative, the integral limit is disabled. |

#### Response

**ACK** — empty payload; echoes the command and message ID.

#### Errors

[`INVALID_LENGTH`](#error-invalid-length) (4) · [`INVALID_ARGUMENT`](#error-invalid-argument) (2) · [`INTERNAL_ERROR`](#error-internal-error) (6) · [`PLATFORM_NOT_INITIALIZED`](#error-platform-not-initialized) (12) · [`INIT_REQUIRED`](#error-init-required) (15)

[Back to command index](#command-index)

---

### SET_PLATFORM_TARGET_VELOCITY (0x42)

**Client → controller** · **Payload:** 24 bytes

This command set the target velocity for the platform in meters per second.

#### Parameters

| Parameter | Type | Bytes | Description |
| --- | --- | ---: | --- |
| `x` | `double` | 8 | X component of platform velocity in meters per second |
| `y` | `double` | 8 | Y component of platform velocity in meters per second |
| `t` | `double` | 8 | Theta component of platform velocity in radians per second |

#### Response

**ACK** — empty payload; echoes the command and message ID.

#### Errors

[`INVALID_LENGTH`](#error-invalid-length) (4) · [`INVALID_ARGUMENT`](#error-invalid-argument) (2) · [`INTERNAL_ERROR`](#error-internal-error) (6) · [`PLATFORM_NOT_INITIALIZED`](#error-platform-not-initialized) (12) · [`CONTROLLER_NOT_INITIALIZED`](#error-controller-not-initialized) (14) · [`INIT_REQUIRED`](#error-init-required) (15)

[Back to command index](#command-index)

---

### GET_PLATFORM_CURRENT_VELOCITY (0x43)

**Client → controller** · **Payload:** 0 bytes

This command gets the current velocity of the platform in meters per second.

#### Parameters

No payload parameters.

#### Response

**Controller → client** · `platform_velocity` · **Payload:** 24 bytes

The current velocity of the platform in meters per second.

| Parameter | Type | Bytes | Description |
| --- | --- | ---: | --- |
| `x` | `double` | 8 | X component of platform velocity in meters per second |
| `y` | `double` | 8 | Y component of platform velocity in meters per second |
| `t` | `double` | 8 | Theta component of platform velocity in radians per second |

#### Errors

[`INVALID_LENGTH`](#error-invalid-length) (4) · [`INVALID_ARGUMENT`](#error-invalid-argument) (2) · [`INTERNAL_ERROR`](#error-internal-error) (6) · [`INIT_REQUIRED`](#error-init-required) (15)

[Back to command index](#command-index)

---

### STOP_PLATFORM_CONTROLLER (0x44)

**Client → controller** · **Payload:** 0 bytes

This command stops the controller for the platform.

#### Parameters

No payload parameters.

#### Response

**ACK** — empty payload; echoes the command and message ID.

#### Errors

[`INVALID_LENGTH`](#error-invalid-length) (4) · [`INVALID_ARGUMENT`](#error-invalid-argument) (2) · [`INTERNAL_ERROR`](#error-internal-error) (6)

[Back to command index](#command-index)

---

### START_PLATFORM_ODOMETRY (0x45)

**Client → controller** · **Payload:** 0 bytes

This command starts the odometry calculation for the platform.

#### Parameters

No payload parameters.

#### Response

**ACK** — empty payload; echoes the command and message ID.

#### Errors

[`INVALID_LENGTH`](#error-invalid-length) (4) · [`INVALID_ARGUMENT`](#error-invalid-argument) (2) · [`INTERNAL_ERROR`](#error-internal-error) (6) · [`INIT_REQUIRED`](#error-init-required) (15) · [`CLOCK_NOT_READY`](#error-clock-not-ready) (7) · [`PLATFORM_NOT_INITIALIZED`](#error-platform-not-initialized) (12)

[Back to command index](#command-index)

---

### RESET_PLATFORM_ODOMETRY (0x46)

**Client → controller** · **Payload:** 0 bytes

This command resets the odometry calculation for the platform.

#### Parameters

No payload parameters.

#### Response

**ACK** — empty payload; echoes the command and message ID.

#### Errors

[`INVALID_LENGTH`](#error-invalid-length) (4) · [`INVALID_ARGUMENT`](#error-invalid-argument) (2) · [`INTERNAL_ERROR`](#error-internal-error) (6) · [`INIT_REQUIRED`](#error-init-required) (15)

[Back to command index](#command-index)

---

### STOP_PLATFORM_ODOMETRY (0x47)

**Client → controller** · **Payload:** 0 bytes

This command stops the odometry calculation for the platform.

#### Parameters

No payload parameters.

#### Response

**ACK** — empty payload; echoes the command and message ID.

#### Errors

[`INVALID_LENGTH`](#error-invalid-length) (4) · [`INVALID_ARGUMENT`](#error-invalid-argument) (2) · [`INTERNAL_ERROR`](#error-internal-error) (6) · [`INIT_REQUIRED`](#error-init-required) (15)

[Back to command index](#command-index)

---

### GET_PLATFORM_ODOMETRY (0x48)

**Client → controller** · **Payload:** 0 bytes

This command retrieves the odometry of the platform in meters and radians.

#### Parameters

No payload parameters.

#### Response

**Controller → client** · `platform_odometry_sample` · **Payload:** 34 bytes

Timestamped odometry measurement. See errors for missing INIT, clock readiness, odometry initialization, and sample availability.

| Parameter | Type | Bytes | Description |
| --- | --- | ---: | --- |
| `timestamp_us` | `uint64_t` | 8 | Measurement timestamp in selected clock domain. |
| `clock_mode` | `uint8_t` | 1 | 0 uptime, 1 Unix wall time. |
| `clock_quality` | `uint8_t` | 1 | 1 valid, 2 stale. |
| `x` | `double` | 8 | X meters. |
| `y` | `double` | 8 | Y meters. |
| `t` | `double` | 8 | Heading radians. |

#### Errors

[`INVALID_LENGTH`](#error-invalid-length) (4) · [`INVALID_ARGUMENT`](#error-invalid-argument) (2) · [`INTERNAL_ERROR`](#error-internal-error) (6) · [`INIT_REQUIRED`](#error-init-required) (15) · [`CLOCK_NOT_READY`](#error-clock-not-ready) (7) · [`ODOMETRY_NOT_INITIALIZED`](#error-odometry-not-initialized) (9) · [`SAMPLE_NOT_AVAILABLE`](#error-sample-not-available) (10)

[Back to command index](#command-index)

---

### BRAKE_PLATFORM (0x49)

**Client → controller** · **Payload:** 0 bytes

This command actively brakes all of this platform's wheel motors (short brake) so they resist motion and hold position, and stops the platform velocity controller if it is running (you must call START_PLATFORM_CONTROLLER again to resume closed-loop platform control). Motors used outside this platform are not affected. The motors resist motion until a new command is issued.

#### Parameters

No payload parameters.

#### Response

**ACK** — empty payload; echoes the command and message ID.

#### Errors

[`INVALID_LENGTH`](#error-invalid-length) (4) · [`INVALID_ARGUMENT`](#error-invalid-argument) (2) · [`INTERNAL_ERROR`](#error-internal-error) (6)

[Back to command index](#command-index)

---

### COAST_PLATFORM (0x4A)

**Client → controller** · **Payload:** 0 bytes

This command lets all of this platform's wheel motors coast freely (high impedance) so they spin down without resistance, and stops the platform velocity controller if it is running (you must call START_PLATFORM_CONTROLLER again to resume closed-loop platform control). Motors used outside this platform are not affected. The motors spin down without resistance.

#### Parameters

No payload parameters.

#### Response

**ACK** — empty payload; echoes the command and message ID.

#### Errors

[`INVALID_LENGTH`](#error-invalid-length) (4) · [`INVALID_ARGUMENT`](#error-invalid-argument) (2) · [`INTERNAL_ERROR`](#error-internal-error) (6)

[Back to command index](#command-index)

---

### INIT (0x70)

**Client → controller** · **Payload:** 8 bytes

Exchange identity and start per-connection clock initialization. Wait for READY before odometry. Clients with wall-clock capability answer controller TIME_SYNC_REQUEST messages; other clients use uptime. Repeating INIT restarts clock setup without altering motor state.

#### Parameters

| Parameter | Type | Bytes | Description |
| --- | --- | ---: | --- |
| `sdk_type` | `uint8_t` | 1 | 0 unknown, 1 Python, 2 JavaScript, 3 Arduino. |
| `client_version_major` | `uint8_t` | 1 | Client library major version. |
| `client_version_minor` | `uint8_t` | 1 | Client library minor version. |
| `client_version_patch` | `uint8_t` | 1 | Client library patch version. |
| `protocol_major` | `uint8_t` | 1 | Client protocol major version. |
| `protocol_minor` | `uint8_t` | 1 | Minimum required protocol minor version. |
| `protocol_patch` | `uint8_t` | 1 | Client protocol patch version, informational; compatibility is determined by major and minor. |
| `client_capabilities` | `uint8_t` | 1 | Bit 0 (1): valid wall-clock time available. Bit 1 (2): client supports telemetry subscriptions. Combine as 3 for both; 0 for neither. Other bits must be zero. Declaring support does not start time synchronization or create subscriptions. |

#### Response

**Controller → client** · `init_response` · **Payload:** 15 bytes

Board revision, protocol version and firmware build identity. Failures use ERROR with the failed command ID.

| Parameter | Type | Bytes | Description |
| --- | --- | ---: | --- |
| `board_model` | `uint8_t` | 1 | 1 Kinisi motor controller. |
| `board_major` | `uint8_t` | 1 | Hardware revision major. |
| `board_minor` | `uint8_t` | 1 | Hardware revision minor. |
| `board_patch` | `uint8_t` | 1 | Hardware revision patch. |
| `protocol_major` | `uint8_t` | 1 | Protocol version major. |
| `protocol_minor` | `uint8_t` | 1 | Protocol version minor. |
| `protocol_patch` | `uint8_t` | 1 | Protocol version patch. |
| `firmware_build_high` | `uint32_t` | 4 | First eight hexadecimal digits of the firmware Git commit. |
| `firmware_build_low` | `uint32_t` | 4 | Next eight hexadecimal digits of the firmware Git commit. |

Clock setup finishes with [`READY`](#ready-0x73), using the same INIT message ID.

#### Errors

[`INVALID_LENGTH`](#error-invalid-length) (4) · [`INVALID_ARGUMENT`](#error-invalid-argument) (2) · [`INTERNAL_ERROR`](#error-internal-error) (6) · [`INCOMPATIBLE_PROTOCOL`](#error-incompatible-protocol) (1) · [`TIME_SYNC_FAILED`](#error-time-sync-failed) (8)

[Back to command index](#command-index)

---

### ERROR (0x7F)

**Controller → client** · **Payload:** 2 bytes

Response-only error message. Payload: failed command ID (uint8_t), error code (uint8_t). Uses the common length and command header; never accepted as a request.

#### Parameters

| Parameter | Type | Bytes | Description |
| --- | --- | ---: | --- |
| `failed_command` | `uint8_t` | 1 | Command ID of the failed request; zero if unavailable. |
| `error_code` | `uint8_t` | 1 | Shared nonzero protocol error code. |

#### Errors

None in this message's declared direction.

[Back to command index](#command-index)

---

### TIME_SYNC_REQUEST (0x71)

**Controller → client** · **Payload:** 0 bytes

Controller-initiated timing request. Client echoes the header message ID in TIME_SYNC_RESPONSE.

#### Parameters

No payload parameters.

#### Response

The client replies with [`TIME_SYNC_RESPONSE`](#time_sync_response-0x72) and the same message ID.

#### Errors

None in this message's declared direction.

[Back to command index](#command-index)

---

### TIME_SYNC_RESPONSE (0x72)

**Client → controller** · **Payload:** 16 bytes

Reply to a pending controller timing request; consumed without an ACK.

#### Parameters

| Parameter | Type | Bytes | Description |
| --- | --- | ---: | --- |
| `host_receive_us` | `uint64_t` | 8 | Client wall-clock Unix microseconds when TIME_SYNC_REQUEST was received. |
| `host_send_us` | `uint64_t` | 8 | Client wall-clock Unix microseconds immediately before its reply is sent. |

#### Response

No acknowledgment on success; the controller consumes this timing reply.

#### Errors

[`INVALID_LENGTH`](#error-invalid-length) (4) · [`INVALID_ARGUMENT`](#error-invalid-argument) (2)

[Back to command index](#command-index)

---

### READY (0x73)

**Controller → client** · **Payload:** 1 bytes

Initial clock setup completed; echoes INIT message ID. Client can now request odometry.

#### Parameters

| Parameter | Type | Bytes | Description |
| --- | --- | ---: | --- |
| `clock_mode` | `uint8_t` | 1 | 0 controller uptime; 1 synchronized Unix wall time. |

#### Errors

None in this message's declared direction.

[Back to command index](#command-index)

---

### SET_TIME_SYNC_INTERVAL (0x74)

**Client → controller** · **Payload:** 4 bytes

Set independent time-sync refresh interval for this connection. Default 30000 ms.

#### Parameters

| Parameter | Type | Bytes | Range | Description |
| --- | --- | ---: | --- | --- |
| `interval_ms` | `uint32_t` | 4 | 1000 to 3600000 | Periodic resynchronization interval in milliseconds. |

#### Response

**ACK** — empty payload; echoes the command and message ID.

#### Errors

[`INVALID_LENGTH`](#error-invalid-length) (4) · [`INVALID_ARGUMENT`](#error-invalid-argument) (2) · [`INTERNAL_ERROR`](#error-internal-error) (6) · [`INIT_REQUIRED`](#error-init-required) (15)

[Back to command index](#command-index)

---

### GET_TIME_STATUS (0x75)

**Client → controller** · **Payload:** 0 bytes

Read this connection's clock mode, quality, interval and age.

#### Parameters

No payload parameters.

#### Response

**Controller → client** · `time_status` · **Payload:** 14 bytes

Clock status. Quality 0 unready, 1 valid, 2 stale.

| Parameter | Type | Bytes | Description |
| --- | --- | ---: | --- |
| `clock_mode` | `uint8_t` | 1 | 0 uptime, 1 Unix wall time. |
| `clock_quality` | `uint8_t` | 1 | 0 unready, 1 valid, 2 stale. |
| `interval_ms` | `uint32_t` | 4 | Sync interval. |
| `last_sync_age_us` | `uint64_t` | 8 | Elapsed since successful sync; zero in uptime or unready mode. |

#### Errors

[`INVALID_LENGTH`](#error-invalid-length) (4) · [`INVALID_ARGUMENT`](#error-invalid-argument) (2) · [`INTERNAL_ERROR`](#error-internal-error) (6) · [`INIT_REQUIRED`](#error-init-required) (15)

[Back to command index](#command-index)

---

### PING (0x76)

**Client → controller** · **Payload:** 0 bytes

Refresh connection activity and receive an empty ACK. Any structurally valid client command also refreshes activity; malformed frames do not.

#### Parameters

No payload parameters.

#### Response

**ACK** — empty payload; echoes the command and message ID.

#### Errors

[`INVALID_LENGTH`](#error-invalid-length) (4) · [`INVALID_ARGUMENT`](#error-invalid-argument) (2) · [`INTERNAL_ERROR`](#error-internal-error) (6) · [`INIT_REQUIRED`](#error-init-required) (15)

[Back to command index](#command-index)

---

### SET_HEARTBEAT_CONFIG (0x77)

**Client → controller** · **Payload:** 5 bytes

Configure this session watchdog after READY. Timeout coasts all motors, clears subscriptions and requires a new INIT before further operations. Disabling also removes this session subscriptions. INIT resets to disabled, 500 ms.

#### Parameters

| Parameter | Type | Bytes | Range | Description |
| --- | --- | ---: | --- | --- |
| `enabled` | `bool` | 1 | — | Enable monitoring; disabled initially for compatibility. |
| `timeout_ms` | `uint32_t` | 4 | 100 to 60000 | Monotonic receive timeout. Default 500 ms; send PING after at most timeout/5 of idle time (100 ms with the default). |

#### Response

**ACK** — empty payload; echoes the command and message ID.

#### Errors

[`INVALID_LENGTH`](#error-invalid-length) (4) · [`INVALID_ARGUMENT`](#error-invalid-argument) (2) · [`INTERNAL_ERROR`](#error-internal-error) (6) · [`INIT_REQUIRED`](#error-init-required) (15) · [`CLOCK_NOT_READY`](#error-clock-not-ready) (7)

[Back to command index](#command-index)

---

### GET_HEARTBEAT_CONFIG (0x78)

**Client → controller** · **Payload:** 0 bytes

Read this session watchdog configuration.

#### Parameters

No payload parameters.

#### Response

**Controller → client** · `heartbeat_config` · **Payload:** 5 bytes

Read this session watchdog configuration.

| Parameter | Type | Bytes | Range | Description |
| --- | --- | ---: | --- | --- |
| `enabled` | `bool` | 1 | — | Enable monitoring; disabled initially for compatibility. |
| `timeout_ms` | `uint32_t` | 4 | 100 to 60000 | Monotonic receive timeout. Default 500 ms; send PING after at most timeout/5 of idle time (100 ms with the default). |

#### Errors

[`INVALID_LENGTH`](#error-invalid-length) (4) · [`INVALID_ARGUMENT`](#error-invalid-argument) (2) · [`INTERNAL_ERROR`](#error-internal-error) (6) · [`INIT_REQUIRED`](#error-init-required) (15)

[Back to command index](#command-index)

---

### SUBSCRIBE_ODOMETRY (0x79)

**Client → controller** · **Payload:** 5 bytes

Publish the latest completed sample using the requested scheduling interval without rounding it to calculation ticks. Delivery is subject to transport capacity and task scheduling. Requires READY, an enabled heartbeat and running odometry. Replaces an existing subscription for this source. Interval must be at least twice the calculation period. No renewal is needed. Stop/reset of calculation pauses samples until fresh measurements exist; unsubscribe, INIT, disconnect or watchdog timeout removes subscriptions.

#### Parameters

| Parameter | Type | Bytes | Range | Description |
| --- | --- | ---: | --- | --- |
| `source` | `uint8_t` | 1 | 0 to 4 | 0 through 3 select encoder odometry; 4 selects platform odometry. |
| `interval_ms` | `uint32_t` | 4 | 2 to 3600000 | Delivery interval, independent of calculation scheduling. |

#### Response

**ACK** — empty payload; echoes the command and message ID.

#### Errors

[`INVALID_LENGTH`](#error-invalid-length) (4) · [`INVALID_ARGUMENT`](#error-invalid-argument) (2) · [`INTERNAL_ERROR`](#error-internal-error) (6) · [`INIT_REQUIRED`](#error-init-required) (15) · [`CLOCK_NOT_READY`](#error-clock-not-ready) (7) · [`ODOMETRY_NOT_INITIALIZED`](#error-odometry-not-initialized) (9)

[Back to command index](#command-index)

---

### UNSUBSCRIBE_ODOMETRY (0x7a)

**Client → controller** · **Payload:** 1 bytes

Remove this source subscription; succeeds if already absent. Does not stop odometry calculation.

#### Parameters

| Parameter | Type | Bytes | Range | Description |
| --- | --- | ---: | --- | --- |
| `source` | `uint8_t` | 1 | 0 to 4 | 0 through 3 select encoder odometry; 4 selects platform odometry. |

#### Response

**ACK** — empty payload; echoes the command and message ID.

#### Errors

[`INVALID_LENGTH`](#error-invalid-length) (4) · [`INVALID_ARGUMENT`](#error-invalid-argument) (2) · [`INTERNAL_ERROR`](#error-internal-error) (6) · [`INIT_REQUIRED`](#error-init-required) (15)

[Back to command index](#command-index)

---

### ENCODER_ODOMETRY_EVENT (0x7b)

**Controller → client** · **Payload:** 19 bytes

Unsolicited latest encoder sample, message_id zero. No response is expected. Slow transports skip intermediate samples rather than queueing stale history.

#### Parameters

| Parameter | Type | Bytes | Description |
| --- | --- | ---: | --- |
| `encoder_index` | `uint8_t` | 1 | Encoder index. |
| `timestamp_us` | `uint64_t` | 8 | Measurement timestamp in selected clock domain. |
| `clock_mode` | `uint8_t` | 1 | 0 uptime, 1 Unix wall time. |
| `clock_quality` | `uint8_t` | 1 | 1 valid, 2 stale. |
| `angle` | `double` | 8 | Encoder odometry radians. |

#### Errors

None in this message's declared direction.

[Back to command index](#command-index)

---

### PLATFORM_ODOMETRY_EVENT (0x7c)

**Controller → client** · **Payload:** 34 bytes

Unsolicited latest platform sample, message_id zero. No response is expected. Timestamp is the measurement time, not the transmission time.

#### Parameters

| Parameter | Type | Bytes | Description |
| --- | --- | ---: | --- |
| `timestamp_us` | `uint64_t` | 8 | Measurement timestamp in selected clock domain. |
| `clock_mode` | `uint8_t` | 1 | 0 uptime, 1 Unix wall time. |
| `clock_quality` | `uint8_t` | 1 | 1 valid, 2 stale. |
| `x` | `double` | 8 | X meters. |
| `y` | `double` | 8 | Y meters. |
| `t` | `double` | 8 | Heading radians. |

#### Errors

None in this message's declared direction.

[Back to command index](#command-index)

---

### POLL_TELEMETRY (0x7d)

**Client → controller** · **Payload:** 0 bytes

I2C master service request. Allows at most one due odometry event before its empty ACK, so the master can clock out telemetry without waiting indefinitely when no sample is available. USB clients receive events automatically and do not need this command.

#### Parameters

No payload parameters.

#### Response

**ACK** — empty payload; echoes the command and message ID.

#### Errors

[`INVALID_LENGTH`](#error-invalid-length) (4) · [`INVALID_ARGUMENT`](#error-invalid-argument) (2) · [`INTERNAL_ERROR`](#error-internal-error) (6) · [`INIT_REQUIRED`](#error-init-required) (15) · [`CLOCK_NOT_READY`](#error-clock-not-ready) (7)

[Back to command index](#command-index)

---

## Error codes

Errors use the shared ERROR message. Each command above links to its possible errors below. `UNKNOWN_COMMAND` also covers unrecognized or incorrectly directed messages.

| Code | Error | Meaning |
| ---: | --- | --- |
| 1 | <a id="error-incompatible-protocol"></a>`INCOMPATIBLE_PROTOCOL` | The requested protocol version is incompatible with this firmware. |
| 2 | <a id="error-invalid-argument"></a>`INVALID_ARGUMENT` | A field is invalid, out of range, or inconsistent with the pending request. |
| 3 | <a id="error-unknown-command"></a>`UNKNOWN_COMMAND` | The received command ID is not accepted by the controller. |
| 4 | <a id="error-invalid-length"></a>`INVALID_LENGTH` | The complete message has an invalid header or payload length. |
| 5 | <a id="error-motor-owned"></a>`MOTOR_OWNED` | The motor belongs to a platform; use the corresponding platform command. |
| 6 | <a id="error-internal-error"></a>`INTERNAL_ERROR` | The controller could not execute or encode the operation. |
| 7 | <a id="error-clock-not-ready"></a>`CLOCK_NOT_READY` | Initial clock setup is incomplete; wait for READY. Uptime mode also supports READY. |
| 8 | <a id="error-time-sync-failed"></a>`TIME_SYNC_FAILED` | Initial time sync exhausted its attempts without a valid sample. |
| 9 | <a id="error-odometry-not-initialized"></a>`ODOMETRY_NOT_INITIALIZED` | Odometry is not running; start odometry before requesting a measurement. |
| 10 | <a id="error-sample-not-available"></a>`SAMPLE_NOT_AVAILABLE` | Odometry is running but has no measurement yet, including immediately after reset; retry after an update. |
| 11 | <a id="error-encoder-not-initialized"></a>`ENCODER_NOT_INITIALIZED` | Initialize the encoder before reading it or starting encoder odometry. |
| 12 | <a id="error-platform-not-initialized"></a>`PLATFORM_NOT_INITIALIZED` | Initialize a platform before performing this operation. |
| 13 | <a id="error-motor-not-initialized"></a>`MOTOR_NOT_INITIALIZED` | Initialize the motor before setting its speed. |
| 14 | <a id="error-controller-not-initialized"></a>`CONTROLLER_NOT_INITIALIZED` | Start or initialize the closed-loop controller before setting its target. |
| 15 | <a id="error-init-required"></a>`INIT_REQUIRED` | Send a valid INIT request before performing this connection-dependent operation. |
