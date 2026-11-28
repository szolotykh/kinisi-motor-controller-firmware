# Kinisi motor controller commands

**Protocol 2.25.0** · Generated from [commands.json](commands.json)

[Command index](#command-index) · [Wire format](#wire-format) · [Error codes](#error-codes)

> **Compatibility:** Protocol 2.x and protocol 1.x clients are incompatible.

## Command index

| Command | Code | Direction | Response |
| --- | --- | --- | --- |
| [`GET_TEMPERATURE`](#get_temperature-0x50) | `0x50` | Client → controller | `double` |
| [`GET_IMU`](#get_imu-0x51) | `0x51` | Client → controller | `imu_sample` |
| [`GET_MOTOR_CURRENT`](#get_motor_current-0x52) | `0x52` | Client → controller | `double` |
| [`GET_MOTOR_FAULTS`](#get_motor_faults-0x53) | `0x53` | Client → controller | `motor_fault_status` |
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
| [`SUBSCRIBE_ODOMETRY`](#subscribe_odometry-0x79) **Retired** | `0x79` | Client → controller | ACK |
| [`UNSUBSCRIBE_ODOMETRY`](#unsubscribe_odometry-0x7a) **Retired** | `0x7a` | Client → controller | ACK |
| [`ENCODER_ODOMETRY_EVENT`](#encoder_odometry_event-0x7b) **Retired** | `0x7b` | Controller → client | — |
| [`PLATFORM_ODOMETRY_EVENT`](#platform_odometry_event-0x7c) **Retired** | `0x7c` | Controller → client | — |
| [`POLL_TELEMETRY`](#poll_telemetry-0x7d) | `0x7d` | Client → controller | ACK |
| [`INITIALIZE_MOTOR_POSITION_CONTROLLER`](#initialize_motor_position_controller-0x0c) | `0x0C` | Client → controller | ACK |
| [`RESET_MOTOR_POSITION`](#reset_motor_position-0x0d) | `0x0D` | Client → controller | ACK |
| [`SET_MOTOR_POSITION`](#set_motor_position-0x0e) | `0x0E` | Client → controller | ACK |
| [`GET_MOTOR_POSITION`](#get_motor_position-0x0f) | `0x0F` | Client → controller | `double` |
| [`INITIALIZE_PLATFORM_POSITION_CONTROLLER`](#initialize_platform_position_controller-0x4b) | `0x4B` | Client → controller | ACK |
| [`RESET_PLATFORM_POSITION`](#reset_platform_position-0x4c) | `0x4C` | Client → controller | ACK |
| [`SET_PLATFORM_POSITION`](#set_platform_position-0x4d) | `0x4D` | Client → controller | ACK |
| [`INITIALIZE_MOTOR_POSITION_PID_CONTROLLER`](#initialize_motor_position_pid_controller-0x10) | `0x10` | Client → controller | ACK |
| [`INITIALIZE_PLATFORM_POSITION_PID_CONTROLLER`](#initialize_platform_position_pid_controller-0x4e) | `0x4E` | Client → controller | ACK |
| [`SUBSCRIBE`](#subscribe-0x80) | `0x80` | Client → controller | ACK |
| [`UNSUBSCRIBE`](#unsubscribe-0x81) | `0x81` | Client → controller | ACK |
| [`TELEMETRY_EVENT`](#telemetry_event-0x82) | `0x82` | Controller → client | — |
| [`SUBSCRIBE_WITH_MODE`](#subscribe_with_mode-0x83) | `0x83` | Client → controller | ACK |
| [`CLEAR_MOTOR_FAULTS`](#clear_motor_faults-0x54) | `0x54` | Client → controller | `motor_fault_status` |
| [`GET_MOTOR_CURRENT_LIMIT`](#get_motor_current_limit-0x55) | `0x55` | Client → controller | `motor_current_limit` |
| [`SET_MOTOR_CURRENT_LIMIT`](#set_motor_current_limit-0x56) | `0x56` | Client → controller | `motor_current_limit` |
| [`SUBSCRIBE_CONFIGURED`](#subscribe_configured-0x84) | `0x84` | Client → controller | ACK |
| [`GET_PWM_FREQUENCY`](#get_pwm_frequency-0x57) | `0x57` | Client → controller | `uint32_t` |
| [`SET_PWM_FREQUENCY`](#set_pwm_frequency-0x58) | `0x58` | Client → controller | `uint32_t` |
| [`SAVE_CONFIGURATION`](#save_configuration-0x59) | `0x59` | Client → controller | ACK |
| [`GET_CONFIGURATION_FIELD`](#get_configuration_field-0x5a) | `0x5A` | Client → controller | `double` |
| [`SET_DRIVER_SLEEP`](#set_driver_sleep-0x5b) | `0x5B` | Client → controller | ACK |
| [`GET_DRIVER_SLEEP`](#get_driver_sleep-0x5c) | `0x5C` | Client → controller | `uint8_t` |
| [`SET_BOARD_IDLE`](#set_board_idle-0x5d) | `0x5D` | Client → controller | ACK |
| [`GET_BOARD_IDLE`](#get_board_idle-0x5e) | `0x5E` | Client → controller | `uint8_t` |
| [`SET_FUSION_EXPERIMENT`](#set_fusion_experiment-0x5f) | `0x5f` | Client → controller | ACK |
| [`GET_FUSION_STATUS`](#get_fusion_status-0x60) | `0x60` | Client → controller | `fusion_status` |
| [`GET_FUSION_VECTOR`](#get_fusion_vector-0x61) | `0x61` | Client → controller | `fusion_vector` |
| [`GET_FUSION_CONFIGURATION`](#get_fusion_configuration-0x62) | `0x62` | Client → controller | `fusion_configuration` |
| [`SET_IMU_TRACKING`](#set_imu_tracking-0x63) | `0x63` | Client → controller | ACK |
| [`GET_IMU_TRACKING_STATUS`](#get_imu_tracking_status-0x64) | `0x64` | Client → controller | `imu_tracking_status` |
| [`GET_IMU_TRACKING_VECTOR`](#get_imu_tracking_vector-0x65) | `0x65` | Client → controller | `imu_tracking_vector` |
| [`GET_IMU_TRACKING_CONFIGURATION`](#get_imu_tracking_configuration-0x66) | `0x66` | Client → controller | `imu_tracking_configuration` |
| [`SET_FEEDBACK_SOURCE`](#set_feedback_source-0x67) | `0x67` | Client → controller | ACK |
| [`GET_FEEDBACK_SOURCE_STATUS`](#get_feedback_source_status-0x68) | `0x68` | Client → controller | `feedback_source_status` |
| [`GET_FEEDBACK_SOURCE_VECTOR`](#get_feedback_source_vector-0x69) | `0x69` | Client → controller | `feedback_source_vector` |
| [`SET_FEEDBACK_FUSION_MOUNT`](#set_feedback_fusion_mount-0x6a) | `0x6a` | Client → controller | ACK |
| [`GET_FEEDBACK_FUSION_MOUNT`](#get_feedback_fusion_mount-0x6b) | `0x6b` | Client → controller | `feedback_fusion_mount` |
| [`GET_FEEDBACK_SOURCE_HEALTH`](#get_feedback_source_health-0x6c) | `0x6c` | Client → controller | `feedback_source_health` |
| [`GET_FEEDBACK_IMU_CHANNELS`](#get_feedback_imu_channels-0x6d) | `0x6d` | Client → controller | `feedback_imu_channels` |
| [`SET_IMU_MOUNT`](#set_imu_mount-0x6e) | `0x6e` | Client → controller | ACK |
| [`GET_IMU_MOUNT`](#get_imu_mount-0x6f) | `0x6f` | Client → controller | `imu_mount_configuration` |
| [`SET_BOARD_MOUNT`](#set_board_mount-0x85) | `0x85` | Client → controller | ACK |
| [`GET_BOARD_MOUNT`](#get_board_mount-0x86) | `0x86` | Client → controller | `board_mount_configuration` |
| [`INITIALIZE_MECANUM_PLATFORM_ORIENTED`](#initialize_mecanum_platform_oriented-0x87) | `0x87` | Client → controller | ACK |

## Command lifecycle

Commands are active unless marked **Retired**. Retired commands and events remain supported with their existing IDs, payloads and behavior for compatibility. Use their replacements for new code when the connected firmware supports them. Removal is allowed only in a later protocol major; the earliest removal major is recorded in the schema. Retirement does not automatically remove or reject a command.

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

## Subscription triggers

| ID | Name | Behavior |
| ---: | --- | --- |
| 0 | PERIODIC | Read at the polling interval. |
| 1 | ON_EVENT | Read promptly on a source event, with background diagnostic reconciliation. |

## Subscription delivery filters

| ID | Name | Behavior |
| ---: | --- | --- |
| 0 | ALL | Send each triggered update. |
| 1 | ON_CHANGE | Send changed values and optional maximum-silence refreshes. |

## PWM frequencies (Hz)

| ID | Name | Behavior |
| ---: | --- | --- |
| 10000 | KHZ_10 | 10 kHz |
| 20000 | KHZ_20 | 20 kHz |
| 25000 | KHZ_25 | 25 kHz |
| 50000 | KHZ_50 | 50 kHz |
| 100000 | KHZ_100 | 100 kHz |

## Subscription modes

| ID | Mode | Behavior |
| ---: | --- | --- |
| 0 | `PERIODIC` | Send the latest sample at each interval. |
| 1 | `ON_CHANGE` | Send an initial sample, then changed values. Check values at each interval; exclude timestamp and clock metadata from comparison. |

## Commands

### Telemetry topics

The selector is a uint8 enum. IDs remain stable within a protocol major.

| ID | Topic | Payload object | Bytes | Maximum index | Minimum interval (ms) |
| ---: | --- | --- | ---: | ---: | ---: |
| 1 | `ENCODER_ODOMETRY` | `encoder_odometry_sample` | 18 | 3 | 2 |
| 2 | `PLATFORM_ODOMETRY` | `platform_odometry_sample` | 34 | 0 | 2 |
| 3 | `TEMPERATURE` | `temperature_sample` | 18 | 255 | 250 |
| 4 | `IMU` | `imu_telemetry_sample` | 58 | 255 | 10 |
| 5 | `MOTOR_CURRENT` | `motor_current_sample` | 18 | 3 | 20 |
| 6 | `MOTOR_FAULTS` | `motor_fault_sample` | 20 | 3 | 100 |

Odometry also requires an interval at least twice the calculation period. Actual resource counts depend on the board.

#### encoder_odometry_sample

Odometry state with controller-captured timestamp and clock metadata.

| Parameter | Type | Bytes | Description |
| --- | --- | ---: | --- |
| `timestamp_us` | `uint64_t` | 8 | Measurement timestamp in selected clock domain. |
| `clock_mode` | `uint8_t` | 1 | 0 uptime, 1 Unix wall time. |
| `clock_quality` | `uint8_t` | 1 | 1 valid, 2 stale. |
| `angle` | `double` | 8 | Encoder odometry radians. |

#### platform_odometry_sample

Odometry state with controller-captured timestamp and clock metadata.

| Parameter | Type | Bytes | Description |
| --- | --- | ---: | --- |
| `timestamp_us` | `uint64_t` | 8 | Measurement timestamp in selected clock domain. |
| `clock_mode` | `uint8_t` | 1 | 0 uptime, 1 Unix wall time. |
| `clock_quality` | `uint8_t` | 1 | 1 valid, 2 stale. |
| `x` | `double` | 8 | X meters. |
| `y` | `double` | 8 | Y meters. |
| `t` | `double` | 8 | Heading radians. |

#### temperature_sample

Timestamped temperature register reading in degrees Celsius.

| Parameter | Type | Bytes | Description |
| --- | --- | ---: | --- |
| `timestamp_us` | `uint64_t` | 8 | Sensor register-read completion timestamp in the selected clock domain; not a hardware conversion timestamp. |
| `clock_mode` | `uint8_t` | 1 | 0 uptime, 1 Unix wall time. |
| `clock_quality` | `uint8_t` | 1 | 1 valid, 2 stale. |
| `celsius` | `double` | 8 | Temperature in degrees Celsius. |

#### imu_telemetry_sample

Timestamped IMU register reading. Acceleration and angular velocity are not synchronized with each other.

| Parameter | Type | Bytes | Description |
| --- | --- | ---: | --- |
| `timestamp_us` | `uint64_t` | 8 | Sensor register-read completion timestamp in the selected clock domain; not a hardware conversion timestamp. |
| `clock_mode` | `uint8_t` | 1 | 0 uptime, 1 Unix wall time. |
| `clock_quality` | `uint8_t` | 1 | 1 valid, 2 stale. |
| `acceleration_x` | `double` | 8 | X acceleration in m/s^2, including gravity. |
| `acceleration_y` | `double` | 8 | Y acceleration in m/s^2, including gravity. |
| `acceleration_z` | `double` | 8 | Z acceleration in m/s^2, including gravity. |
| `angular_velocity_x` | `double` | 8 | X angular velocity in rad/s. |
| `angular_velocity_y` | `double` | 8 | Y angular velocity in rad/s. |
| `angular_velocity_z` | `double` | 8 | Z angular velocity in rad/s. |

#### motor_current_sample

Timestamped filtered motor-current estimate.

| Parameter | Type | Bytes | Description |
| --- | --- | ---: | --- |
| `timestamp_us` | `uint64_t` | 8 | ADC read completion timestamp in the selected clock domain. |
| `clock_mode` | `uint8_t` | 1 | 0 uptime, 1 Unix wall time. |
| `clock_quality` | `uint8_t` | 1 | 1 valid, 2 stale. |
| `amps` | `double` | 8 | Nonnegative filtered high-side current estimate in amperes. |

#### motor_fault_sample

Timestamped motor-driver diagnostic status. Timestamp marks completion of the read; faults can be latched rather than new events.

| Parameter | Type | Bytes | Description |
| --- | --- | ---: | --- |
| `timestamp_us` | `uint64_t` | 8 | ADC read completion timestamp in the selected clock domain. |
| `clock_mode` | `uint8_t` | 1 | 0 uptime, 1 Unix wall time. |
| `clock_quality` | `uint8_t` | 1 | 1 valid, 2 stale. |
| `sleep_requested` | `bool` | 1 | The sleep control is asserted; reading never wakes the driver. |
| `status_valid` | `bool` | 1 | Driver diagnostic registers were read successfully. Detailed flags are unknown when false. |
| `fault_active` | `bool` | 1 | The fault input is asserted or a valid register reports a global fault. |
| `overtemperature_warning` | `bool` | 1 | Driver temperature warning is reported. |
| `undervoltage` | `bool` | 1 | Driver supply undervoltage is reported; this does not prove a disconnected supply. |
| `charge_pump_undervoltage` | `bool` | 1 | Driver charge-pump undervoltage is reported. |
| `overcurrent` | `bool` | 1 | An overcurrent fault is reported. |
| `overtemperature_shutdown` | `bool` | 1 | A thermal shutdown fault is reported. |
| `open_load` | `bool` | 1 | The configured open-load detector reports a fault; detection depends on operating conditions. |
| `current_regulation` | `bool` | 1 | At least one output reports current regulation; this is not an overcurrent fault. |

### GET_TEMPERATURE (0x50)

**Client → controller** · **Payload:** 1 bytes

Read the latest board temperature in degrees Celsius. Sensor 0 is the TMP102 on v0.4. Sampling runs at 4 Hz; faster reads may repeat a sample. Returns SAMPLE_NOT_AVAILABLE during the first 300 ms after sensor initialization or recovery, SENSOR_NOT_AVAILABLE when the board has no such sensor, and INTERNAL_ERROR on I2C or configuration failure.

#### Parameters

| Parameter | Type | Bytes | Description |
| --- | --- | ---: | --- |
| `sensor_index` | `uint8_t` | 1 | Zero-based index in the board temperature_sensors array. |

#### Response

**Controller → client** · `double` · **Payload:** 8 bytes

Latest temperature in degrees Celsius, encoded as a little-endian IEEE 754 64-bit double.

| Parameter | Type | Bytes | Description |
| --- | --- | ---: | --- |
| `temperature_celsius` | `double` | 8 | Latest temperature in degrees Celsius, encoded as a little-endian IEEE 754 64-bit double. |

#### Errors

[`INVALID_LENGTH`](#error-invalid-length) (4) · [`INVALID_ARGUMENT`](#error-invalid-argument) (2) · [`INIT_REQUIRED`](#error-init-required) (15) · [`SENSOR_NOT_AVAILABLE`](#error-sensor-not-available) (16) · [`SAMPLE_NOT_AVAILABLE`](#error-sample-not-available) (10) · [`INTERNAL_ERROR`](#error-internal-error) (6)

[Back to command index](#command-index)

---

### GET_IMU (0x51)

**Client → controller** · **Payload:** 1 bytes

Read the latest accelerometer and gyroscope samples in native sensor axes. Acceleration is in m/s^2 and includes gravity; angular velocity is in rad/s. BMI088 uses +/-6 g and +/-500 degrees/s at 100 Hz. Each three-axis sample is coherent, but accel and gyro are not time-synchronized. Faster requests may repeat samples. No orientation or calibration is applied. Returns SAMPLE_NOT_AVAILABLE during the 50 ms settling interval after initialization or recovery, SENSOR_NOT_AVAILABLE for an absent sensor, and INTERNAL_ERROR on SPI, identity or configuration failure.

#### Parameters

| Parameter | Type | Bytes | Description |
| --- | --- | ---: | --- |
| `sensor_index` | `uint8_t` | 1 | Zero-based index in the board imu_sensors array. |

#### Response

**Controller → client** · `imu_sample` · **Payload:** 48 bytes

Six little-endian IEEE 754 doubles, 48 bytes total.

| Parameter | Type | Bytes | Description |
| --- | --- | ---: | --- |
| `acceleration_x` | `double` | 8 | X acceleration in m/s^2, including gravity. |
| `acceleration_y` | `double` | 8 | Y acceleration in m/s^2, including gravity. |
| `acceleration_z` | `double` | 8 | Z acceleration in m/s^2, including gravity. |
| `angular_velocity_x` | `double` | 8 | X angular velocity in rad/s. |
| `angular_velocity_y` | `double` | 8 | Y angular velocity in rad/s. |
| `angular_velocity_z` | `double` | 8 | Z angular velocity in rad/s. |

#### Errors

[`INVALID_LENGTH`](#error-invalid-length) (4) · [`INVALID_ARGUMENT`](#error-invalid-argument) (2) · [`INIT_REQUIRED`](#error-init-required) (15) · [`SENSOR_NOT_AVAILABLE`](#error-sensor-not-available) (16) · [`SAMPLE_NOT_AVAILABLE`](#error-sample-not-available) (10) · [`INTERNAL_ERROR`](#error-internal-error) (6)

[Back to command index](#command-index)

---

### GET_MOTOR_CURRENT (0x52)

**Client → controller** · **Payload:** 1 bytes

Read the filtered motor-current estimate in amperes. The unsigned reading measures driver high-side current; PWM decay intervals and the analog filter affect it. Reading does not enable or initialize the motor.

#### Parameters

| Parameter | Type | Bytes | Range | Description |
| --- | --- | ---: | --- | --- |
| `motor_index` | `uint8_t` | 1 | 0 to 3 | Motor connector index. |

#### Response

**Controller → client** · `double` · **Payload:** 8 bytes

Nonnegative current estimate in amperes, encoded as a little-endian IEEE 754 64-bit double.

| Parameter | Type | Bytes | Description |
| --- | --- | ---: | --- |
| `current_amps` | `double` | 8 | Nonnegative current estimate in amperes, encoded as a little-endian IEEE 754 64-bit double. |

#### Errors

[`INVALID_LENGTH`](#error-invalid-length) (4) · [`INVALID_ARGUMENT`](#error-invalid-argument) (2) · [`INIT_REQUIRED`](#error-init-required) (15) · [`SENSOR_NOT_AVAILABLE`](#error-sensor-not-available) (16) · [`SAMPLE_OUT_OF_RANGE`](#error-sample-out-of-range) (18) · [`INTERNAL_ERROR`](#error-internal-error) (6)

[Back to command index](#command-index)

---

### GET_MOTOR_FAULTS (0x53)

**Client → controller** · **Payload:** 1 bytes

Read motor-driver fault status without waking, initializing, enabling or clearing the driver. Detailed flags are meaningful only when status_valid is true. An unavailable SPI response cannot distinguish missing motor power from a communication or driver failure. Flags may retain latched faults until explicit motor reinitialization.

#### Parameters

| Parameter | Type | Bytes | Range | Description |
| --- | --- | ---: | --- | --- |
| `motor_index` | `uint8_t` | 1 | 0 to 3 | Motor connector index. |

#### Response

**Controller → client** · `motor_fault_status` · **Payload:** 10 bytes

Driver state and diagnostic flags, encoded as ten one-byte booleans.

| Parameter | Type | Bytes | Description |
| --- | --- | ---: | --- |
| `sleep_requested` | `bool` | 1 | The sleep control is asserted; reading never wakes the driver. |
| `status_valid` | `bool` | 1 | Driver diagnostic registers were read successfully. Detailed flags are unknown when false. |
| `fault_active` | `bool` | 1 | The fault input is asserted or a valid register reports a global fault. |
| `overtemperature_warning` | `bool` | 1 | Driver temperature warning is reported. |
| `undervoltage` | `bool` | 1 | Driver supply undervoltage is reported; this does not prove a disconnected supply. |
| `charge_pump_undervoltage` | `bool` | 1 | Driver charge-pump undervoltage is reported. |
| `overcurrent` | `bool` | 1 | An overcurrent fault is reported. |
| `overtemperature_shutdown` | `bool` | 1 | A thermal shutdown fault is reported. |
| `open_load` | `bool` | 1 | The configured open-load detector reports a fault; detection depends on operating conditions. |
| `current_regulation` | `bool` | 1 | At least one output reports current regulation; this is not an overcurrent fault. |

#### Errors

[`INVALID_LENGTH`](#error-invalid-length) (4) · [`INVALID_ARGUMENT`](#error-invalid-argument) (2) · [`INIT_REQUIRED`](#error-init-required) (15) · [`SENSOR_NOT_AVAILABLE`](#error-sensor-not-available) (16)

[Back to command index](#command-index)

---

### INITIALIZE_MOTOR (0x01)

**Client → controller** · **Payload:** 2 bytes

This command initializes a motor and prepares it for use. Rejected with MOTOR_OWNED if the motor is currently owned by an active platform (one of its wheels), so platform wheels are not reconfigured out from under the platform. Stops the motor velocity and position controllers before applying the command.

#### Parameters

| Parameter | Type | Bytes | Range | Description |
| --- | --- | ---: | --- | --- |
| `motor_index` | `uint8_t` | 1 | 0 to 3 | The index of the motor to initialize. |
| `is_reversed` | `bool` | 1 | — | Whether or not the motor is reversed. |

#### Response

**ACK** — empty payload; echoes the command and message ID.

#### Errors

[`DRIVERS_ASLEEP`](#error-drivers-asleep) (19) · [`INVALID_LENGTH`](#error-invalid-length) (4) · [`INVALID_ARGUMENT`](#error-invalid-argument) (2) · [`INTERNAL_ERROR`](#error-internal-error) (6) · [`MOTOR_OWNED`](#error-motor-owned) (5) · [`INIT_REQUIRED`](#error-init-required) (15)

[Back to command index](#command-index)

---

### SET_MOTOR_SPEED (0x02)

**Client → controller** · **Payload:** 9 bytes

This command sets the speed of the specified motor in PWM. Rejected with MOTOR_OWNED if the motor is currently owned by an active platform (one of its wheels); use the platform velocity commands to drive platform wheels. Stops the motor velocity and position controllers before applying the command.

#### Parameters

| Parameter | Type | Bytes | Range | Description |
| --- | --- | ---: | --- | --- |
| `motor_index` | `uint8_t` | 1 | 0 to 3 | The index of the motor to set the speed for. |
| `pwm` | `double` | 8 | -100.0 to 100.0 | The speed of the motor. |

#### Response

**ACK** — empty payload; echoes the command and message ID.

#### Errors

[`DRIVERS_ASLEEP`](#error-drivers-asleep) (19) · [`INVALID_LENGTH`](#error-invalid-length) (4) · [`INVALID_ARGUMENT`](#error-invalid-argument) (2) · [`INTERNAL_ERROR`](#error-internal-error) (6) · [`MOTOR_OWNED`](#error-motor-owned) (5) · [`MOTOR_NOT_INITIALIZED`](#error-motor-not-initialized) (13) · [`INIT_REQUIRED`](#error-init-required) (15)

[Back to command index](#command-index)

---

### STOP_MOTOR (0x03)

**Client → controller** · **Payload:** 1 bytes

Stops the motor's velocity and position controllers and lets the motor coast with the H-bridge outputs in high impedance. On dual-PWM boards both PWM control inputs are low. On V4 enable/direction boards DISABLE is high and EN duty is zero. The command ID and payload are the same for all boards; firmware selects the hardware behavior from the board configuration. A motor owned by an active platform is rejected with MOTOR_OWNED; use COAST_PLATFORM or STOP_PLATFORM_CONTROLLER for platform wheels. Re-initialize the motor controller before using target-speed control again. Use BRAKE_MOTOR for a short brake instead of coasting.

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

Stops the motor's velocity and position controllers and applies a short brake, connecting both motor terminals to the same bridge rail to resist rotation. On dual-PWM boards both PWM control inputs are high. On V4 enable/direction boards DISABLE is low and EN duty is zero; direction does not affect braking. The command ID and payload are the same for all boards; firmware selects the hardware behavior from the board configuration. Short braking does not regulate or hold a specific position. A motor owned by an active platform is rejected with MOTOR_OWNED; use BRAKE_PLATFORM for platform wheels. Re-initialize the motor controller before using target-speed control again. Use STOP_MOTOR to coast instead.

#### Parameters

| Parameter | Type | Bytes | Range | Description |
| --- | --- | ---: | --- | --- |
| `motor_index` | `uint8_t` | 1 | 0 to 3 | The index of the motor to set the speed for. |

#### Response

**ACK** — empty payload; echoes the command and message ID.

#### Errors

[`DRIVERS_ASLEEP`](#error-drivers-asleep) (19) · [`INVALID_LENGTH`](#error-invalid-length) (4) · [`INVALID_ARGUMENT`](#error-invalid-argument) (2) · [`INTERNAL_ERROR`](#error-internal-error) (6) · [`MOTOR_OWNED`](#error-motor-owned) (5)

[Back to command index](#command-index)

---

### INITIALIZE_MOTOR_CONTROLLER (0x05)

**Client → controller** · **Payload:** 44 bytes

This command sets the controller for the specified motor. Rejected with MOTOR_OWNED if the motor is currently owned by an active platform (one of its wheels), so it cannot create a competing controller on a platform wheel. Firmware 2.3.1 uses direct P+I+D PWM output with saturation anti-windup; retune gains from earlier builds.

#### Parameters

| Parameter | Type | Bytes | Range | Description |
| --- | --- | ---: | --- | --- |
| `motor_index` | `uint8_t` | 1 | 0 to 3 | The index of the motor to set the controller for. |
| `is_reversed` | `bool` | 1 | — | Whether or not the motor is reversed. |
| `encoder_index` | `uint8_t` | 1 | 0 to 3 | The index of the encoder to use for the controller. |
| `is_encoder_reversed` | `bool` | 1 | — | Reverses the encoder counting direction, independently of the motor's is_reversed. For closed-loop control the encoder must report a positive measured speed when a positive speed is commanded (negative feedback); if the controller runs away, flip this flag. |
| `encoder_resolution` | `double` | 8 | — | Encoder resolution in ticks per revolution. The value can not be negative or zero. |
| `kp` | `double` | 8 | — | kp gain (PWM percentage points per rad/s), finite and nonnegative. |
| `ki` | `double` | 8 | — | ki gain (PWM percentage points per rad), finite and nonnegative. |
| `kd` | `double` | 8 | — | kd gain (PWM percentage points per rad/s^2), finite and nonnegative. |
| `integral_limit` | `double` | 8 | 0 to 100 | Maximum absolute integral contribution in PWM percentage points (0..100). Zero disables integral action. |

#### Response

**ACK** — empty payload; echoes the command and message ID.

#### Errors

[`DRIVERS_ASLEEP`](#error-drivers-asleep) (19) · [`INVALID_LENGTH`](#error-invalid-length) (4) · [`INVALID_ARGUMENT`](#error-invalid-argument) (2) · [`INTERNAL_ERROR`](#error-internal-error) (6) · [`MOTOR_OWNED`](#error-motor-owned) (5) · [`INIT_REQUIRED`](#error-init-required) (15) · [`ENCODER_NOT_INITIALIZED`](#error-encoder-not-initialized) (11) · [`SAMPLE_NOT_AVAILABLE`](#error-sample-not-available) (10)

[Back to command index](#command-index)

---

### SET_MOTOR_TARGET_SPEED (0x06)

**Client → controller** · **Payload:** 9 bytes

This command sets the target speed for the specified motor in radians. Rejected with MOTOR_OWNED if the motor is currently owned by an active platform (one of its wheels); use SET_PLATFORM_TARGET_VELOCITY to drive platform wheels. Suspends position mode; a new SET_MOTOR_POSITION reactivates it.

#### Parameters

| Parameter | Type | Bytes | Range | Description |
| --- | --- | ---: | --- | --- |
| `motor_index` | `uint8_t` | 1 | 0 to 3 | The index of the motor to set the target velocity for. |
| `speed` | `double` | 8 | — | The speed of the motor. |

#### Response

**ACK** — empty payload; echoes the command and message ID.

#### Errors

[`DRIVERS_ASLEEP`](#error-drivers-asleep) (19) · [`INVALID_LENGTH`](#error-invalid-length) (4) · [`INVALID_ARGUMENT`](#error-invalid-argument) (2) · [`INTERNAL_ERROR`](#error-internal-error) (6) · [`MOTOR_OWNED`](#error-motor-owned) (5) · [`CONTROLLER_NOT_INITIALIZED`](#error-controller-not-initialized) (14) · [`INIT_REQUIRED`](#error-init-required) (15) · [`ENCODER_NOT_INITIALIZED`](#error-encoder-not-initialized) (11) · [`SAMPLE_NOT_AVAILABLE`](#error-sample-not-available) (10)

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

This command initializes an encoder and prepares it for use. Stops controllers using this encoder before changing its configuration.

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

This command initializes a mecanum (4-wheel) platform and prepares it for use. It uses motor and encoder indices 0, 1, 2 and 3 (one per wheel), which correspond to the is_reversed_0..3 and is_encoder_reversed_0..3 parameters. All four motor slots are occupied by this platform. Selects the STANDARD roller layout.

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

[`DRIVERS_ASLEEP`](#error-drivers-asleep) (19) · [`INVALID_LENGTH`](#error-invalid-length) (4) · [`INVALID_ARGUMENT`](#error-invalid-argument) (2) · [`INTERNAL_ERROR`](#error-internal-error) (6) · [`INIT_REQUIRED`](#error-init-required) (15)

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
| `wheels_diameter` | `double` | 8 | Diameter of the robot wheels in meters. |
| `robot_radius` | `double` | 8 | Distance from the robot center to each wheel center in meters. |
| `encoder_resolution` | `double` | 8 | Encoder resolution in ticks per revolution. The value can not be negative. If platform does not have encoders, the value should be set to zero. |

#### Response

**ACK** — empty payload; echoes the command and message ID.

#### Errors

[`DRIVERS_ASLEEP`](#error-drivers-asleep) (19) · [`INVALID_LENGTH`](#error-invalid-length) (4) · [`INVALID_ARGUMENT`](#error-invalid-argument) (2) · [`INTERNAL_ERROR`](#error-internal-error) (6) · [`INIT_REQUIRED`](#error-init-required) (15)

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

[`DRIVERS_ASLEEP`](#error-drivers-asleep) (19) · [`INVALID_LENGTH`](#error-invalid-length) (4) · [`INVALID_ARGUMENT`](#error-invalid-argument) (2) · [`INTERNAL_ERROR`](#error-internal-error) (6) · [`INIT_REQUIRED`](#error-init-required) (15)

[Back to command index](#command-index)

---

### SET_PLATFORM_VELOCITY (0x40)

**Client → controller** · **Payload:** 24 bytes

This command sets the velocity for the platform in PWM. Stops platform velocity and position control before applying PWM.

#### Parameters

| Parameter | Type | Bytes | Range | Description |
| --- | --- | ---: | --- | --- |
| `x` | `double` | 8 | -100.0 to 100.0 | X component of platform velocity in PWM |
| `y` | `double` | 8 | -100.0 to 100.0 | Y component of platform velocity in PWM |
| `t` | `double` | 8 | -100.0 to 100.0 | Theta component of platform velocity in PWM |

#### Response

**ACK** — empty payload; echoes the command and message ID.

#### Errors

[`DRIVERS_ASLEEP`](#error-drivers-asleep) (19) · [`INVALID_LENGTH`](#error-invalid-length) (4) · [`INVALID_ARGUMENT`](#error-invalid-argument) (2) · [`INTERNAL_ERROR`](#error-internal-error) (6) · [`PLATFORM_NOT_INITIALIZED`](#error-platform-not-initialized) (12) · [`INIT_REQUIRED`](#error-init-required) (15)

[Back to command index](#command-index)

---

### START_PLATFORM_CONTROLLER (0x41)

**Client → controller** · **Payload:** 32 bytes

This command sets the controller for the platform. Firmware 2.3.1 uses direct P+I+D PWM output with saturation anti-windup; retune gains from earlier builds.

#### Parameters

| Parameter | Type | Bytes | Range | Description |
| --- | --- | ---: | --- | --- |
| `kp` | `double` | 8 | — | kp gain (PWM percentage points per rad/s), finite and nonnegative. |
| `ki` | `double` | 8 | — | ki gain (PWM percentage points per rad), finite and nonnegative. |
| `kd` | `double` | 8 | — | kd gain (PWM percentage points per rad/s^2), finite and nonnegative. |
| `integral_limit` | `double` | 8 | 0 to 100 | Maximum absolute integral contribution in PWM percentage points (0..100). Zero disables integral action. |

#### Response

**ACK** — empty payload; echoes the command and message ID.

#### Errors

[`DRIVERS_ASLEEP`](#error-drivers-asleep) (19) · [`INVALID_LENGTH`](#error-invalid-length) (4) · [`INVALID_ARGUMENT`](#error-invalid-argument) (2) · [`INTERNAL_ERROR`](#error-internal-error) (6) · [`ENCODER_NOT_INITIALIZED`](#error-encoder-not-initialized) (11) · [`PLATFORM_NOT_INITIALIZED`](#error-platform-not-initialized) (12) · [`INIT_REQUIRED`](#error-init-required) (15) · [`SAMPLE_NOT_AVAILABLE`](#error-sample-not-available) (10)

[Back to command index](#command-index)

---

### SET_PLATFORM_TARGET_VELOCITY (0x42)

**Client → controller** · **Payload:** 24 bytes

This command set the target velocity for the platform in meters per second. Cancels platform position control; initialize it again before another pose target. With an active fusion experiment, rejects invalid or stale feedback and targets fenced by an earlier fault with SAMPLE_NOT_AVAILABLE; both experimental modes require 100 ms of observed unchanged configured counters for initial rest arming. Once armed, ordinary target updates remain accepted while moving. Wait for healthy feedback and submit a fresh target; stopped cold configuration can ACK while still WAITING.

#### Parameters

| Parameter | Type | Bytes | Description |
| --- | --- | ---: | --- |
| `x` | `double` | 8 | X component of platform velocity in meters per second |
| `y` | `double` | 8 | Y component of platform velocity in meters per second |
| `t` | `double` | 8 | Theta component of platform velocity in radians per second |

#### Response

**ACK** — empty payload; echoes the command and message ID.

#### Errors

[`DRIVERS_ASLEEP`](#error-drivers-asleep) (19) · [`INVALID_LENGTH`](#error-invalid-length) (4) · [`INVALID_ARGUMENT`](#error-invalid-argument) (2) · [`INTERNAL_ERROR`](#error-internal-error) (6) · [`PLATFORM_NOT_INITIALIZED`](#error-platform-not-initialized) (12) · [`CONTROLLER_NOT_INITIALIZED`](#error-controller-not-initialized) (14) · [`INIT_REQUIRED`](#error-init-required) (15) · [`SAMPLE_NOT_AVAILABLE`](#error-sample-not-available) (10) · [`ENCODER_NOT_INITIALIZED`](#error-encoder-not-initialized) (11)

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

[`INVALID_LENGTH`](#error-invalid-length) (4) · [`INVALID_ARGUMENT`](#error-invalid-argument) (2) · [`INTERNAL_ERROR`](#error-internal-error) (6) · [`INIT_REQUIRED`](#error-init-required) (15) · [`CLOCK_NOT_READY`](#error-clock-not-ready) (7) · [`ENCODER_NOT_INITIALIZED`](#error-encoder-not-initialized) (11) · [`PLATFORM_NOT_INITIALIZED`](#error-platform-not-initialized) (12)

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

Stops the platform velocity and position controllers and applies a short brake to this platform's wheel motors using the selected board's motor control model. Other motors are not affected. The brake resists motion until a new command is issued; it does not regulate or hold a specific position. Call START_PLATFORM_CONTROLLER again to resume closed-loop platform control.

#### Parameters

No payload parameters.

#### Response

**ACK** — empty payload; echoes the command and message ID.

#### Errors

[`DRIVERS_ASLEEP`](#error-drivers-asleep) (19) · [`INVALID_LENGTH`](#error-invalid-length) (4) · [`INVALID_ARGUMENT`](#error-invalid-argument) (2) · [`INTERNAL_ERROR`](#error-internal-error) (6)

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

> **Retired since protocol 2.6.0.** Use [`SUBSCRIBE`](#subscribe-0x80). Kept for compatibility; eligible for removal in protocol 3.0.0.

Create or replace this connection's subscription for an odometry source. Requires READY, enabled heartbeat and running odometry. The interval must be at least twice the calculation period. Stop or reset pauses delivery until a fresh sample is available.

#### Parameters

| Parameter | Type | Bytes | Range | Description |
| --- | --- | ---: | --- | --- |
| `source` | `uint8_t` | 1 | 0 to 4 | 0 through 3 select encoder odometry; 4 selects platform odometry. |
| `interval_ms` | `uint32_t` | 4 | 2 to 3600000 | Delivery interval, independent of calculation scheduling. |

#### Response

**ACK** — empty payload; echoes the command and message ID.

#### Errors

[`BOARD_IDLE`](#error-board-idle) (20) · [`INVALID_LENGTH`](#error-invalid-length) (4) · [`INVALID_ARGUMENT`](#error-invalid-argument) (2) · [`INTERNAL_ERROR`](#error-internal-error) (6) · [`INIT_REQUIRED`](#error-init-required) (15) · [`CLOCK_NOT_READY`](#error-clock-not-ready) (7) · [`ODOMETRY_NOT_INITIALIZED`](#error-odometry-not-initialized) (9)

[Back to command index](#command-index)

---

### UNSUBSCRIBE_ODOMETRY (0x7a)

**Client → controller** · **Payload:** 1 bytes

> **Retired since protocol 2.6.0.** Use [`UNSUBSCRIBE`](#unsubscribe-0x81). Kept for compatibility; eligible for removal in protocol 3.0.0.

Remove this connection's subscription for an odometry source. Succeeds if already absent and leaves odometry calculation running.

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

> **Retired since protocol 2.6.0.** Use [`TELEMETRY_EVENT`](#telemetry_event-0x82). Kept for compatibility; eligible for removal in protocol 3.0.0.

Deliver the latest encoder sample using message ID zero. No ACK is required. Slow transports skip intermediate samples.

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

> **Retired since protocol 2.6.0.** Use [`TELEMETRY_EVENT`](#telemetry_event-0x82). Kept for compatibility; eligible for removal in protocol 3.0.0.

Deliver the latest platform sample using message ID zero. No ACK is required. The timestamp records measurement time.

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

Allow an I2C master to read at most one due telemetry event followed by an empty ACK. Return only the ACK when no sample is available. USB clients receive events automatically.

#### Parameters

No payload parameters.

#### Response

**ACK** — empty payload; echoes the command and message ID.

#### Errors

[`INVALID_LENGTH`](#error-invalid-length) (4) · [`INVALID_ARGUMENT`](#error-invalid-argument) (2) · [`INTERNAL_ERROR`](#error-internal-error) (6) · [`INIT_REQUIRED`](#error-init-required) (15) · [`CLOCK_NOT_READY`](#error-clock-not-ready) (7)

[Back to command index](#command-index)

---

### INITIALIZE_MOTOR_POSITION_CONTROLLER (0x0C)

**Client → controller** · **Payload:** 25 bytes

Initialize a bounded proportional position loop over an already running motor velocity controller. Zero is the current encoder position; initially holds zero. Stop/delete/reinitialize of velocity control discards position tuning.

#### Parameters

| Parameter | Type | Bytes | Range | Description |
| --- | --- | ---: | --- | --- |
| `motor_index` | `uint8_t` | 1 | 0 to 3 | Motor index. |
| `kp` | `double` | 8 | — | Position gain in inverse seconds; must be positive. |
| `max_speed` | `double` | 8 | — | Maximum commanded speed in radians per second; must be positive. |
| `tolerance` | `double` | 8 | — | Position tolerance in radians; must be nonnegative. |

#### Response

**ACK** — empty payload; echoes the command and message ID.

#### Errors

[`DRIVERS_ASLEEP`](#error-drivers-asleep) (19) · [`INVALID_LENGTH`](#error-invalid-length) (4) · [`INVALID_ARGUMENT`](#error-invalid-argument) (2) · [`INTERNAL_ERROR`](#error-internal-error) (6) · [`INIT_REQUIRED`](#error-init-required) (15) · [`CONTROLLER_NOT_INITIALIZED`](#error-controller-not-initialized) (14) · [`MOTOR_OWNED`](#error-motor-owned) (5) · [`ENCODER_NOT_INITIALIZED`](#error-encoder-not-initialized) (11) · [`SAMPLE_NOT_AVAILABLE`](#error-sample-not-available) (10)

[Back to command index](#command-index)

---

### RESET_MOTOR_POSITION (0x0D)

**Client → controller** · **Payload:** 1 bytes

Zero the motor position and target at the current encoder count, clear velocity PID history and output, and hold zero. Requires initialized position control. Does not reset independent encoder odometry.

#### Parameters

| Parameter | Type | Bytes | Range | Description |
| --- | --- | ---: | --- | --- |
| `motor_index` | `uint8_t` | 1 | 0 to 3 | Motor index. |

#### Response

**ACK** — empty payload; echoes the command and message ID.

#### Errors

[`INVALID_LENGTH`](#error-invalid-length) (4) · [`INVALID_ARGUMENT`](#error-invalid-argument) (2) · [`INTERNAL_ERROR`](#error-internal-error) (6) · [`INIT_REQUIRED`](#error-init-required) (15) · [`CONTROLLER_NOT_INITIALIZED`](#error-controller-not-initialized) (14) · [`MOTOR_OWNED`](#error-motor-owned) (5)

[Back to command index](#command-index)

---

### SET_MOTOR_POSITION (0x0E)

**Client → controller** · **Payload:** 9 bytes

Set an absolute multi-turn angle in radians relative to the last position initialization/reset. Requires initialized position and velocity controllers. Reactivates position mode after a velocity override.

#### Parameters

| Parameter | Type | Bytes | Range | Description |
| --- | --- | ---: | --- | --- |
| `motor_index` | `uint8_t` | 1 | 0 to 3 | Motor index. |
| `position` | `double` | 8 | — | Absolute target in radians; not wrapped. |

#### Response

**ACK** — empty payload; echoes the command and message ID.

#### Errors

[`DRIVERS_ASLEEP`](#error-drivers-asleep) (19) · [`INVALID_LENGTH`](#error-invalid-length) (4) · [`INVALID_ARGUMENT`](#error-invalid-argument) (2) · [`INTERNAL_ERROR`](#error-internal-error) (6) · [`INIT_REQUIRED`](#error-init-required) (15) · [`CONTROLLER_NOT_INITIALIZED`](#error-controller-not-initialized) (14) · [`MOTOR_OWNED`](#error-motor-owned) (5) · [`ENCODER_NOT_INITIALIZED`](#error-encoder-not-initialized) (11) · [`SAMPLE_NOT_AVAILABLE`](#error-sample-not-available) (10)

[Back to command index](#command-index)

---

### GET_MOTOR_POSITION (0x0F)

**Client → controller** · **Payload:** 1 bytes

Read the latest motor position in radians in the position-controller frame. Requires initialized position control.

#### Parameters

| Parameter | Type | Bytes | Range | Description |
| --- | --- | ---: | --- | --- |
| `motor_index` | `uint8_t` | 1 | 0 to 3 | Motor index. |

#### Response

**Controller → client** · `double` · **Payload:** 8 bytes

Latest sampled continuous angle in radians.

| Parameter | Type | Bytes | Description |
| --- | --- | ---: | --- |
| `position` | `double` | 8 | Latest sampled continuous angle in radians. |

#### Errors

[`INVALID_LENGTH`](#error-invalid-length) (4) · [`INVALID_ARGUMENT`](#error-invalid-argument) (2) · [`INTERNAL_ERROR`](#error-internal-error) (6) · [`INIT_REQUIRED`](#error-init-required) (15) · [`CONTROLLER_NOT_INITIALIZED`](#error-controller-not-initialized) (14) · [`MOTOR_OWNED`](#error-motor-owned) (5)

[Back to command index](#command-index)

---

### INITIALIZE_PLATFORM_POSITION_CONTROLLER (0x4B)

**Client → controller** · **Payload:** 48 bytes

Initialize bounded pose control after START_PLATFORM_CONTROLLER. Starts odometry if needed, preserves its world frame and zeros velocity targets. No motion until SET_PLATFORM_POSITION. Supports omni, mecanum and differential bases.

#### Parameters

| Parameter | Type | Bytes | Description |
| --- | --- | ---: | --- |
| `linear_kp` | `double` | 8 | Translation gain in inverse seconds; positive. |
| `angular_kp` | `double` | 8 | Heading gain in inverse seconds; positive. |
| `max_linear_speed` | `double` | 8 | Maximum translation speed in meters per second; positive. |
| `max_angular_speed` | `double` | 8 | Maximum angular speed in radians per second; positive. |
| `position_tolerance` | `double` | 8 | Translation tolerance in meters; nonnegative. |
| `heading_tolerance` | `double` | 8 | Heading tolerance in radians; nonnegative. |

#### Response

**ACK** — empty payload; echoes the command and message ID.

#### Errors

[`DRIVERS_ASLEEP`](#error-drivers-asleep) (19) · [`INVALID_LENGTH`](#error-invalid-length) (4) · [`INVALID_ARGUMENT`](#error-invalid-argument) (2) · [`INTERNAL_ERROR`](#error-internal-error) (6) · [`INIT_REQUIRED`](#error-init-required) (15) · [`CONTROLLER_NOT_INITIALIZED`](#error-controller-not-initialized) (14) · [`PLATFORM_NOT_INITIALIZED`](#error-platform-not-initialized) (12) · [`ENCODER_NOT_INITIALIZED`](#error-encoder-not-initialized) (11) · [`SAMPLE_NOT_AVAILABLE`](#error-sample-not-available) (10)

[Back to command index](#command-index)

---

### RESET_PLATFORM_POSITION (0x4C)

**Client → controller** · **Payload:** 0 bytes

Cancel the pose target, zero velocity targets, and reset platform odometry to (0,0,0). Keeps position tuning. Wait for a fresh odometry sample before setting another target.

#### Parameters

No payload parameters.

#### Response

**ACK** — empty payload; echoes the command and message ID.

#### Errors

[`INVALID_LENGTH`](#error-invalid-length) (4) · [`INVALID_ARGUMENT`](#error-invalid-argument) (2) · [`INTERNAL_ERROR`](#error-internal-error) (6) · [`INIT_REQUIRED`](#error-init-required) (15) · [`CONTROLLER_NOT_INITIALIZED`](#error-controller-not-initialized) (14) · [`PLATFORM_NOT_INITIALIZED`](#error-platform-not-initialized) (12)

[Back to command index](#command-index)

---

### SET_PLATFORM_POSITION (0x4D)

**Client → controller** · **Payload:** 24 bytes

Set absolute (x,y,t) in the current odometry world frame: meters, meters, radians. Heading uses the shortest angular path. Requires initialized position control and fresh odometry. Differential bases approach the point before aligning final heading.

#### Parameters

| Parameter | Type | Bytes | Description |
| --- | --- | ---: | --- |
| `x` | `double` | 8 | World x position in meters. |
| `y` | `double` | 8 | World y position in meters. |
| `t` | `double` | 8 | World heading in radians, positive counterclockwise. |

#### Response

**ACK** — empty payload; echoes the command and message ID.

#### Errors

[`DRIVERS_ASLEEP`](#error-drivers-asleep) (19) · [`INVALID_LENGTH`](#error-invalid-length) (4) · [`INVALID_ARGUMENT`](#error-invalid-argument) (2) · [`INTERNAL_ERROR`](#error-internal-error) (6) · [`INIT_REQUIRED`](#error-init-required) (15) · [`CONTROLLER_NOT_INITIALIZED`](#error-controller-not-initialized) (14) · [`PLATFORM_NOT_INITIALIZED`](#error-platform-not-initialized) (12) · [`ODOMETRY_NOT_INITIALIZED`](#error-odometry-not-initialized) (9) · [`SAMPLE_NOT_AVAILABLE`](#error-sample-not-available) (10) · [`ENCODER_NOT_INITIALIZED`](#error-encoder-not-initialized) (11)

[Back to command index](#command-index)

---

### INITIALIZE_MOTOR_POSITION_PID_CONTROLLER (0x10)

**Client → controller** · **Payload:** 49 bytes

Initialize a position PID over the existing velocity controller. Initialize a bounded position PID loop over an already running motor velocity controller. Zero is the current encoder position; initially holds zero. Stop/delete/reinitialize of velocity control discards position tuning. Clears position PID history. Ki and Kd may be zero to disable I and D. Integral limits bound the integral velocity contribution.

#### Parameters

| Parameter | Type | Bytes | Range | Description |
| --- | --- | ---: | --- | --- |
| `motor_index` | `uint8_t` | 1 | 0 to 3 | Motor index. |
| `kp` | `double` | 8 | — | Position gain in inverse seconds; must be positive. |
| `max_speed` | `double` | 8 | — | Maximum commanded speed in radians per second; must be positive. |
| `tolerance` | `double` | 8 | — | Position tolerance in radians; must be nonnegative. |
| `ki` | `double` | 8 | — | Integral gain (1/s^2), nonnegative. |
| `kd` | `double` | 8 | — | Derivative gain, dimensionless and nonnegative. |
| `integral_limit` | `double` | 8 | — | Maximum absolute integral contribution (rad/s), nonnegative. |

#### Response

**ACK** — empty payload; echoes the command and message ID.

#### Errors

[`DRIVERS_ASLEEP`](#error-drivers-asleep) (19) · [`INVALID_LENGTH`](#error-invalid-length) (4) · [`INVALID_ARGUMENT`](#error-invalid-argument) (2) · [`INTERNAL_ERROR`](#error-internal-error) (6) · [`INIT_REQUIRED`](#error-init-required) (15) · [`CONTROLLER_NOT_INITIALIZED`](#error-controller-not-initialized) (14) · [`MOTOR_OWNED`](#error-motor-owned) (5) · [`ENCODER_NOT_INITIALIZED`](#error-encoder-not-initialized) (11) · [`SAMPLE_NOT_AVAILABLE`](#error-sample-not-available) (10)

[Back to command index](#command-index)

---

### INITIALIZE_PLATFORM_POSITION_PID_CONTROLLER (0x4E)

**Client → controller** · **Payload:** 96 bytes

Initialize a position PID over the existing velocity controller. Initialize bounded pose control after START_PLATFORM_CONTROLLER. Starts odometry if needed, preserves its world frame and zeros velocity targets. No motion until SET_PLATFORM_POSITION. Supports omni, mecanum and differential bases. Clears position PID history. Ki and Kd may be zero to disable I and D. Integral limits bound the integral velocity contribution.

#### Parameters

| Parameter | Type | Bytes | Description |
| --- | --- | ---: | --- |
| `linear_kp` | `double` | 8 | Translation gain in inverse seconds; positive. |
| `angular_kp` | `double` | 8 | Heading gain in inverse seconds; positive. |
| `max_linear_speed` | `double` | 8 | Maximum translation speed in meters per second; positive. |
| `max_angular_speed` | `double` | 8 | Maximum angular speed in radians per second; positive. |
| `position_tolerance` | `double` | 8 | Translation tolerance in meters; nonnegative. |
| `heading_tolerance` | `double` | 8 | Heading tolerance in radians; nonnegative. |
| `linear_ki` | `double` | 8 | Translation integral gain (1/s^2), nonnegative. |
| `linear_kd` | `double` | 8 | Translation derivative gain, dimensionless and nonnegative. |
| `linear_integral_limit` | `double` | 8 | Translation integral contribution limit (m/s), nonnegative. |
| `angular_ki` | `double` | 8 | Heading integral gain (1/s^2), nonnegative. |
| `angular_kd` | `double` | 8 | Heading derivative gain, dimensionless and nonnegative. |
| `angular_integral_limit` | `double` | 8 | Heading integral contribution limit (rad/s), nonnegative. |

#### Response

**ACK** — empty payload; echoes the command and message ID.

#### Errors

[`DRIVERS_ASLEEP`](#error-drivers-asleep) (19) · [`INVALID_LENGTH`](#error-invalid-length) (4) · [`INVALID_ARGUMENT`](#error-invalid-argument) (2) · [`INTERNAL_ERROR`](#error-internal-error) (6) · [`INIT_REQUIRED`](#error-init-required) (15) · [`CONTROLLER_NOT_INITIALIZED`](#error-controller-not-initialized) (14) · [`PLATFORM_NOT_INITIALIZED`](#error-platform-not-initialized) (12) · [`ENCODER_NOT_INITIALIZED`](#error-encoder-not-initialized) (11) · [`SAMPLE_NOT_AVAILABLE`](#error-sample-not-available) (10)

[Back to command index](#command-index)

---

### SUBSCRIBE (0x80)

**Client → controller** · **Payload:** 6 bytes

Create or replace this connection's subscription for a topic and resource. Requires READY and enabled heartbeat. Odometry must be running. The interval must meet the topic minimum and, for odometry, be at least twice the calculation period. Failed reads pause delivery until recovery.

#### Parameters

| Parameter | Type | Bytes | Range | Description |
| --- | --- | ---: | --- | --- |
| `topic_id` | `uint8_t` | 1 | — | Generated telemetry topic enum. |
| `index` | `uint8_t` | 1 | — | Resource index; platform uses 0. Hardware availability is checked by firmware. |
| `interval_ms` | `uint32_t` | 4 | 2 to 3600000 | Requested delivery interval in milliseconds; scheduling is best effort. |

#### Response

**ACK** — empty payload; echoes the command and message ID.

#### Errors

[`BOARD_IDLE`](#error-board-idle) (20) · [`INVALID_LENGTH`](#error-invalid-length) (4) · [`INVALID_ARGUMENT`](#error-invalid-argument) (2) · [`INIT_REQUIRED`](#error-init-required) (15) · [`CLOCK_NOT_READY`](#error-clock-not-ready) (7) · [`ODOMETRY_NOT_INITIALIZED`](#error-odometry-not-initialized) (9) · [`SENSOR_NOT_AVAILABLE`](#error-sensor-not-available) (16) · [`INTERNAL_ERROR`](#error-internal-error) (6) · [`SUBSCRIPTION_LIMIT`](#error-subscription-limit) (17) · [`SAMPLE_OUT_OF_RANGE`](#error-sample-out-of-range) (18)

[Back to command index](#command-index)

---

### UNSUBSCRIBE (0x81)

**Client → controller** · **Payload:** 2 bytes

Remove this connection's subscription for a topic and resource. Requires READY. Succeeds if already absent and leaves calculation running.

#### Parameters

| Parameter | Type | Bytes | Description |
| --- | --- | ---: | --- |
| `topic_id` | `uint8_t` | 1 | Generated telemetry topic enum. |
| `index` | `uint8_t` | 1 | Resource index; platform uses 0. Hardware availability is checked by firmware. |

#### Response

**ACK** — empty payload; echoes the command and message ID.

#### Errors

[`INVALID_LENGTH`](#error-invalid-length) (4) · [`INVALID_ARGUMENT`](#error-invalid-argument) (2) · [`INIT_REQUIRED`](#error-init-required) (15) · [`CLOCK_NOT_READY`](#error-clock-not-ready) (7)

[Back to command index](#command-index)

---

### TELEMETRY_EVENT (0x82)

**Controller → client** · **Payload:** 2 bytes fixed prefix + topic payload

Deliver the latest sample for a topic and resource using message ID zero. The topic selects the payload object. No ACK is required. Odometry timestamps mark acquisition; sensor timestamps mark register-read completion. Slow transports skip intermediate samples.

#### Parameters

| Parameter | Type | Bytes | Description |
| --- | --- | ---: | --- |
| `topic_id` | `uint8_t` | 1 | Generated telemetry topic enum. |
| `index` | `uint8_t` | 1 | Resource index; platform uses 0. Hardware availability is checked by firmware. |

The fixed prefix is followed by the topic's payload object from [telemetry topics](#telemetry-topics); total payload length varies by topic.

#### Errors

None in this message's declared direction.

[Back to command index](#command-index)

---

### SUBSCRIBE_WITH_MODE (0x83)

**Client → controller** · **Payload:** 7 bytes

Create or replace this connection's subscription with a delivery mode. Requires READY and enabled heartbeat. The interval sets the sampling cadence and must meet the topic minimum; odometry also requires twice the calculation period. ON_CHANGE sends an initial snapshot and changed values, excluding timestamp and clock metadata. Failed reads pause delivery; recovery sends a fresh snapshot.

#### Parameters

| Parameter | Type | Bytes | Range | Description |
| --- | --- | ---: | --- | --- |
| `topic_id` | `uint8_t` | 1 | — | Generated telemetry topic enum. |
| `index` | `uint8_t` | 1 | — | Resource index; platform uses 0. Hardware availability is checked by firmware. |
| `interval_ms` | `uint32_t` | 4 | 2 to 3600000 | Sampling interval in milliseconds; scheduling is best effort. |
| `mode` | `uint8_t` | 1 | — | Subscription delivery mode. |

#### Response

**ACK** — empty payload; echoes the command and message ID.

#### Errors

[`BOARD_IDLE`](#error-board-idle) (20) · [`INVALID_LENGTH`](#error-invalid-length) (4) · [`INVALID_ARGUMENT`](#error-invalid-argument) (2) · [`INIT_REQUIRED`](#error-init-required) (15) · [`CLOCK_NOT_READY`](#error-clock-not-ready) (7) · [`ODOMETRY_NOT_INITIALIZED`](#error-odometry-not-initialized) (9) · [`SENSOR_NOT_AVAILABLE`](#error-sensor-not-available) (16) · [`INTERNAL_ERROR`](#error-internal-error) (6) · [`SUBSCRIPTION_LIMIT`](#error-subscription-limit) (17) · [`SAMPLE_OUT_OF_RANGE`](#error-sample-out-of-range) (18)

[Back to command index](#command-index)

---

### CLEAR_MOTOR_FAULTS (0x54)

**Client → controller** · **Payload:** 1 bytes

Stop the initialized motor and its controller, clear latched driver faults, and return fresh diagnostics. Requires a ready connection. The motor remains stopped. Active conditions can remain reported. Platform-owned motors are rejected. Driver configuration is preserved; sleeping drivers are not woken. A failed clear or unreadable verification returns an error.

#### Parameters

| Parameter | Type | Bytes | Range | Description |
| --- | --- | ---: | --- | --- |
| `motor_index` | `uint8_t` | 1 | 0 to 3 | Motor connector index. |

#### Response

**Controller → client** · `motor_fault_status` · **Payload:** 10 bytes

Driver state and diagnostic flags, encoded as ten one-byte booleans.

| Parameter | Type | Bytes | Description |
| --- | --- | ---: | --- |
| `sleep_requested` | `bool` | 1 | The sleep control is asserted; reading never wakes the driver. |
| `status_valid` | `bool` | 1 | Driver diagnostic registers were read successfully. Detailed flags are unknown when false. |
| `fault_active` | `bool` | 1 | The fault input is asserted or a valid register reports a global fault. |
| `overtemperature_warning` | `bool` | 1 | Driver temperature warning is reported. |
| `undervoltage` | `bool` | 1 | Driver supply undervoltage is reported; this does not prove a disconnected supply. |
| `charge_pump_undervoltage` | `bool` | 1 | Driver charge-pump undervoltage is reported. |
| `overcurrent` | `bool` | 1 | An overcurrent fault is reported. |
| `overtemperature_shutdown` | `bool` | 1 | A thermal shutdown fault is reported. |
| `open_load` | `bool` | 1 | The configured open-load detector reports a fault; detection depends on operating conditions. |
| `current_regulation` | `bool` | 1 | At least one output reports current regulation; this is not an overcurrent fault. |

#### Errors

[`DRIVERS_ASLEEP`](#error-drivers-asleep) (19) · [`INVALID_LENGTH`](#error-invalid-length) (4) · [`INVALID_ARGUMENT`](#error-invalid-argument) (2) · [`INIT_REQUIRED`](#error-init-required) (15) · [`SENSOR_NOT_AVAILABLE`](#error-sensor-not-available) (16) · [`MOTOR_OWNED`](#error-motor-owned) (5) · [`MOTOR_NOT_INITIALIZED`](#error-motor-not-initialized) (13) · [`INTERNAL_ERROR`](#error-internal-error) (6) · [`CLOCK_NOT_READY`](#error-clock-not-ready) (7)

[Back to command index](#command-index)

---

### GET_MOTOR_CURRENT_LIMIT (0x55)

**Client → controller** · **Payload:** 1 bytes

Read the applied hardware current-regulation setting in milliamps. Requires a ready connection and an initialized, awake motor driver. Does not change motor state.

#### Parameters

| Parameter | Type | Bytes | Range | Description |
| --- | --- | ---: | --- | --- |
| `motor_index` | `uint8_t` | 1 | 0 to 3 | Motor connector index. |

#### Response

**Controller → client** · `motor_current_limit` · **Payload:** 3 bytes

Applied current-regulation setting.

| Parameter | Type | Bytes | Description |
| --- | --- | ---: | --- |
| `enabled` | `bool` | 1 | Current regulation is enabled on both outputs. |
| `threshold_ma` | `uint16_t` | 2 | Nominal threshold in milliamps. |

#### Errors

[`DRIVERS_ASLEEP`](#error-drivers-asleep) (19) · [`INVALID_LENGTH`](#error-invalid-length) (4) · [`INVALID_ARGUMENT`](#error-invalid-argument) (2) · [`INIT_REQUIRED`](#error-init-required) (15) · [`CLOCK_NOT_READY`](#error-clock-not-ready) (7) · [`MOTOR_NOT_INITIALIZED`](#error-motor-not-initialized) (13) · [`SENSOR_NOT_AVAILABLE`](#error-sensor-not-available) (16) · [`INTERNAL_ERROR`](#error-internal-error) (6)

[Back to command index](#command-index)

---

### SET_MOTOR_CURRENT_LIMIT (0x56)

**Client → controller** · **Payload:** 4 bytes

Stop the motor and its controller, apply and verify hardware current regulation, and return the applied setting. Requires a ready connection and an initialized, awake driver. Only board-supported thresholds are accepted. Platform-owned motors are rejected. The motor stays stopped. Settings are volatile; motor reinitialization applies board defaults. Driver power loss requires reinitialization. Overcurrent protection is unchanged.

#### Parameters

| Parameter | Type | Bytes | Range | Description |
| --- | --- | ---: | --- | --- |
| `motor_index` | `uint8_t` | 1 | 0 to 3 | Motor connector index. |
| `enabled` | `bool` | 1 | — | Enable current regulation on both motor outputs. |
| `threshold_ma` | `uint16_t` | 2 | 1 to 65535 | Nominal regulation threshold in milliamps, selected from the board profile. |

#### Response

**Controller → client** · `motor_current_limit` · **Payload:** 3 bytes

Applied current-regulation setting.

| Parameter | Type | Bytes | Description |
| --- | --- | ---: | --- |
| `enabled` | `bool` | 1 | Current regulation is enabled on both outputs. |
| `threshold_ma` | `uint16_t` | 2 | Nominal threshold in milliamps. |

#### Errors

[`DRIVERS_ASLEEP`](#error-drivers-asleep) (19) · [`INVALID_LENGTH`](#error-invalid-length) (4) · [`INVALID_ARGUMENT`](#error-invalid-argument) (2) · [`INIT_REQUIRED`](#error-init-required) (15) · [`CLOCK_NOT_READY`](#error-clock-not-ready) (7) · [`MOTOR_NOT_INITIALIZED`](#error-motor-not-initialized) (13) · [`SENSOR_NOT_AVAILABLE`](#error-sensor-not-available) (16) · [`INTERNAL_ERROR`](#error-internal-error) (6) · [`MOTOR_OWNED`](#error-motor-owned) (5)

[Back to command index](#command-index)

---

### SUBSCRIBE_CONFIGURED (0x84)

**Client → controller** · **Payload:** 12 bytes

Create or replace a subscription with independent trigger and delivery settings. Send an initial snapshot. PERIODIC uses poll_interval_ms; ON_EVENT requires zero polling interval and an event-capable source. ON_CHANGE suppresses unchanged values, excluding timestamp and clock metadata; max_silence_ms requests fresh refreshes, or zero disables refreshes. Delivery is best effort and requires READY and heartbeat.

#### Parameters

| Parameter | Type | Bytes | Range | Description |
| --- | --- | ---: | --- | --- |
| `topic_id` | `uint8_t` | 1 | — | Telemetry topic. |
| `index` | `uint8_t` | 1 | — | Resource index. |
| `poll_interval_ms` | `uint32_t` | 4 | 0 to 3600000 | Polling interval; zero for ON_EVENT. |
| `trigger` | `uint8_t` | 1 | — | Update trigger. |
| `delivery` | `uint8_t` | 1 | — | Delivery filter. |
| `max_silence_ms` | `uint32_t` | 4 | 0 to 3600000 | Fresh-sample refresh interval for ON_CHANGE; zero disables refreshes. Must meet the topic minimum. |

#### Response

**ACK** — empty payload; echoes the command and message ID.

#### Errors

[`BOARD_IDLE`](#error-board-idle) (20) · [`INVALID_LENGTH`](#error-invalid-length) (4) · [`INVALID_ARGUMENT`](#error-invalid-argument) (2) · [`INIT_REQUIRED`](#error-init-required) (15) · [`CLOCK_NOT_READY`](#error-clock-not-ready) (7) · [`ODOMETRY_NOT_INITIALIZED`](#error-odometry-not-initialized) (9) · [`SENSOR_NOT_AVAILABLE`](#error-sensor-not-available) (16) · [`INTERNAL_ERROR`](#error-internal-error) (6) · [`SUBSCRIPTION_LIMIT`](#error-subscription-limit) (17) · [`SAMPLE_OUT_OF_RANGE`](#error-sample-out-of-range) (18)

[Back to command index](#command-index)

---

### GET_PWM_FREQUENCY (0x57)

**Client → controller** · **Payload:** 0 bytes

Read the board-wide motor PWM frequency in hertz. Does not initialize motors or change outputs.

#### Parameters

No payload parameters.

#### Response

**Controller → client** · `uint32_t` · **Payload:** 4 bytes

Applied board-wide PWM frequency in hertz.

| Parameter | Type | Bytes | Description |
| --- | --- | ---: | --- |
| `frequency_hz` | `uint32_t` | 4 | Applied board-wide PWM frequency in hertz. |

#### Errors

[`INVALID_LENGTH`](#error-invalid-length) (4) · [`INVALID_ARGUMENT`](#error-invalid-argument) (2) · [`INIT_REQUIRED`](#error-init-required) (15) · [`CLOCK_NOT_READY`](#error-clock-not-ready) (7) · [`INTERNAL_ERROR`](#error-internal-error) (6)

[Back to command index](#command-index)

---

### SET_PWM_FREQUENCY (0x58)

**Client → controller** · **Payload:** 4 bytes

Stop all motors and motor/platform controllers, then set the board-wide PWM frequency. All motors remain stopped. The setting survives motor initialization and connection changes. At boot the board uses its default unless it restores an explicitly saved v0.4 configuration.

#### Parameters

| Parameter | Type | Bytes | Description |
| --- | --- | ---: | --- |
| `frequency_hz` | `uint32_t` | 4 | PWM frequency selected from the board-supported enum values. |

#### Response

**Controller → client** · `uint32_t` · **Payload:** 4 bytes

Applied board-wide PWM frequency in hertz.

| Parameter | Type | Bytes | Description |
| --- | --- | ---: | --- |
| `frequency_hz` | `uint32_t` | 4 | Applied board-wide PWM frequency in hertz. |

#### Errors

[`INVALID_LENGTH`](#error-invalid-length) (4) · [`INVALID_ARGUMENT`](#error-invalid-argument) (2) · [`INIT_REQUIRED`](#error-init-required) (15) · [`CLOCK_NOT_READY`](#error-clock-not-ready) (7) · [`INTERNAL_ERROR`](#error-internal-error) (6)

[Back to command index](#command-index)

---

### SAVE_CONFIGURATION (0x59)

**Client → controller** · **Payload:** 0 bytes

Save the complete running platform setup and motor PWM frequency to onboard FRAM. Available on v0.4 with protocol 2.14 or later. ACK confirms a durable save. A subsequent boot restores the setup with outputs stopped. Protocol 2.23+ also saves the applied physical IMU mount and accepts an explicitly configured mount before platform commissioning.

#### Parameters

No payload parameters.

#### Response

**ACK** — empty payload; echoes the command and message ID.

#### Errors

[`INVALID_LENGTH`](#error-invalid-length) (4) · [`INIT_REQUIRED`](#error-init-required) (15) · [`CLOCK_NOT_READY`](#error-clock-not-ready) (7) · [`SENSOR_NOT_AVAILABLE`](#error-sensor-not-available) (16) · [`PLATFORM_NOT_INITIALIZED`](#error-platform-not-initialized) (12) · [`INVALID_ARGUMENT`](#error-invalid-argument) (2) · [`INTERNAL_ERROR`](#error-internal-error) (6)

[Back to command index](#command-index)

---

### GET_CONFIGURATION_FIELD (0x5A)

**Client → controller** · **Payload:** 2 bytes

Read a running configuration field. Fields 0 through 6 require v0.4. Field 7 is available on all boards with protocol 2.25 and a configured mecanum platform. Use GET_PWM_FREQUENCY for the board-wide switching frequency.

#### Parameters

| Parameter | Type | Bytes | Description |
| --- | --- | ---: | --- |
| `field` | `uint8_t` | 1 | 0 platform type (0 unconfigured, 1 differential, 2 omni, 3 mecanum), 1 wheel diameter, 2 track or length, 3 width or radius, 4 encoder resolution, 5 encoder reversed, 6 motor reversed, 7 mecanum roller orientation (0 STANDARD, 1 MIRRORED). |
| `index` | `uint8_t` | 1 | Zero for platform fields. For fields 4 through 6, use the zero-based channel index. |

#### Response

**Controller → client** · `double` · **Payload:** 8 bytes

Current field value in the units of the corresponding initialization command. Boolean flags are returned as zero or one.

| Parameter | Type | Bytes | Description |
| --- | --- | ---: | --- |
| `value` | `double` | 8 | Current field value in the units of the corresponding initialization command. Boolean flags are returned as zero or one. |

#### Errors

[`INVALID_LENGTH`](#error-invalid-length) (4) · [`INIT_REQUIRED`](#error-init-required) (15) · [`CLOCK_NOT_READY`](#error-clock-not-ready) (7) · [`SENSOR_NOT_AVAILABLE`](#error-sensor-not-available) (16) · [`INVALID_ARGUMENT`](#error-invalid-argument) (2) · [`PLATFORM_NOT_INITIALIZED`](#error-platform-not-initialized) (12)

[Back to command index](#command-index)

---

### SET_DRIVER_SLEEP (0x5B)

**Client → controller** · **Payload:** 1 bytes

Set the board-wide motor inhibit. Entry stops all controllers and blocks new motion. On v0.3 and Nano (protocol 2.17), entry holds each DRV8870 input pair low. Release keeps the inputs low until a fresh drive or explicit short brake. On v0.4 (protocol 2.15+), wake verifies retained settings for initialized drivers and leaves outputs stopped. Failed transitions retain the inhibit for explicit retry. Requires READY and a profile supporting driver sleep.

#### Parameters

| Parameter | Type | Bytes | Range | Description |
| --- | --- | ---: | --- | --- |
| `sleeping` | `uint8_t` | 1 | 0 to 1 | 1 to request driver sleep or 0 to wake. |

#### Response

**ACK** — empty payload; echoes the command and message ID.

#### Errors

[`DRIVERS_ASLEEP`](#error-drivers-asleep) (19) · [`BOARD_IDLE`](#error-board-idle) (20) · [`INVALID_LENGTH`](#error-invalid-length) (4) · [`INVALID_ARGUMENT`](#error-invalid-argument) (2) · [`INIT_REQUIRED`](#error-init-required) (15) · [`CLOCK_NOT_READY`](#error-clock-not-ready) (7) · [`SENSOR_NOT_AVAILABLE`](#error-sensor-not-available) (16) · [`INTERNAL_ERROR`](#error-internal-error) (6)

[Back to command index](#command-index)

---

### GET_DRIVER_SLEEP (0x5C)

**Client → controller** · **Payload:** 0 bytes

Read the software motor inhibit across connections. It clears at boot. A zero permits commands and does not establish that each chip is awake. Requires READY and driver sleep support (v0.4 since protocol 2.15, original v0.3 and Nano since 2.17).

#### Parameters

No payload parameters.

#### Response

**Controller → client** · `uint8_t` · **Payload:** 1 bytes

1 when driver sleep is requested or 0 when motor commands are permitted.

| Parameter | Type | Bytes | Description |
| --- | --- | ---: | --- |
| `sleeping` | `uint8_t` | 1 | 1 when driver sleep is requested or 0 when motor commands are permitted. |

#### Errors

[`INVALID_LENGTH`](#error-invalid-length) (4) · [`INIT_REQUIRED`](#error-init-required) (15) · [`CLOCK_NOT_READY`](#error-clock-not-ready) (7) · [`SENSOR_NOT_AVAILABLE`](#error-sensor-not-available) (16)

[Back to command index](#command-index)

---

### SET_BOARD_IDLE (0x5D)

**Client → controller** · **Payload:** 1 bytes

Set communication-preserving board idle. Entry (1) stops control, inhibits the drivers and permits shallow CPU sleep between tasks. USB traffic, time sync, queries and heartbeat keep using the existing session. Only explicit exit (0) clears idle. Exit preserves a prior independent driver inhibit and leaves outputs stopped. On original v0.3 and Nano both DRV8870 inputs stay low until fresh drive or brake. On v0.4, previously active drivers are verified before release. A failed transition retains idle and inhibit for explicit retry. Requires READY, an idle-capable profile and protocol 2.18 on v0.3 or Nano, or 2.16 on v0.4.

#### Parameters

| Parameter | Type | Bytes | Range | Description |
| --- | --- | ---: | --- | --- |
| `idle` | `uint8_t` | 1 | 0 to 1 | 1 to enter board idle or 0 to wake the board. |

#### Response

**ACK** — empty payload; echoes the command and message ID.

#### Errors

[`INVALID_LENGTH`](#error-invalid-length) (4) · [`INVALID_ARGUMENT`](#error-invalid-argument) (2) · [`INIT_REQUIRED`](#error-init-required) (15) · [`CLOCK_NOT_READY`](#error-clock-not-ready) (7) · [`SENSOR_NOT_AVAILABLE`](#error-sensor-not-available) (16) · [`INTERNAL_ERROR`](#error-internal-error) (6) · [`BOARD_IDLE`](#error-board-idle) (20)

[Back to command index](#command-index)

---

### GET_BOARD_IDLE (0x5E)

**Client → controller** · **Payload:** 0 bytes

Read the shared logical idle request without clearing it. Interrupts can resume the CPU while this request remains set. GET_DRIVER_SLEEP reports the effective driver inhibit. Idle survives reconnect and clears at reset. Requires READY and board idle support (original v0.3 and Nano since protocol 2.18, v0.4 since 2.16).

#### Parameters

No payload parameters.

#### Response

**Controller → client** · `uint8_t` · **Payload:** 1 bytes

1 while the board idle request is active, otherwise 0.

| Parameter | Type | Bytes | Description |
| --- | --- | ---: | --- |
| `idle` | `uint8_t` | 1 | 1 while the board idle request is active, otherwise 0. |

#### Errors

[`INVALID_LENGTH`](#error-invalid-length) (4) · [`INIT_REQUIRED`](#error-init-required) (15) · [`CLOCK_NOT_READY`](#error-clock-not-ready) (7) · [`SENSOR_NOT_AVAILABLE`](#error-sensor-not-available) (16)

[Back to command index](#command-index)

---

### SET_FUSION_EXPERIMENT (0x5f)

**Client → controller** · **Payload:** 9 bytes

Configure the volatile v0.4 experiment only while motion is stopped and all commanded targets and motor outputs are zero; initialized controllers with idle zero targets are allowed. Mode 0 retains the legacy encoder path (boot default), 1 uses encoder feedback with the common chassis correction, and 2 uses fused feedback with that same correction. No IMU-only mode. Without a configured physical mount, modes 1/2 require a rigid, level board with component side up, IMU at the chassis reference point, and mounting_yaw_rad as the angle of board +X expressed in chassis XY, counterclockwise about chassis +Z. Sensor-to-board axes are (-x,-y,+z); chassis axes are forward/left/up. Reject nonfinite or out-of-range yaw, any observed configured encoder movement in the preceding 100 ms, or a counter change since a fresh sample. Stopped cold setup may ACK in WAITING; ACK does not establish readiness. Both experimental modes require 100 ms of observed unchanged configured counters for initial rest arming before accepting a motion target; once armed, ordinary target updates remain accepted while moving; fused bias calibration may run within that window. Reconfiguration cancels old targets. Sources may requalify automatically after an ordinary sensor fault, but healthy inputs and a fresh target are required before control resumes. Does not save to FRAM or start motion. Requires READY, exact v0.4.0 fusion capability and protocol 2.19. On protocol 2.23+, a configured physical mount takes precedence over legacy yaw. Use GET_BOARD_MOUNT on 2.24+ for PCB underside-center geometry. On 2.23 use GET_IMU_MOUNT for chip-reference geometry.

#### Parameters

| Parameter | Type | Bytes | Range | Description |
| --- | --- | ---: | --- | --- |
| `mode` | `uint8_t` | 1 | 0 to 2 | FusionMode value. |
| `mounting_yaw_rad` | `double` | 8 | -3.141592653589793 to 3.141592653589793 | Legacy board yaw in radians for a centered, level installation. A configured physical mount takes precedence. |

#### Response

**ACK** — empty payload; echoes the command and message ID.

#### Errors

[`INVALID_LENGTH`](#error-invalid-length) (4) · [`INVALID_ARGUMENT`](#error-invalid-argument) (2) · [`INIT_REQUIRED`](#error-init-required) (15) · [`CLOCK_NOT_READY`](#error-clock-not-ready) (7) · [`SENSOR_NOT_AVAILABLE`](#error-sensor-not-available) (16) · [`INTERNAL_ERROR`](#error-internal-error) (6) · [`BOARD_IDLE`](#error-board-idle) (20) · [`DRIVERS_ASLEEP`](#error-drivers-asleep) (19) · [`PLATFORM_NOT_INITIALIZED`](#error-platform-not-initialized) (12)

[Back to command index](#command-index)

---

### GET_FUSION_STATUS (0x60)

**Client → controller** · **Payload:** 0 bytes

Read experiment mode, health, fault, flags and timestamped sample ages without setting targets or changing configuration. A read may enforce an expired feedback deadline, clear the live target and coast. Health/fault/flag values use the FusionHealth/FusionFault/FusionFlag catalogs. Timestamp uses the monotonic board clock; unavailable ages are UINT32_MAX. The physical IMU producer waits 2 ms between passes, without guaranteeing a 2 ms acquisition cadence, and pairs only acquisition windows entirely preceding the encoder timestamp: each window is at most 5 ms wide, worst-case accel/gyro skew at most 15 ms, and age at most 35 ms. These are software acceptance limits, not a physical timing qualification. ESTIMATE_VALID describes the selected experiment estimate, not physical accuracy. Requires READY, exact v0.4.0 fusion capability and protocol 2.19.

#### Parameters

No payload parameters.

#### Response

**Controller → client** · `fusion_status` · **Payload:** 28 bytes

Packed little-endian diagnostic snapshot.

| Parameter | Type | Bytes | Description |
| --- | --- | ---: | --- |
| `mode` | `uint8_t` | 1 | Selected fusion_modes value. |
| `health` | `uint8_t` | 1 | Current fusion_health value. |
| `fault` | `uint8_t` | 1 | Current fusion_faults reason, or NONE. |
| `flags` | `uint8_t` | 1 | Bitmask of fusion_flags; ESTIMATE_VALID is required before using estimates. |
| `sequence` | `uint32_t` | 4 | Snapshot sequence; compare with each vector before associating this status. |
| `timestamp_us` | `uint64_t` | 8 | Monotonic board microseconds of this estimate snapshot; not wall time. |
| `accel_age_us` | `uint32_t` | 4 | UINT32_MAX means unavailable. |
| `gyro_age_us` | `uint32_t` | 4 | UINT32_MAX means unavailable. |
| `encoder_age_us` | `uint32_t` | 4 | UINT32_MAX means unavailable. |

#### Errors

[`INVALID_LENGTH`](#error-invalid-length) (4) · [`INVALID_ARGUMENT`](#error-invalid-argument) (2) · [`INIT_REQUIRED`](#error-init-required) (15) · [`CLOCK_NOT_READY`](#error-clock-not-ready) (7) · [`SENSOR_NOT_AVAILABLE`](#error-sensor-not-available) (16) · [`INTERNAL_ERROR`](#error-internal-error) (6)

[Back to command index](#command-index)

---

### GET_FUSION_VECTOR (0x61)

**Client → controller** · **Payload:** 1 bytes

Read one compact experiment vector: 0 encoder pose (relative x,y,yaw), 1 encoder body twist (vx,vy,yaw rate), 2 fused pose, 3 fused body twist, 4 estimated bias (ax,ay,gyro z). Units are metres, seconds and radians. On outage recovery, the last fused-from-encoder world transform is reapplied to the current encoder pose. Returns SAMPLE_NOT_AVAILABLE when the requested estimate is invalid. Match sequence with GET_FUSION_STATUS to associate timestamp/health; different sequences are different snapshots. At most 28 payload bytes so the framed reply fits Arduino Wire32. Does not reset state or start motion. Requires READY, exact v0.4.0 fusion capability and protocol 2.19.

#### Parameters

| Parameter | Type | Bytes | Range | Description |
| --- | --- | ---: | --- | --- |
| `kind` | `uint8_t` | 1 | 0 to 4 | FusionVectorKind value. |

#### Response

**Controller → client** · `fusion_vector` · **Payload:** 28 bytes

Packed little-endian diagnostic snapshot.

| Parameter | Type | Bytes | Description |
| --- | --- | ---: | --- |
| `sequence` | `uint32_t` | 4 | Match GET_FUSION_STATUS sequence before associating its timestamp or health. |
| `x` | `double` | 8 | Pose X in metres, twist X in metres per second, or X accelerometer bias in metres per second squared, selected by kind. |
| `y` | `double` | 8 | Pose Y in metres, twist Y in metres per second, or Y accelerometer bias in metres per second squared, selected by kind. |
| `z` | `double` | 8 | Pose yaw in radians, twist yaw rate in radians per second, or Z gyro bias in radians per second, selected by kind. |

#### Errors

[`INVALID_LENGTH`](#error-invalid-length) (4) · [`INVALID_ARGUMENT`](#error-invalid-argument) (2) · [`INIT_REQUIRED`](#error-init-required) (15) · [`CLOCK_NOT_READY`](#error-clock-not-ready) (7) · [`SENSOR_NOT_AVAILABLE`](#error-sensor-not-available) (16) · [`INTERNAL_ERROR`](#error-internal-error) (6) · [`SAMPLE_NOT_AVAILABLE`](#error-sample-not-available) (10)

[Back to command index](#command-index)

---

### GET_FUSION_CONFIGURATION (0x62)

**Client → controller** · **Payload:** 0 bytes

Read the volatile experiment mode and declared mounting yaw without changing state. Reset selects legacy encoder mode; SAVE_CONFIGURATION does not persist this experiment. Requires READY, exact v0.4.0 fusion capability and protocol 2.19. On protocol 2.23+, a configured physical mount takes precedence over legacy yaw. Use GET_BOARD_MOUNT on 2.24+ for PCB underside-center geometry. On 2.23 use GET_IMU_MOUNT for chip-reference geometry.

#### Parameters

No payload parameters.

#### Response

**Controller → client** · `fusion_configuration` · **Payload:** 9 bytes

Packed little-endian diagnostic snapshot.

| Parameter | Type | Bytes | Description |
| --- | --- | ---: | --- |
| `mode` | `uint8_t` | 1 | Selected fusion_modes value; boot default is LEGACY_ENCODER. |
| `mounting_yaw_rad` | `double` | 8 | Angle of board +X in the chassis XY plane, counterclockwise about chassis +Z; finite radians in [-pi, pi]. |

#### Errors

[`INVALID_LENGTH`](#error-invalid-length) (4) · [`INVALID_ARGUMENT`](#error-invalid-argument) (2) · [`INIT_REQUIRED`](#error-init-required) (15) · [`CLOCK_NOT_READY`](#error-clock-not-ready) (7) · [`SENSOR_NOT_AVAILABLE`](#error-sensor-not-available) (16) · [`INTERNAL_ERROR`](#error-internal-error) (6)

[Back to command index](#command-index)

---

### SET_IMU_TRACKING (0x63)

**Client → controller** · **Payload:** 10 bytes

Control volatile IMU-only relative tracking. START=0, RESET=2 and RECALIBRATE=3 are accepted from any state with stationary_mount_confirmed=1 and finite yaw in [-pi,pi]; all discard prior bias and pose and begin full cold stationary calibration at a new relative origin. The caller confirms true zero velocity and the actual rigid installation. Without a configured physical mount, use a level sensor with +Z up at the tracking origin. Legacy yaw is the angle of sensor +X in chassis XY, counterclockwise about chassis +Z. The legacy sensor-to-chassis transform is Rz(yaw)*(x,y,z), axes forward/left/up. IMU stationarity checks cannot establish zero constant velocity. STOP=1 requires confirmation=0 and yaw=0 and stops integration. ACK confirms the action, not a valid estimate. Fault, expiry or interruption requires deliberate restart; no automatic recovery. Not saved to FRAM. Requires READY, exact model 1 hardware 0.4.0 with imu_tracking capability and protocol 2.20+. Independent of platform, encoders and fusion; never commands motion or changes control feedback. On protocol 2.23+, a configured physical mount takes precedence over legacy yaw. Use GET_BOARD_MOUNT on 2.24+ for PCB underside-center geometry. On 2.23 use GET_IMU_MOUNT for chip-reference geometry.

#### Parameters

| Parameter | Type | Bytes | Range | Description |
| --- | --- | ---: | --- | --- |
| `action` | `uint8_t` | 1 | 0 to 3 | ImuTrackingAction value. |
| `stationary_mount_confirmed` | `uint8_t` | 1 | 0 to 1 | Explicit caller confirmation of zero velocity and the declared rigid installation, 0 or 1. |
| `mounting_yaw_rad` | `double` | 8 | -3.141592653589793 to 3.141592653589793 | Legacy sensor +X yaw about chassis +Z in finite radians for a level sensor. A configured physical mount takes precedence. |

#### Response

**ACK** — empty payload; echoes the command and message ID.

#### Errors

[`INVALID_LENGTH`](#error-invalid-length) (4) · [`INVALID_ARGUMENT`](#error-invalid-argument) (2) · [`INIT_REQUIRED`](#error-init-required) (15) · [`CLOCK_NOT_READY`](#error-clock-not-ready) (7) · [`SENSOR_NOT_AVAILABLE`](#error-sensor-not-available) (16) · [`INTERNAL_ERROR`](#error-internal-error) (6) · [`BOARD_IDLE`](#error-board-idle) (20) · [`DRIVERS_ASLEEP`](#error-drivers-asleep) (19)

[Back to command index](#command-index)

---

### GET_IMU_TRACKING_STATUS (0x64)

**Client → controller** · **Payload:** 0 bytes

Read IMU-only tracking state, fault, flags, sequence, last eligible joint sample-watermark timestamp, wall elapsed time since the source origin capped at window_us and separate accel/gyro ages in microseconds. UINT32_MAX age means unavailable. ESTIMATE_VALID is required for usable current estimates; timestamp and health are not physical accuracy guarantees. A delayed in-window source sample may remain readable briefly after the wall clock reaches the source end. Healthy tracking expires upon the first consumed source watermark at or beyond the end, or by the first serviced status/vector/configuration read at or after end + 50000 us. Missing, stale or invalid input may fault earlier. This is a serviced-read guarantee, not a hard real-time scheduling guarantee. Reads never start tracking. Status and vector sequences must match to combine separate reads. Requires READY, exact model 1 hardware 0.4.0 with imu_tracking capability and protocol 2.20+. Independent of platform, encoders and fusion; never commands motion or changes control feedback.

#### Parameters

No payload parameters.

#### Response

**Controller → client** · `imu_tracking_status` · **Payload:** 28 bytes

Packed little-endian IMU-only tracking snapshot.

| Parameter | Type | Bytes | Description |
| --- | --- | ---: | --- |
| `state` | `uint8_t` | 1 | ImuTrackingState value. |
| `fault` | `uint8_t` | 1 | ImuTrackingFault value. |
| `flags` | `uint8_t` | 1 | ImuTrackingFlag bitmask. |
| `reserved` | `uint8_t` | 1 | Reserved; must be zero. |
| `sequence` | `uint32_t` | 4 | Snapshot sequence; match vector reads. |
| `timestamp_us` | `uint64_t` | 8 | Last eligible monotonic joint accel/gyro source watermark; no published watermark reaches origin + window_us. Zero before the first estimate. |
| `elapsed_us` | `uint32_t` | 4 | Wall elapsed microseconds since the source origin, capped at window_us; distinct from the retained source watermark. Frozen on fault or expiry; STOP clears it. |
| `accel_age_us` | `uint32_t` | 4 | Accel age in microseconds; UINT32_MAX if unavailable. |
| `gyro_age_us` | `uint32_t` | 4 | Gyro age in microseconds; UINT32_MAX if unavailable. |

#### Errors

[`INVALID_LENGTH`](#error-invalid-length) (4) · [`INVALID_ARGUMENT`](#error-invalid-argument) (2) · [`INIT_REQUIRED`](#error-init-required) (15) · [`CLOCK_NOT_READY`](#error-clock-not-ready) (7) · [`SENSOR_NOT_AVAILABLE`](#error-sensor-not-available) (16) · [`INTERNAL_ERROR`](#error-internal-error) (6)

[Back to command index](#command-index)

---

### GET_IMU_TRACKING_VECTOR (0x65)

**Client → controller** · **Payload:** 1 bytes

Read a compact IMU-only vector selected by kind: POSE=0 (relative metres,metres,radians), TWIST=1 (body metres/second,metres/second,radians/second), BIAS=2 (accel-X,accel-Y metres/second squared and gyro-Z radians/second). SAMPLE_NOT_AVAILABLE means the requested estimate is invalid. Match sequence with GET_IMU_TRACKING_STATUS; never combine mismatched snapshots. Relative integration drifts and has no absolute position or heading reference. Requires READY, exact model 1 hardware 0.4.0 with imu_tracking capability and protocol 2.20+. Independent of platform, encoders and fusion; never commands motion or changes control feedback.

#### Parameters

| Parameter | Type | Bytes | Range | Description |
| --- | --- | ---: | --- | --- |
| `kind` | `uint8_t` | 1 | 0 to 2 | ImuTrackingVectorKind value. |

#### Response

**Controller → client** · `imu_tracking_vector` · **Payload:** 28 bytes

Packed little-endian IMU-only tracking snapshot.

| Parameter | Type | Bytes | Description |
| --- | --- | ---: | --- |
| `sequence` | `uint32_t` | 4 | Match status sequence before associating health and time. |
| `x` | `double` | 8 | Pose X, body velocity X, or acceleration bias X, selected by kind. |
| `y` | `double` | 8 | Pose Y, body velocity Y, or acceleration bias Y, selected by kind. |
| `z` | `double` | 8 | Pose yaw, body yaw rate, or gyro Z bias, selected by kind. |

#### Errors

[`INVALID_LENGTH`](#error-invalid-length) (4) · [`INVALID_ARGUMENT`](#error-invalid-argument) (2) · [`INIT_REQUIRED`](#error-init-required) (15) · [`CLOCK_NOT_READY`](#error-clock-not-ready) (7) · [`SENSOR_NOT_AVAILABLE`](#error-sensor-not-available) (16) · [`INTERNAL_ERROR`](#error-internal-error) (6) · [`SAMPLE_NOT_AVAILABLE`](#error-sample-not-available) (10)

[Back to command index](#command-index)

---

### GET_IMU_TRACKING_CONFIGURATION (0x66)

**Client → controller** · **Payload:** 0 bytes

Read mounting yaw and the actual bounded tracking horizon in microseconds. This v0.4 release reports 1000000 us for a one-second motion-onset diagnostic. Always use readback for the connected board horizon. The eligible source interval is [origin, origin + window_us); at 100 Hz it contains 100 source stamps including origin and excluding the end. No estimate at or beyond the source end is published. Reset clears volatile tracking state; reconnect does not start or re-anchor tracking. Requires READY, exact model 1 hardware 0.4.0 with imu_tracking capability and protocol 2.20+. Independent of platform, encoders and fusion; never commands motion or changes control feedback. On protocol 2.23+, a configured physical mount takes precedence over legacy yaw. Use GET_BOARD_MOUNT on 2.24+ for PCB underside-center geometry. On 2.23 use GET_IMU_MOUNT for chip-reference geometry.

#### Parameters

No payload parameters.

#### Response

**Controller → client** · `imu_tracking_configuration` · **Payload:** 12 bytes

Packed little-endian IMU-only tracking snapshot.

| Parameter | Type | Bytes | Description |
| --- | --- | ---: | --- |
| `mounting_yaw_rad` | `double` | 8 | Declared supported mounting yaw in radians. |
| `window_us` | `uint32_t` | 4 | Actual source tracking horizon in microseconds; 1000000 for this v0.4 release. Eligible source times are [origin, origin + window_us). |

#### Errors

[`INVALID_LENGTH`](#error-invalid-length) (4) · [`INVALID_ARGUMENT`](#error-invalid-argument) (2) · [`INIT_REQUIRED`](#error-init-required) (15) · [`CLOCK_NOT_READY`](#error-clock-not-ready) (7) · [`SENSOR_NOT_AVAILABLE`](#error-sensor-not-available) (16) · [`INTERNAL_ERROR`](#error-internal-error) (6)

[Back to command index](#command-index)

---

### SET_FEEDBACK_SOURCE (0x67)

**Client → controller** · **Payload:** 1 bytes

Choose the board-wide telemetry preference. A change requires stopped outputs, disarmed controllers and stationary observed encoders. An unchanged preference is idempotent. The selector never starts motion, wheel odometry, standalone IMU calibration or a controller. FUSED uses an independent passive instance of the release estimator when its explicit mount and encoder odometry are available. It anchors a new relative frame at the first available source sample serviced after a switch or producer restart. The existing SAVE_CONFIGURATION explicitly persists the preference alongside platform setup; cold restore never restores an origin, bias, stationary declaration or motion target. Requires READY and exact model 1 hardware 0.4.0 with feedback_source_selector and protocol 2.21 or newer. Selection only routes tracking telemetry.

#### Parameters

| Parameter | Type | Bytes | Range | Description |
| --- | --- | ---: | --- | --- |
| `source` | `uint8_t` | 1 | 0 to 2 | FeedbackSource value (NONE is read-only). |

#### Response

**ACK** — empty payload; echoes the command and message ID.

#### Errors

[`INVALID_LENGTH`](#error-invalid-length) (4) · [`INVALID_ARGUMENT`](#error-invalid-argument) (2) · [`INIT_REQUIRED`](#error-init-required) (15) · [`CLOCK_NOT_READY`](#error-clock-not-ready) (7) · [`SENSOR_NOT_AVAILABLE`](#error-sensor-not-available) (16) · [`INTERNAL_ERROR`](#error-internal-error) (6) · [`SOURCE_SWITCH_UNSAFE`](#error-source-switch-unsafe) (21)

[Back to command index](#command-index)

---

### GET_FEEDBACK_SOURCE_STATUS (0x68)

**Client → controller** · **Payload:** 0 bytes

Get requested and active source, availability reason and producer detail. Invalid status reports active NONE; no fallback. While valid the sample sequence and monotonic timestamp identify the underlying producer publication. Inspect requested source diagnostics for acquisition ages and fault detail. The standalone IMU is a one-second relative motion-onset diagnostic; fused telemetry uses the existing experimental estimator. Reconnect reads live board state. Requires READY and exact model 1 hardware 0.4.0 with feedback_source_selector and protocol 2.21 or newer. Selection only routes tracking telemetry.

#### Parameters

No payload parameters.

#### Response

**Controller → client** · `feedback_source_status` · **Payload:** 27 bytes

Packed little-endian selector readback.

| Parameter | Type | Bytes | Description |
| --- | --- | ---: | --- |
| `requested` | `uint8_t` | 1 | Requested FeedbackSource. |
| `active` | `uint8_t` | 1 | Active FeedbackSource, NONE while invalid. |
| `valid` | `uint8_t` | 1 | One when a current source snapshot is available, otherwise zero. |
| `reason` | `uint8_t` | 1 | FeedbackSourceReason. |
| `sequence` | `uint32_t` | 4 | Underlying sample sequence when valid; zero when invalid. |
| `timestamp_us` | `uint64_t` | 8 | Underlying monotonic board sample time; zero when invalid. |
| `origin_id` | `uint32_t` | 4 | Board-local frame generation. Discard prior coordinates on change or board reboot. |
| `source_detail` | `uint8_t` | 1 | Native IMU tracking or fusion fault code for the requested source; zero otherwise. |
| `age_us` | `uint32_t` | 4 | Board-clock sample age in microseconds; UINT32_MAX when unavailable. |
| `max_age_ms` | `uint16_t` | 2 | Maximum eligible captured sample age in milliseconds for this source, including the configured encoder period. Compare updated original sample age with the smaller of its captured and current limits. |

#### Errors

[`INVALID_LENGTH`](#error-invalid-length) (4) · [`INVALID_ARGUMENT`](#error-invalid-argument) (2) · [`INIT_REQUIRED`](#error-init-required) (15) · [`CLOCK_NOT_READY`](#error-clock-not-ready) (7) · [`SENSOR_NOT_AVAILABLE`](#error-sensor-not-available) (16) · [`INTERNAL_ERROR`](#error-internal-error) (6)

[Back to command index](#command-index)

---

### GET_FEEDBACK_SOURCE_VECTOR (0x69)

**Client → controller** · **Payload:** 9 bytes

Get selector-relative pose (metres, metres, unwrapped radians) or body twist (metres/second, metres/second, radians/second), +X forward, +Y left, yaw CCW. Relative XY is rotated into the heading at the new anchor. First capture a valid status, then request both vectors with its exact origin_id and sequence. Eight distinct recent captures are retained for at most 50000 us after capture. Both keys and the current healthy source/frame must match. Selected historical sample age cannot exceed 50000 us for IMU-only, 60000 us for fused, or three configured encoder periods. Expiry or invalidation immediately rejects cached data. A final live status may naturally advance sequence and time; require the same valid requested/active source and origin, match both vector sequences to the original capture and retry on SAMPLE_NOT_AVAILABLE. Compare origins only within the same boot. Requires READY and exact model 1 hardware 0.4.0 with feedback_source_selector and protocol 2.21 or newer. Selection only routes tracking telemetry.

#### Parameters

| Parameter | Type | Bytes | Range | Description |
| --- | --- | ---: | --- | --- |
| `kind` | `uint8_t` | 1 | 0 to 1 | FeedbackSourceVectorKind value. |
| `origin_id` | `uint32_t` | 4 | — | Frame generation captured by GET_FEEDBACK_SOURCE_STATUS. |
| `sequence` | `uint32_t` | 4 | — | Exact source sample sequence captured by GET_FEEDBACK_SOURCE_STATUS. |

#### Response

**Controller → client** · `feedback_source_vector` · **Payload:** 28 bytes

Packed little-endian selector readback.

| Parameter | Type | Bytes | Description |
| --- | --- | ---: | --- |
| `sequence` | `uint32_t` | 4 | Exact producer sequence captured by status and requested by the vector command. |
| `x` | `double` | 8 | X coordinate or body X velocity. |
| `y` | `double` | 8 | Y coordinate or body Y velocity. |
| `z` | `double` | 8 | Continuous relative yaw or body yaw rate. |

#### Errors

[`INVALID_LENGTH`](#error-invalid-length) (4) · [`INVALID_ARGUMENT`](#error-invalid-argument) (2) · [`INIT_REQUIRED`](#error-init-required) (15) · [`CLOCK_NOT_READY`](#error-clock-not-ready) (7) · [`SENSOR_NOT_AVAILABLE`](#error-sensor-not-available) (16) · [`INTERNAL_ERROR`](#error-internal-error) (6) · [`SAMPLE_NOT_AVAILABLE`](#error-sample-not-available) (10)

[Back to command index](#command-index)

---

### SET_FEEDBACK_FUSION_MOUNT (0x6a)

**Client → controller** · **Payload:** 8 bytes

Declare the legacy centered, level board orientation for passive fused telemetry, with finite board yaw in [-pi,pi]. Requires stopped outputs and disarmed controllers. Each successful command discards prior passive calibration and pose; the observer requires fresh stationary observations. This volatile setting applies only to passive telemetry. Requires READY, exact v0.4 and protocol 2.21+. On protocol 2.23+, a configured physical mount takes precedence over legacy yaw. Use GET_BOARD_MOUNT on 2.24+ for PCB underside-center geometry. On 2.23 use GET_IMU_MOUNT for chip-reference geometry.

#### Parameters

| Parameter | Type | Bytes | Range | Description |
| --- | --- | ---: | --- | --- |
| `mounting_yaw_rad` | `double` | 8 | -3.141592653589793 to 3.141592653589793 | Legacy yaw of board +X in chassis XY, counterclockwise about +Z in radians. Centered and level applies only to a yaw-only installation. |

#### Response

**ACK** — empty payload; echoes the command and message ID.

#### Errors

[`INVALID_LENGTH`](#error-invalid-length) (4) · [`INVALID_ARGUMENT`](#error-invalid-argument) (2) · [`INIT_REQUIRED`](#error-init-required) (15) · [`CLOCK_NOT_READY`](#error-clock-not-ready) (7) · [`SENSOR_NOT_AVAILABLE`](#error-sensor-not-available) (16) · [`INTERNAL_ERROR`](#error-internal-error) (6) · [`SOURCE_SWITCH_UNSAFE`](#error-source-switch-unsafe) (21)

[Back to command index](#command-index)

---

### GET_FEEDBACK_FUSION_MOUNT (0x6b)

**Client → controller** · **Payload:** 0 bytes

Read the explicit volatile passive fusion mounting confirmation and board yaw. Reset clears confirmation. Requires READY, exact v0.4 and protocol 2.21+. On protocol 2.23+, a configured physical mount takes precedence over legacy yaw. Use GET_BOARD_MOUNT on 2.24+ for PCB underside-center geometry. On 2.23 use GET_IMU_MOUNT for chip-reference geometry.

#### Parameters

No payload parameters.

#### Response

**Controller → client** · `feedback_fusion_mount` · **Payload:** 9 bytes

Packed little-endian selector readback.

| Parameter | Type | Bytes | Description |
| --- | --- | ---: | --- |
| `confirmed` | `uint8_t` | 1 | One after an explicit mount declaration this boot. |
| `mounting_yaw_rad` | `double` | 8 | Angle of board +X in chassis XY, counterclockwise about +Z in radians. A centered, level board with +Z up is required only for a yaw-only installation. |

#### Errors

[`INVALID_LENGTH`](#error-invalid-length) (4) · [`INVALID_ARGUMENT`](#error-invalid-argument) (2) · [`INIT_REQUIRED`](#error-init-required) (15) · [`CLOCK_NOT_READY`](#error-clock-not-ready) (7) · [`SENSOR_NOT_AVAILABLE`](#error-sensor-not-available) (16) · [`INTERNAL_ERROR`](#error-internal-error) (6)

[Back to command index](#command-index)

---

### GET_FEEDBACK_SOURCE_HEALTH (0x6c)

**Client → controller** · **Payload:** 1 bytes

Read producer health independently of selector and control. ENCODER is the running platform encoder batch, IMU_ONLY is physical BMI088 pair availability independent of the one-second estimate, FUSED is the passive fused estimate. Disagreement indicates conflicting observations without assigning sensor blame. HEALTHY means directly usable data. Comparability is separate. Historical provenance remains readable during failure. Reconnect reads board state. Requires READY, exact v0.4 and protocol 2.22+ with feedback_source_health.

#### Parameters

| Parameter | Type | Bytes | Range | Description |
| --- | --- | ---: | --- | --- |
| `source` | `uint8_t` | 1 | 0 to 2 | FeedbackSource value, excluding NONE. |

#### Response

**Controller → client** · `feedback_source_health` · **Payload:** 28 bytes

Read producer health independently of selector and control. ENCODER is the running platform encoder batch, IMU_ONLY is physical BMI088 pair availability independent of the one-second estimate, FUSED is the passive fused estimate. Disagreement indicates conflicting observations without assigning sensor blame. HEALTHY means directly usable data. Comparability is separate. Historical provenance remains readable during failure. Reconnect reads board state.

| Parameter | Type | Bytes | Description |
| --- | --- | ---: | --- |
| `source` | `uint8_t` | 1 | Echo of FeedbackSource request. |
| `health` | `uint8_t` | 1 | FeedbackSourceHealthState. |
| `reason` | `uint8_t` | 1 | FeedbackSourceReason for current producer availability. |
| `detail` | `uint8_t` | 1 | FusionFault explanation if present. |
| `flags` | `uint32_t` | 4 | FeedbackSourceHealthFlag bit mask. |
| `sequence` | `uint32_t` | 4 | Sequence of the last producer publication or observed distinct raw IMU pair, wrapping modulo 2^32; use PROVENANCE for presence. |
| `timestamp_us` | `uint64_t` | 8 | Last usable source time in board uptime microseconds. For the BMI088 pair this is its older die timestamp. |
| `age_us` | `uint32_t` | 4 | Age of that timestamp in board uptime microseconds, UINT32_MAX if absent, future or too old to represent. |
| `max_age_us` | `uint32_t` | 4 | Source freshness budget in microseconds. |

#### Errors

[`INVALID_LENGTH`](#error-invalid-length) (4) · [`INVALID_ARGUMENT`](#error-invalid-argument) (2) · [`INIT_REQUIRED`](#error-init-required) (15) · [`CLOCK_NOT_READY`](#error-clock-not-ready) (7) · [`SENSOR_NOT_AVAILABLE`](#error-sensor-not-available) (16) · [`INTERNAL_ERROR`](#error-internal-error) (6)

[Back to command index](#command-index)

---

### GET_FEEDBACK_IMU_CHANNELS (0x6d)

**Client → controller** · **Payload:** 0 bytes

Read independent last observed accelerometer and gyro acquisition times in board uptime microseconds. A timestamp of zero and age UINT32_MAX indicate no observed acquisition. These are physical source observations, independent of the finite IMU-only tracking window. A pair must meet the 35000 us age and 15000 us skew limits. Requires READY, exact v0.4 and protocol 2.22+ with feedback_source_health.

#### Parameters

No payload parameters.

#### Response

**Controller → client** · `feedback_imu_channels` · **Payload:** 24 bytes

Read independent last observed accelerometer and gyro acquisition times in board uptime microseconds. A timestamp of zero and age UINT32_MAX indicate no observed acquisition. These are physical source observations, independent of the finite IMU-only tracking window. A pair must meet the 35000 us age and 15000 us skew limits.

| Parameter | Type | Bytes | Description |
| --- | --- | ---: | --- |
| `accel_timestamp_us` | `uint64_t` | 8 | Last observed accelerometer acquisition time in board uptime microseconds. |
| `gyro_timestamp_us` | `uint64_t` | 8 | Last observed gyro acquisition time in board uptime microseconds. |
| `accel_age_us` | `uint32_t` | 4 | Board-derived acquisition age in microseconds, UINT32_MAX if unknown. |
| `gyro_age_us` | `uint32_t` | 4 | Board-derived acquisition age in microseconds, UINT32_MAX if unknown. |

#### Errors

[`INVALID_LENGTH`](#error-invalid-length) (4) · [`INVALID_ARGUMENT`](#error-invalid-argument) (2) · [`INIT_REQUIRED`](#error-init-required) (15) · [`CLOCK_NOT_READY`](#error-clock-not-ready) (7) · [`SENSOR_NOT_AVAILABLE`](#error-sensor-not-available) (16) · [`INTERNAL_ERROR`](#error-internal-error) (6)

[Back to command index](#command-index)

---

### SET_IMU_MOUNT (0x6e)

**Client → controller** · **Payload:** 18 bytes

Legacy chip-reference API, deprecated for new installations. Set the complete rigid mount for IMU-only and fused tracking on a stopped, disarmed v0.4. The board-to-chassis rotation is Rz(yaw) Ry(pitch) Rx(roll). A changed mount requires fresh tracking initialization. SAVE_CONFIGURATION persists this geometry. Requires READY and protocol 2.23+ with imu_mounting. XYZ refers to the BMI088 chip. Use SET_BOARD_MOUNT for the PCB underside center.

#### Parameters

| Parameter | Type | Bytes | Range | Description |
| --- | --- | ---: | --- | --- |
| `roll_cdeg` | `int16_t` | 2 | -18000 to 18000 | Board-to-chassis roll in signed hundredths of a degree. |
| `pitch_cdeg` | `int16_t` | 2 | -9000 to 9000 | Board-to-chassis pitch in signed hundredths of a degree. |
| `yaw_cdeg` | `int16_t` | 2 | -18000 to 18000 | Board-to-chassis yaw in signed hundredths of a degree. |
| `x_um` | `int32_t` | 4 | -10000000 to 10000000 | Signed displacement from the tracking origin to the BMI088 chip along chassis +X (forward), in micrometres. |
| `y_um` | `int32_t` | 4 | -10000000 to 10000000 | Signed displacement from the tracking origin to the BMI088 chip along chassis +Y (left), in micrometres. |
| `z_um` | `int32_t` | 4 | -10000000 to 10000000 | Signed displacement from the tracking origin to the BMI088 chip along chassis +Z (up), in micrometres. |

#### Response

**ACK** — empty payload; echoes the command and message ID.

#### Errors

[`INVALID_LENGTH`](#error-invalid-length) (4) · [`INVALID_ARGUMENT`](#error-invalid-argument) (2) · [`INIT_REQUIRED`](#error-init-required) (15) · [`CLOCK_NOT_READY`](#error-clock-not-ready) (7) · [`SENSOR_NOT_AVAILABLE`](#error-sensor-not-available) (16) · [`INTERNAL_ERROR`](#error-internal-error) (6) · [`SOURCE_SWITCH_UNSAFE`](#error-source-switch-unsafe) (21)

[Back to command index](#command-index)

---

### GET_IMU_MOUNT (0x6f)

**Client → controller** · **Payload:** 0 bytes

Legacy chip-reference API, deprecated for new installations. Read the shared physical mount. When configured is false, IMU-only uses its legacy sensor yaw and fusion its legacy board yaw. Requires READY and protocol 2.23+ with imu_mounting. XYZ is the BMI088 chip position, including when derived from a PCB underside-center installation.

#### Parameters

No payload parameters.

#### Response

**Controller → client** · `imu_mount_configuration` · **Payload:** 19 bytes

Applied board-to-chassis geometry and configuration state.

| Parameter | Type | Bytes | Range | Description |
| --- | --- | ---: | --- | --- |
| `roll_cdeg` | `int16_t` | 2 | -18000 to 18000 | Board-to-chassis roll in signed hundredths of a degree. |
| `pitch_cdeg` | `int16_t` | 2 | -9000 to 9000 | Board-to-chassis pitch in signed hundredths of a degree. |
| `yaw_cdeg` | `int16_t` | 2 | -18000 to 18000 | Board-to-chassis yaw in signed hundredths of a degree. |
| `x_um` | `int32_t` | 4 | -10031098 to 10031098 | Signed displacement from the tracking origin to the BMI088 chip along chassis +X (forward), in micrometres. A position derived from the other reference can exceed its setter's 10 m input range. |
| `y_um` | `int32_t` | 4 | -10031098 to 10031098 | Signed displacement from the tracking origin to the BMI088 chip along chassis +Y (left), in micrometres. A position derived from the other reference can exceed its setter's 10 m input range. |
| `z_um` | `int32_t` | 4 | -10031098 to 10031098 | Signed displacement from the tracking origin to the BMI088 chip along chassis +Z (up), in micrometres. A position derived from the other reference can exceed its setter's 10 m input range. |
| `configured` | `bool` | 1 | — | True after explicit mount setup or restore. False selects legacy per-mode yaw. |

#### Errors

[`INVALID_LENGTH`](#error-invalid-length) (4) · [`INVALID_ARGUMENT`](#error-invalid-argument) (2) · [`INIT_REQUIRED`](#error-init-required) (15) · [`CLOCK_NOT_READY`](#error-clock-not-ready) (7) · [`SENSOR_NOT_AVAILABLE`](#error-sensor-not-available) (16) · [`INTERNAL_ERROR`](#error-internal-error) (6)

[Back to command index](#command-index)

---

### SET_BOARD_MOUNT (0x85)

**Client → controller** · **Payload:** 18 bytes

Set the rigid mount on a stopped, disarmed Mini. XYZ is the displacement in chassis axes from the tracking origin to the geometric center of the PCB underside. Board-to-chassis rotation is Rz(yaw) Ry(pitch) Rx(roll). Firmware derives the chip offset. SAVE_CONFIGURATION persists the entered board pose. Requires READY, exact v0.4 imu_mounting and protocol 2.24+.

#### Parameters

| Parameter | Type | Bytes | Range | Description |
| --- | --- | ---: | --- | --- |
| `roll_cdeg` | `int16_t` | 2 | -18000 to 18000 | Board-to-chassis roll in signed hundredths of a degree. |
| `pitch_cdeg` | `int16_t` | 2 | -9000 to 9000 | Board-to-chassis pitch in signed hundredths of a degree. |
| `yaw_cdeg` | `int16_t` | 2 | -18000 to 18000 | Board-to-chassis yaw in signed hundredths of a degree. |
| `x_um` | `int32_t` | 4 | -10000000 to 10000000 | Signed displacement from the tracking origin to the geometric center of the PCB underside along chassis +X (forward), in micrometres. |
| `y_um` | `int32_t` | 4 | -10000000 to 10000000 | Signed displacement from the tracking origin to the geometric center of the PCB underside along chassis +Y (left), in micrometres. |
| `z_um` | `int32_t` | 4 | -10000000 to 10000000 | Signed displacement from the tracking origin to the geometric center of the PCB underside along chassis +Z (up), in micrometres. |

#### Response

**ACK** — empty payload; echoes the command and message ID.

#### Errors

[`INVALID_LENGTH`](#error-invalid-length) (4) · [`INVALID_ARGUMENT`](#error-invalid-argument) (2) · [`INIT_REQUIRED`](#error-init-required) (15) · [`CLOCK_NOT_READY`](#error-clock-not-ready) (7) · [`SENSOR_NOT_AVAILABLE`](#error-sensor-not-available) (16) · [`INTERNAL_ERROR`](#error-internal-error) (6) · [`SOURCE_SWITCH_UNSAFE`](#error-source-switch-unsafe) (21)

[Back to command index](#command-index)

---

### GET_BOARD_MOUNT (0x86)

**Client → controller** · **Payload:** 0 bytes

Read the applied board orientation and geometric PCB underside center in chassis coordinates. Legacy chip mounts are converted. When unconfigured all coordinates and angles are zero. Requires READY, exact v0.4 imu_mounting and protocol 2.24+.

#### Parameters

No payload parameters.

#### Response

**Controller → client** · `board_mount_configuration` · **Payload:** 19 bytes

Applied orientation and chassis-to-PCB underside-center displacement.

| Parameter | Type | Bytes | Range | Description |
| --- | --- | ---: | --- | --- |
| `roll_cdeg` | `int16_t` | 2 | -18000 to 18000 | Board-to-chassis roll in signed hundredths of a degree. |
| `pitch_cdeg` | `int16_t` | 2 | -9000 to 9000 | Board-to-chassis pitch in signed hundredths of a degree. |
| `yaw_cdeg` | `int16_t` | 2 | -18000 to 18000 | Board-to-chassis yaw in signed hundredths of a degree. |
| `x_um` | `int32_t` | 4 | -10031098 to 10031098 | Signed displacement from the tracking origin to the geometric center of the PCB underside along chassis +X (forward), in micrometres. A position derived from the other reference can exceed its setter's 10 m input range. |
| `y_um` | `int32_t` | 4 | -10031098 to 10031098 | Signed displacement from the tracking origin to the geometric center of the PCB underside along chassis +Y (left), in micrometres. A position derived from the other reference can exceed its setter's 10 m input range. |
| `z_um` | `int32_t` | 4 | -10031098 to 10031098 | Signed displacement from the tracking origin to the geometric center of the PCB underside along chassis +Z (up), in micrometres. A position derived from the other reference can exceed its setter's 10 m input range. |
| `configured` | `bool` | 1 | — | True after explicit mount setup or restore. False selects legacy per-mode yaw. |

#### Errors

[`INVALID_LENGTH`](#error-invalid-length) (4) · [`INVALID_ARGUMENT`](#error-invalid-argument) (2) · [`INIT_REQUIRED`](#error-init-required) (15) · [`CLOCK_NOT_READY`](#error-clock-not-ready) (7) · [`SENSOR_NOT_AVAILABLE`](#error-sensor-not-available) (16) · [`INTERNAL_ERROR`](#error-internal-error) (6)

[Back to command index](#command-index)

---

### INITIALIZE_MECANUM_PLATFORM_ORIENTED (0x87)

**Client → controller** · **Payload:** 41 bytes

Initialize four mecanum wheels with an explicit 45-degree roller layout. Requires protocol 2.25 and a READY session. Motor/encoder indices 0, 1, 2, 3 are front right, rear right, rear left, front left. Length and width are positive front-to-rear and side-to-side wheel-center spacings in meters. STANDARD uses yaw lever (width + length)/2. MIRRORED uses (width - length)/2 and requires unequal spacings. Invalid orientation or geometry leaves the running setup unchanged.

#### Parameters

| Parameter | Type | Bytes | Range | Description |
| --- | --- | ---: | --- | --- |
| `is_reversed_0` | `bool` | 1 | — | Determines if motor 0 is reversed. |
| `is_reversed_1` | `bool` | 1 | — | Determines if motor 1 is reversed. |
| `is_reversed_2` | `bool` | 1 | — | Determines if motor 2 is reversed. |
| `is_reversed_3` | `bool` | 1 | — | Determines if motor 3 is reversed. |
| `is_encoder_reversed_0` | `bool` | 1 | — | Reverses encoder 0 counting direction, independently of motor 0's is_reversed_0. Set so the closed-loop feedback is negative (flip if the wheel runs away when given a target speed). |
| `is_encoder_reversed_1` | `bool` | 1 | — | Reverses encoder 1 counting direction, independently of motor 1's is_reversed_1. Set so the closed-loop feedback is negative (flip if the wheel runs away when given a target speed). |
| `is_encoder_reversed_2` | `bool` | 1 | — | Reverses encoder 2 counting direction, independently of motor 2's is_reversed_2. Set so the closed-loop feedback is negative (flip if the wheel runs away when given a target speed). |
| `is_encoder_reversed_3` | `bool` | 1 | — | Reverses encoder 3 counting direction, independently of motor 3's is_reversed_3. Set so the closed-loop feedback is negative (flip if the wheel runs away when given a target speed). |
| `length` | `double` | 8 | — | Length of the platform in meters. |
| `width` | `double` | 8 | — | Width of the platform in meters. |
| `wheels_diameter` | `double` | 8 | — | Diameter of the robot wheels in meters. |
| `encoder_resolution` | `double` | 8 | — | Encoder resolution in ticks per revolution. The value can not be negative. If platform does not have encoders, the value should be set to zero. |
| `roller_orientation` | `uint8_t` | 1 | 0 to 1 | 0 STANDARD (legacy), 1 MIRRORED (all four roller slopes reversed). Independent of each motor and encoder reversal. |

#### Response

**ACK** — empty payload; echoes the command and message ID.

#### Errors

[`DRIVERS_ASLEEP`](#error-drivers-asleep) (19) · [`BOARD_IDLE`](#error-board-idle) (20) · [`INVALID_LENGTH`](#error-invalid-length) (4) · [`INVALID_ARGUMENT`](#error-invalid-argument) (2) · [`INTERNAL_ERROR`](#error-internal-error) (6) · [`INIT_REQUIRED`](#error-init-required) (15) · [`CLOCK_NOT_READY`](#error-clock-not-ready) (7)

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
| 6 | <a id="error-internal-error"></a>`INTERNAL_ERROR` | The controller could not execute or encode the operation. During motor initialization, possible causes include missing motor power, a driver communication failure, or a faulty driver. |
| 7 | <a id="error-clock-not-ready"></a>`CLOCK_NOT_READY` | Initial clock setup is incomplete; wait for READY. Uptime mode also supports READY. |
| 8 | <a id="error-time-sync-failed"></a>`TIME_SYNC_FAILED` | Initial time sync exhausted its attempts without a valid sample. |
| 9 | <a id="error-odometry-not-initialized"></a>`ODOMETRY_NOT_INITIALIZED` | Odometry is not running; start odometry before requesting a measurement. |
| 10 | <a id="error-sample-not-available"></a>`SAMPLE_NOT_AVAILABLE` | The requested measurement is not ready yet, including after initialization or reset; retry after an update. |
| 11 | <a id="error-encoder-not-initialized"></a>`ENCODER_NOT_INITIALIZED` | Initialize the encoder before reading it or starting encoder odometry. |
| 12 | <a id="error-platform-not-initialized"></a>`PLATFORM_NOT_INITIALIZED` | Initialize a platform before performing this operation. |
| 13 | <a id="error-motor-not-initialized"></a>`MOTOR_NOT_INITIALIZED` | Initialize the motor before setting its speed. |
| 14 | <a id="error-controller-not-initialized"></a>`CONTROLLER_NOT_INITIALIZED` | Start or initialize the closed-loop controller before setting its target. |
| 15 | <a id="error-init-required"></a>`INIT_REQUIRED` | Send a valid INIT request before performing this connection-dependent operation. |
| 16 | <a id="error-sensor-not-available"></a>`SENSOR_NOT_AVAILABLE` | The selected sensor is not defined on this board. |
| 17 | <a id="error-subscription-limit"></a>`SUBSCRIPTION_LIMIT` | This connection has no free subscription slots; unsubscribe an existing source first. |
| 18 | <a id="error-sample-out-of-range"></a>`SAMPLE_OUT_OF_RANGE` | The sensor reading is saturated; no valid measurement can be returned. |
| 19 | <a id="error-drivers-asleep"></a>`DRIVERS_ASLEEP` | Driver sleep is active. Explicitly wake the drivers before requesting this operation. |
| 20 | <a id="error-board-idle"></a>`BOARD_IDLE` | Board idle is active. Explicitly wake the board before changing driver sleep or starting a subscription. |
| 21 | <a id="error-source-switch-unsafe"></a>`SOURCE_SWITCH_UNSAFE` | Stop all motor outputs, disarm all controllers and wait for stationary wheel observations before changing the tracking preference. |
