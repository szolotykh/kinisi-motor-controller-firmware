# Kinisi motor controller commands

Version: 2.0.0
---

All replies use `[length][command ID][message ID][payload]`. Failures use ERROR with the failed command ID and error code. Ordinary commands without a data response return an empty acknowledgment. Successful TIME_SYNC_RESPONSE messages receive no ACK; INIT additionally receives READY after clock setup (see [time sync](docs/time-sync.md)). See [shared response format and errors](docs/responses.md). Protocol 1.x clients require updates.

## Error codes

Errors use the shared ERROR message, echoing the request message ID and identifying the failed command. Each command below lists its possible errors. UNKNOWN_COMMAND applies to unrecognized or incorrectly directed messages.

| Code | Name | Meaning |
| --- | --- | --- |
| 1 | `INCOMPATIBLE_PROTOCOL` | The requested protocol version is incompatible with this firmware. |
| 2 | `INVALID_ARGUMENT` | A field is invalid, out of range, or inconsistent with the pending request. |
| 3 | `UNKNOWN_COMMAND` | The received command ID is not accepted by the controller. |
| 4 | `INVALID_LENGTH` | The complete message has an invalid header or payload length. |
| 5 | `MOTOR_OWNED` | The motor belongs to a platform; use the corresponding platform command. |
| 6 | `INTERNAL_ERROR` | The controller could not execute or encode the operation. |
| 7 | `CLOCK_NOT_READY` | Initial clock setup is incomplete; wait for READY. Uptime mode also supports READY. |
| 8 | `TIME_SYNC_FAILED` | Initial time sync exhausted its attempts without a valid sample. |
| 9 | `ODOMETRY_NOT_INITIALIZED` | Odometry is not running; start odometry before requesting a measurement. |
| 10 | `SAMPLE_NOT_AVAILABLE` | Odometry is running but has no measurement yet, including immediately after reset; retry after an update. |
| 11 | `ENCODER_NOT_INITIALIZED` | Initialize the encoder before reading it or starting encoder odometry. |
| 12 | `PLATFORM_NOT_INITIALIZED` | Initialize a platform before performing this operation. |
| 13 | `MOTOR_NOT_INITIALIZED` | Initialize the motor before setting its speed. |
| 14 | `CONTROLLER_NOT_INITIALIZED` | Start or initialize the closed-loop controller before setting its target. |
| 15 | `INIT_REQUIRED` | Send a valid INIT request before performing this connection-dependent operation. |

## Commands
### INITIALIZE_MOTOR (0x01)
Direction: `client_to_controller`\
Description: This command initializes a motor and prepares it for use. Rejected with MOTOR_OWNED if the motor is currently owned by an active platform (one of its wheels), so platform wheels are not reconfigured out from under the platform.\
Properties:
- motor_index (uint8_t): The index of the motor to initialize.
  - Range: 0 to 3
- is_reversed (bool): Whether or not the motor is reversed.

Errors:

- `INVALID_LENGTH` (4): The complete message has an invalid header or payload length.
- `INVALID_ARGUMENT` (2): A field is invalid, out of range, or inconsistent with the pending request.
- `INTERNAL_ERROR` (6): The controller could not execute or encode the operation.
- `MOTOR_OWNED` (5): The motor belongs to a platform; use the corresponding platform command.

### SET_MOTOR_SPEED (0x02)
Direction: `client_to_controller`\
Description: This command sets the speed of the specified motor in PWM. Rejected with MOTOR_OWNED if the motor is currently owned by an active platform (one of its wheels); use the platform velocity commands to drive platform wheels.\
Properties:
- motor_index (uint8_t): The index of the motor to set the speed for.
  - Range: 0 to 3
- pwm (double): The speed of the motor.
  - Range: -100.0 to 100.0

Errors:

- `INVALID_LENGTH` (4): The complete message has an invalid header or payload length.
- `INVALID_ARGUMENT` (2): A field is invalid, out of range, or inconsistent with the pending request.
- `INTERNAL_ERROR` (6): The controller could not execute or encode the operation.
- `MOTOR_OWNED` (5): The motor belongs to a platform; use the corresponding platform command.
- `MOTOR_NOT_INITIALIZED` (13): Initialize the motor before setting its speed.

### STOP_MOTOR (0x03)
Direction: `client_to_controller`\
Description: Coasts the motor to a stop: both H-bridge outputs are driven low, leaving the motor terminals open (high impedance) so it free-wheels and spins down gradually under its own friction. This also stops that motor's closed-loop speed controller if one is running (started via INITIALIZE_MOTOR_CONTROLLER), so the PID loop cannot re-drive the motor; to command the motor by target speed again you must re-initialize its controller. This is a single-motor command and is ignored if the motor is currently owned by an active platform (one of its wheels); to stop a platform, use STOP_PLATFORM_CONTROLLER, COAST_PLATFORM or BRAKE_PLATFORM instead. Use STOP_MOTOR for a soft, low-stress stop; use BRAKE_MOTOR when you need the motor to hold position and stop quickly.\
Properties:
- motor_index (uint8_t): The index of the motor to set the speed for.
  - Range: 0 to 3

Errors:

- `INVALID_LENGTH` (4): The complete message has an invalid header or payload length.
- `INVALID_ARGUMENT` (2): A field is invalid, out of range, or inconsistent with the pending request.
- `INTERNAL_ERROR` (6): The controller could not execute or encode the operation.
- `MOTOR_OWNED` (5): The motor belongs to a platform; use the corresponding platform command.

### BRAKE_MOTOR (0x04)
Direction: `client_to_controller`\
Description: Actively brakes the motor (short brake): both H-bridge outputs are driven high, shorting the motor terminals together so the motor's own back-EMF resists rotation and it stops quickly and holds position. This also stops that motor's closed-loop speed controller if one is running (started via INITIALIZE_MOTOR_CONTROLLER), so the PID loop cannot re-drive the motor; to command the motor by target speed again you must re-initialize its controller. This is a single-motor command and is ignored if the motor is currently owned by an active platform (one of its wheels); to brake a platform, use BRAKE_PLATFORM (or STOP_PLATFORM_CONTROLLER / COAST_PLATFORM) instead. Use BRAKE_MOTOR for a fast, holding stop; use STOP_MOTOR to let the motor coast freely instead.\
Properties:
- motor_index (uint8_t): The index of the motor to set the speed for.
  - Range: 0 to 3

Errors:

- `INVALID_LENGTH` (4): The complete message has an invalid header or payload length.
- `INVALID_ARGUMENT` (2): A field is invalid, out of range, or inconsistent with the pending request.
- `INTERNAL_ERROR` (6): The controller could not execute or encode the operation.
- `MOTOR_OWNED` (5): The motor belongs to a platform; use the corresponding platform command.

### INITIALIZE_MOTOR_CONTROLLER (0x05)
Direction: `client_to_controller`\
Description: This command sets the controller for the specified motor. Rejected with MOTOR_OWNED if the motor is currently owned by an active platform (one of its wheels), so it cannot create a competing controller on a platform wheel.\
Properties:
- motor_index (uint8_t): The index of the motor to set the controller for.
  - Range: 0 to 3
- is_reversed (bool): Whether or not the motor is reversed.
- encoder_index (uint8_t): The index of the encoder to use for the controller.
  - Range: 0 to 3
- is_encoder_reversed (bool): Reverses the encoder counting direction, independently of the motor's is_reversed. For closed-loop control the encoder must report a positive measured speed when a positive speed is commanded (negative feedback); if the controller runs away, flip this flag.
- encoder_resolution (double): Encoder resolution in ticks per revolution. The value can not be negative or zero.
- kp (double): Proportional constant of PID
- ki (double): Integral constant of PID
- kd (double): Derivative constant of PID
- integral_limit (double): Integral limit of PID controller. The value can not be negative or zero. If the value is zero or negative, the integral limit is disabled.

Errors:

- `INVALID_LENGTH` (4): The complete message has an invalid header or payload length.
- `INVALID_ARGUMENT` (2): A field is invalid, out of range, or inconsistent with the pending request.
- `INTERNAL_ERROR` (6): The controller could not execute or encode the operation.
- `MOTOR_OWNED` (5): The motor belongs to a platform; use the corresponding platform command.

### SET_MOTOR_TARGET_SPEED (0x06)
Direction: `client_to_controller`\
Description: This command sets the target speed for the specified motor in radians. Rejected with MOTOR_OWNED if the motor is currently owned by an active platform (one of its wheels); use SET_PLATFORM_TARGET_VELOCITY to drive platform wheels.\
Properties:
- motor_index (uint8_t): The index of the motor to set the target velocity for.
  - Range: 0 to 3
- speed (double): The speed of the motor.

Errors:

- `INVALID_LENGTH` (4): The complete message has an invalid header or payload length.
- `INVALID_ARGUMENT` (2): A field is invalid, out of range, or inconsistent with the pending request.
- `INTERNAL_ERROR` (6): The controller could not execute or encode the operation.
- `MOTOR_OWNED` (5): The motor belongs to a platform; use the corresponding platform command.
- `CONTROLLER_NOT_INITIALIZED` (14): Start or initialize the closed-loop controller before setting its target.

### RESET_MOTOR_CONTROLLER (0x07)
Direction: `client_to_controller`\
Description: This command resets the closed-loop controller for the specified motor: it clears the accumulated PID state (integrator windup, derivative history and internal output) and re-zeros the target speed, while keeping the controller running with its existing tuning (kp/ki/kd). Use it to recover from integrator windup or to bring a motor cleanly to a stop without deleting and re-initializing the controller. No effect if no controller is running for that motor, and ignored if the motor is currently owned by an active platform (one of its wheels).\
Properties:
- motor_index (uint8_t): The index of the motor to reset the controller for.
  - Range: 0 to 3

Errors:

- `INVALID_LENGTH` (4): The complete message has an invalid header or payload length.
- `INVALID_ARGUMENT` (2): A field is invalid, out of range, or inconsistent with the pending request.
- `INTERNAL_ERROR` (6): The controller could not execute or encode the operation.
- `MOTOR_OWNED` (5): The motor belongs to a platform; use the corresponding platform command.

### GET_MOTOR_CONTROLLER_STATE (0x08)
Direction: `client_to_controller`\
Description: This command gets the state of the controller for the specified motor.\
Properties:
- motor_index (uint8_t): The index of the motor to get the state for.
  - Range: 0 to 3
Response (`controller_to_client`):
 - motor_controller_state (object): The state of the controller for the specified motor.

Errors:

- `INVALID_LENGTH` (4): The complete message has an invalid header or payload length.
- `INVALID_ARGUMENT` (2): A field is invalid, out of range, or inconsistent with the pending request.
- `INTERNAL_ERROR` (6): The controller could not execute or encode the operation.

### DELETE_MOTOR_CONTROLLER (0x09)
Direction: `client_to_controller`\
Description: This command deletes the controller for the specified motor. Rejected with MOTOR_OWNED if the motor is currently owned by an active platform (one of its wheels); use STOP_PLATFORM_CONTROLLER to stop the platform controller instead.\
Properties:
- motor_index (uint8_t): The index of the motor to delete the controller for.
  - Range: 0 to 3

Errors:

- `INVALID_LENGTH` (4): The complete message has an invalid header or payload length.
- `INVALID_ARGUMENT` (2): A field is invalid, out of range, or inconsistent with the pending request.
- `INTERNAL_ERROR` (6): The controller could not execute or encode the operation.
- `MOTOR_OWNED` (5): The motor belongs to a platform; use the corresponding platform command.

### SET_CONTROLLER_FREQUENCY (0x0A)
Direction: `client_to_controller`\
Description: This command sets the global update frequency (in Hz) of the closed-loop motor controller task. All motor controllers share a single control loop, so this frequency is global and affects every currently running controller as well as any created afterwards; the PID sampling time is updated to match. The requested value is clamped to the supported range of 1 to 1000 Hz (the 1000 Hz maximum is bounded by the 1 ms RTOS tick). The value is then quantized to the 1 ms RTOS tick (period_ms = 1000 / frequency), so effective frequencies are 1000/N Hz. A value of 0 is invalid and ignored. Defaults to 10 Hz (100 ms) at start-up.\
Properties:
- frequency (uint16_t): The controller update frequency in Hz. Valid range 1 to 1000 Hz; values outside are clamped, and 0 is ignored.
  - Range: 1 to 1000

Errors:

- `INVALID_LENGTH` (4): The complete message has an invalid header or payload length.
- `INVALID_ARGUMENT` (2): A field is invalid, out of range, or inconsistent with the pending request.
- `INTERNAL_ERROR` (6): The controller could not execute or encode the operation.

### GET_CONTROLLER_FREQUENCY (0x0B)
Direction: `client_to_controller`\
Description: This command retrieves the current global update frequency (in Hz) of the closed-loop motor controller task.\
Properties:
- None
Response (`controller_to_client`):
 - frequency (uint16_t): The current controller update frequency in Hz (1 to 1000 Hz).

Errors:

- `INVALID_LENGTH` (4): The complete message has an invalid header or payload length.
- `INVALID_ARGUMENT` (2): A field is invalid, out of range, or inconsistent with the pending request.
- `INTERNAL_ERROR` (6): The controller could not execute or encode the operation.

### INITIALIZE_ENCODER (0x11)
Direction: `client_to_controller`\
Description: This command initializes an encoder and prepares it for use.\
Properties:
- encoder_index (uint8_t): The index of the encoder to initialize.
  - Range: 0 to 3
- encoder_resolution (double): Encoder resolution in ticks per revolution. The value can not be negative or zero.
- is_reversed (bool): Whether or not the encoder is reversed.

Errors:

- `INVALID_LENGTH` (4): The complete message has an invalid header or payload length.
- `INVALID_ARGUMENT` (2): A field is invalid, out of range, or inconsistent with the pending request.
- `INTERNAL_ERROR` (6): The controller could not execute or encode the operation.

### GET_ENCODER_VALUE (0x12)
Direction: `client_to_controller`\
Description: This command retrieves the current value of the encoder.\
Properties:
- encoder_index (uint8_t): The index of the encoder to retrieve the value for.
  - Range: 0 to 3
Response (`controller_to_client`):
 - encoderValue (uint16_t): The current value of the encoder.

Errors:

- `INVALID_LENGTH` (4): The complete message has an invalid header or payload length.
- `INVALID_ARGUMENT` (2): A field is invalid, out of range, or inconsistent with the pending request.
- `INTERNAL_ERROR` (6): The controller could not execute or encode the operation.
- `ENCODER_NOT_INITIALIZED` (11): Initialize the encoder before reading it or starting encoder odometry.

### START_ENCODER_ODOMETRY (0x13)
Direction: `client_to_controller`\
Description: This command starts the odometry calculation for the specified encoder.\
Properties:
- encoder_index (uint8_t): The index of the encoder to start the odometry calculation for.
  - Range: 0 to 3

Errors:

- `INVALID_LENGTH` (4): The complete message has an invalid header or payload length.
- `INVALID_ARGUMENT` (2): A field is invalid, out of range, or inconsistent with the pending request.
- `INTERNAL_ERROR` (6): The controller could not execute or encode the operation.
- `INIT_REQUIRED` (15): Send a valid INIT request before performing this connection-dependent operation.
- `CLOCK_NOT_READY` (7): Initial clock setup is incomplete; wait for READY. Uptime mode also supports READY.
- `ENCODER_NOT_INITIALIZED` (11): Initialize the encoder before reading it or starting encoder odometry.

### RESET_ENCODER_ODOMETRY (0x14)
Direction: `client_to_controller`\
Description: This command resets the odometry calculation for the specified encoder.\
Properties:
- encoder_index (uint8_t): The index of the encoder to reset the odometry calculation for.
  - Range: 0 to 3

Errors:

- `INVALID_LENGTH` (4): The complete message has an invalid header or payload length.
- `INVALID_ARGUMENT` (2): A field is invalid, out of range, or inconsistent with the pending request.
- `INTERNAL_ERROR` (6): The controller could not execute or encode the operation.

### STOP_ENCODER_ODOMETRY (0x15)
Direction: `client_to_controller`\
Description: This command stops the odometry calculation for the specified encoder.\
Properties:
- encoder_index (uint8_t): The index of the encoder to stop the odometry calculation for.
  - Range: 0 to 3

Errors:

- `INVALID_LENGTH` (4): The complete message has an invalid header or payload length.
- `INVALID_ARGUMENT` (2): A field is invalid, out of range, or inconsistent with the pending request.
- `INTERNAL_ERROR` (6): The controller could not execute or encode the operation.

### GET_ENCODER_ODOMETRY (0x16)
Direction: `client_to_controller`\
Description: This command retrieves the odometry of the specified encoder.\
Properties:
- encoder_index (uint8_t): The index of the encoder to retrieve the odometry for.
  - Range: 0 to 3
Response (`controller_to_client`):
 - sample (encoder_odometry_sample): Timestamped odometry measurement. See errors for missing INIT, clock readiness, odometry initialization, and sample availability.

Errors:

- `INVALID_LENGTH` (4): The complete message has an invalid header or payload length.
- `INVALID_ARGUMENT` (2): A field is invalid, out of range, or inconsistent with the pending request.
- `INTERNAL_ERROR` (6): The controller could not execute or encode the operation.
- `INIT_REQUIRED` (15): Send a valid INIT request before performing this connection-dependent operation.
- `CLOCK_NOT_READY` (7): Initial clock setup is incomplete; wait for READY. Uptime mode also supports READY.
- `ODOMETRY_NOT_INITIALIZED` (9): Odometry is not running; start odometry before requesting a measurement.
- `SAMPLE_NOT_AVAILABLE` (10): Odometry is running but has no measurement yet, including immediately after reset; retry after an update.

### SET_ODOMETRY_FREQUENCY (0x17)
Direction: `client_to_controller`\
Description: This command sets the global update frequency (in Hz) of the odometry task. A single odometry task integrates all encoder and platform odometry, so this frequency is global. The requested value is clamped to the supported range of 1 to 1000 Hz (the 1000 Hz maximum is bounded by the 1 ms RTOS tick). The value is then quantized to the 1 ms RTOS tick (period_ms = 1000 / frequency), so effective frequencies are 1000/N Hz. A value of 0 is invalid and ignored. Defaults to 20 Hz (50 ms) at start-up.\
Properties:
- frequency (uint16_t): The odometry update frequency in Hz. Valid range 1 to 1000 Hz; values outside are clamped, and 0 is ignored.
  - Range: 1 to 1000

Errors:

- `INVALID_LENGTH` (4): The complete message has an invalid header or payload length.
- `INVALID_ARGUMENT` (2): A field is invalid, out of range, or inconsistent with the pending request.
- `INTERNAL_ERROR` (6): The controller could not execute or encode the operation.

### GET_ODOMETRY_FREQUENCY (0x18)
Direction: `client_to_controller`\
Description: This command retrieves the current global update frequency (in Hz) of the odometry task.\
Properties:
- None
Response (`controller_to_client`):
 - frequency (uint16_t): The current odometry update frequency in Hz (1 to 1000 Hz).

Errors:

- `INVALID_LENGTH` (4): The complete message has an invalid header or payload length.
- `INVALID_ARGUMENT` (2): A field is invalid, out of range, or inconsistent with the pending request.
- `INTERNAL_ERROR` (6): The controller could not execute or encode the operation.

### INITIALIZE_GPIO_PIN (0x20)
Direction: `client_to_controller`\
Description: This command initializes a digital pin and prepares it for use.\
Properties:
- pin_number (uint8_t): The number of the pin to initialize.
- mode (uint8_t): Set digital pin as input or output. Modes: 0 = INPUT_PULLDOWN, 1 = INPUT_PULLUP, 2 = INPUT_NOPULL, 3 = OUTPUT.

Errors:

- `INVALID_LENGTH` (4): The complete message has an invalid header or payload length.
- `INVALID_ARGUMENT` (2): A field is invalid, out of range, or inconsistent with the pending request.
- `INTERNAL_ERROR` (6): The controller could not execute or encode the operation.

### SET_GPIO_PIN_STATE (0x21)
Direction: `client_to_controller`\
Description: This command sets the specified pin to a state.\
Properties:
- pin_number (uint8_t): The number of the pin to set to a state.
- state (uint8_t): The state of the pin. 0 = LOW, 1 = HIGH.

Errors:

- `INVALID_LENGTH` (4): The complete message has an invalid header or payload length.
- `INVALID_ARGUMENT` (2): A field is invalid, out of range, or inconsistent with the pending request.
- `INTERNAL_ERROR` (6): The controller could not execute or encode the operation.

### GET_GPIO_PIN_STATE (0x22)
Direction: `client_to_controller`\
Description: This command gets the state of the specified pin.\
Properties:
- pin_number (uint8_t): The number of the pin to get the state for.
Response (`controller_to_client`):
 - state (uint8_t): The state of the pin. 0 = LOW, 1 = HIGH.

Errors:

- `INVALID_LENGTH` (4): The complete message has an invalid header or payload length.
- `INVALID_ARGUMENT` (2): A field is invalid, out of range, or inconsistent with the pending request.
- `INTERNAL_ERROR` (6): The controller could not execute or encode the operation.

### TOGGLE_GPIO_PIN_STATE (0x23)
Direction: `client_to_controller`\
Description: This command toggles the specified pin.\
Properties:
- pin_number (uint8_t): The number of the pin to toggle.

Errors:

- `INVALID_LENGTH` (4): The complete message has an invalid header or payload length.
- `INVALID_ARGUMENT` (2): A field is invalid, out of range, or inconsistent with the pending request.
- `INTERNAL_ERROR` (6): The controller could not execute or encode the operation.

### SET_STATUS_LED_STATE (0x25)
Direction: `client_to_controller`\
Description: This command sets the status LED to a state.\
Properties:
- state (uint8_t): The state of the status LED. 0 = OFF, 1 = ON.

Errors:

- `INVALID_LENGTH` (4): The complete message has an invalid header or payload length.
- `INVALID_ARGUMENT` (2): A field is invalid, out of range, or inconsistent with the pending request.
- `INTERNAL_ERROR` (6): The controller could not execute or encode the operation.

### TOGGLE_STATUS_LED_STATE (0x26)
Direction: `client_to_controller`\
Description: This command toggles the status LED.\
Properties:
- None

Errors:

- `INVALID_LENGTH` (4): The complete message has an invalid header or payload length.
- `INVALID_ARGUMENT` (2): A field is invalid, out of range, or inconsistent with the pending request.
- `INTERNAL_ERROR` (6): The controller could not execute or encode the operation.

### INITIALIZE_MECANUM_PLATFORM (0x30)
Direction: `client_to_controller`\
Description: This command initializes a mecanum (4-wheel) platform and prepares it for use. It uses motor and encoder indices 0, 1, 2 and 3 (one per wheel), which correspond to the is_reversed_0..3 and is_encoder_reversed_0..3 parameters. All four motor slots are occupied by this platform.\
Properties:
- is_reversed_0 (bool): Determines if motor 0 is reversed.
- is_reversed_1 (bool): Determines if motor 1 is reversed.
- is_reversed_2 (bool): Determines if motor 2 is reversed.
- is_reversed_3 (bool): Determines if motor 3 is reversed.
- is_encoder_reversed_0 (bool): Reverses encoder 0 counting direction, independently of motor 0's is_reversed_0. Set so the closed-loop feedback is negative (flip if the wheel runs away when given a target speed).
- is_encoder_reversed_1 (bool): Reverses encoder 1 counting direction, independently of motor 1's is_reversed_1. Set so the closed-loop feedback is negative (flip if the wheel runs away when given a target speed).
- is_encoder_reversed_2 (bool): Reverses encoder 2 counting direction, independently of motor 2's is_reversed_2. Set so the closed-loop feedback is negative (flip if the wheel runs away when given a target speed).
- is_encoder_reversed_3 (bool): Reverses encoder 3 counting direction, independently of motor 3's is_reversed_3. Set so the closed-loop feedback is negative (flip if the wheel runs away when given a target speed).
- length (double): Length of the platform in meters.
- width (double): Width of the platform in meters.
- wheels_diameter (double): Diameter of the robot wheels in meters.
- encoder_resolution (double): Encoder resolution in ticks per revolution. The value can not be negative. If platform does not have encoders, the value should be set to zero.

Errors:

- `INVALID_LENGTH` (4): The complete message has an invalid header or payload length.
- `INVALID_ARGUMENT` (2): A field is invalid, out of range, or inconsistent with the pending request.
- `INTERNAL_ERROR` (6): The controller could not execute or encode the operation.

### INITIALIZE_OMNI_PLATFORM (0x31)
Direction: `client_to_controller`\
Description: This command initializes an omni (3-wheel) platform and prepares it for use. It uses motor and encoder indices 0, 1 and 2 (one per wheel), which correspond to the is_reversed_0..2 and is_encoder_reversed_0..2 parameters. Motor index 3 is not used by this platform and stays free for other purposes.\
Properties:
- is_reversed_0 (bool): Determines if motor 0 is reversed.
- is_reversed_1 (bool): Determines if motor 1 is reversed.
- is_reversed_2 (bool): Determines if motor 2 is reversed.
- is_encoder_reversed_0 (bool): Reverses encoder 0 counting direction, independently of motor 0's is_reversed_0. Set so the closed-loop feedback is negative (flip if the wheel runs away when given a target speed).
- is_encoder_reversed_1 (bool): Reverses encoder 1 counting direction, independently of motor 1's is_reversed_1. Set so the closed-loop feedback is negative (flip if the wheel runs away when given a target speed).
- is_encoder_reversed_2 (bool): Reverses encoder 2 counting direction, independently of motor 2's is_reversed_2. Set so the closed-loop feedback is negative (flip if the wheel runs away when given a target speed).
- wheels_diameter (double): Diameter of the robot wheels in millimeters.
- robot_radius (double): Distance berween the center of the robot and the center of the wheels in millimeters.
- encoder_resolution (double): Encoder resolution in ticks per revolution. The value can not be negative. If platform does not have encoders, the value should be set to zero.

Errors:

- `INVALID_LENGTH` (4): The complete message has an invalid header or payload length.
- `INVALID_ARGUMENT` (2): A field is invalid, out of range, or inconsistent with the pending request.
- `INTERNAL_ERROR` (6): The controller could not execute or encode the operation.

### INITIALIZE_DIFFERENTIAL_PLATFORM (0x32)
Direction: `client_to_controller`\
Description: This command initializes a differential (2-wheel) platform and prepares it for use. It uses motor and encoder index 0 for the left wheel and index 1 for the right wheel, which correspond to the is_reversed_0/1 and is_encoder_reversed_0/1 parameters. Motor indices 2 and 3 are not used by this platform and stay free for other purposes.\
Properties:
- is_reversed_0 (bool): Determines if motor 0 is reversed.
- is_reversed_1 (bool): Determines if motor 1 is reversed.
- is_encoder_reversed_0 (bool): Reverses encoder 0 counting direction, independently of motor 0's is_reversed_0. Set so the closed-loop feedback is negative (flip if the wheel runs away when given a target speed).
- is_encoder_reversed_1 (bool): Reverses encoder 1 counting direction, independently of motor 1's is_reversed_1. Set so the closed-loop feedback is negative (flip if the wheel runs away when given a target speed).
- wheel_diameter (double): Diameter of the robot wheels in meters.
- wheel_base (double): Distance between the two wheels in meters.
- encoder_resolution (double): Encoder resolution in ticks per revolution. The value can not be negative. If platform does not have encoders, the value should be set to zero.

Errors:

- `INVALID_LENGTH` (4): The complete message has an invalid header or payload length.
- `INVALID_ARGUMENT` (2): A field is invalid, out of range, or inconsistent with the pending request.
- `INTERNAL_ERROR` (6): The controller could not execute or encode the operation.

### SET_PLATFORM_VELOCITY (0x40)
Direction: `client_to_controller`\
Description: This command sets the velocity for the platform in PWM.\
Properties:
- x (double): X component of platform velocity in PWM
  - Range: -100.0 to 100.0
- y (double): Y component of platform velocity in PWM
  - Range: -100.0 to 100.0
- t (double): Theta component of platform velocity in PWM
  - Range: -100.0 to 100.0

Errors:

- `INVALID_LENGTH` (4): The complete message has an invalid header or payload length.
- `INVALID_ARGUMENT` (2): A field is invalid, out of range, or inconsistent with the pending request.
- `INTERNAL_ERROR` (6): The controller could not execute or encode the operation.
- `PLATFORM_NOT_INITIALIZED` (12): Initialize a platform before performing this operation.

### START_PLATFORM_CONTROLLER (0x41)
Direction: `client_to_controller`\
Description: This command sets the controller for the platform.\
Properties:
- kp (double): Proportional constant of PID
- ki (double): Integral constant of PID
- kd (double): Derivative constant of PID
- integral_limit (double): Integral limit of PID controller. The value can not be negative or zero. If the value is zero or negative, the integral limit is disabled.

Errors:

- `INVALID_LENGTH` (4): The complete message has an invalid header or payload length.
- `INVALID_ARGUMENT` (2): A field is invalid, out of range, or inconsistent with the pending request.
- `INTERNAL_ERROR` (6): The controller could not execute or encode the operation.
- `PLATFORM_NOT_INITIALIZED` (12): Initialize a platform before performing this operation.

### SET_PLATFORM_TARGET_VELOCITY (0x42)
Direction: `client_to_controller`\
Description: This command set the target velocity for the platform in meters per second.\
Properties:
- x (double): X component of platform velocity in meters per second
- y (double): Y component of platform velocity in meters per second
- t (double): Theta component of platform velocity in radians per second

Errors:

- `INVALID_LENGTH` (4): The complete message has an invalid header or payload length.
- `INVALID_ARGUMENT` (2): A field is invalid, out of range, or inconsistent with the pending request.
- `INTERNAL_ERROR` (6): The controller could not execute or encode the operation.
- `PLATFORM_NOT_INITIALIZED` (12): Initialize a platform before performing this operation.
- `CONTROLLER_NOT_INITIALIZED` (14): Start or initialize the closed-loop controller before setting its target.

### GET_PLATFORM_CURRENT_VELOCITY (0x43)
Direction: `client_to_controller`\
Description: This command gets the current velocity of the platform in meters per second.\
Properties:
- None
Response (`controller_to_client`):
 - platform_velocity (object): The current velocity of the platform in meters per second.

Errors:

- `INVALID_LENGTH` (4): The complete message has an invalid header or payload length.
- `INVALID_ARGUMENT` (2): A field is invalid, out of range, or inconsistent with the pending request.
- `INTERNAL_ERROR` (6): The controller could not execute or encode the operation.

### STOP_PLATFORM_CONTROLLER (0x44)
Direction: `client_to_controller`\
Description: This command stops the controller for the platform.\
Properties:
- None

Errors:

- `INVALID_LENGTH` (4): The complete message has an invalid header or payload length.
- `INVALID_ARGUMENT` (2): A field is invalid, out of range, or inconsistent with the pending request.
- `INTERNAL_ERROR` (6): The controller could not execute or encode the operation.

### START_PLATFORM_ODOMETRY (0x45)
Direction: `client_to_controller`\
Description: This command starts the odometry calculation for the platform.\
Properties:
- None

Errors:

- `INVALID_LENGTH` (4): The complete message has an invalid header or payload length.
- `INVALID_ARGUMENT` (2): A field is invalid, out of range, or inconsistent with the pending request.
- `INTERNAL_ERROR` (6): The controller could not execute or encode the operation.
- `INIT_REQUIRED` (15): Send a valid INIT request before performing this connection-dependent operation.
- `CLOCK_NOT_READY` (7): Initial clock setup is incomplete; wait for READY. Uptime mode also supports READY.
- `PLATFORM_NOT_INITIALIZED` (12): Initialize a platform before performing this operation.

### RESET_PLATFORM_ODOMETRY (0x46)
Direction: `client_to_controller`\
Description: This command resets the odometry calculation for the platform.\
Properties:
- None

Errors:

- `INVALID_LENGTH` (4): The complete message has an invalid header or payload length.
- `INVALID_ARGUMENT` (2): A field is invalid, out of range, or inconsistent with the pending request.
- `INTERNAL_ERROR` (6): The controller could not execute or encode the operation.

### STOP_PLATFORM_ODOMETRY (0x47)
Direction: `client_to_controller`\
Description: This command stops the odometry calculation for the platform.\
Properties:
- None

Errors:

- `INVALID_LENGTH` (4): The complete message has an invalid header or payload length.
- `INVALID_ARGUMENT` (2): A field is invalid, out of range, or inconsistent with the pending request.
- `INTERNAL_ERROR` (6): The controller could not execute or encode the operation.

### GET_PLATFORM_ODOMETRY (0x48)
Direction: `client_to_controller`\
Description: This command retrieves the odometry of the platform in meters and radians.\
Properties:
- None
Response (`controller_to_client`):
 - sample (platform_odometry_sample): Timestamped odometry measurement. See errors for missing INIT, clock readiness, odometry initialization, and sample availability.

Errors:

- `INVALID_LENGTH` (4): The complete message has an invalid header or payload length.
- `INVALID_ARGUMENT` (2): A field is invalid, out of range, or inconsistent with the pending request.
- `INTERNAL_ERROR` (6): The controller could not execute or encode the operation.
- `INIT_REQUIRED` (15): Send a valid INIT request before performing this connection-dependent operation.
- `CLOCK_NOT_READY` (7): Initial clock setup is incomplete; wait for READY. Uptime mode also supports READY.
- `ODOMETRY_NOT_INITIALIZED` (9): Odometry is not running; start odometry before requesting a measurement.
- `SAMPLE_NOT_AVAILABLE` (10): Odometry is running but has no measurement yet, including immediately after reset; retry after an update.

### BRAKE_PLATFORM (0x49)
Direction: `client_to_controller`\
Description: This command actively brakes all of this platform's wheel motors (short brake) so they resist motion and hold position, and stops the platform velocity controller if it is running (you must call START_PLATFORM_CONTROLLER again to resume closed-loop platform control). Motors used outside this platform are not affected. The motors resist motion until a new command is issued.\
Properties:
- None

Errors:

- `INVALID_LENGTH` (4): The complete message has an invalid header or payload length.
- `INVALID_ARGUMENT` (2): A field is invalid, out of range, or inconsistent with the pending request.
- `INTERNAL_ERROR` (6): The controller could not execute or encode the operation.

### COAST_PLATFORM (0x4A)
Direction: `client_to_controller`\
Description: This command lets all of this platform's wheel motors coast freely (high impedance) so they spin down without resistance, and stops the platform velocity controller if it is running (you must call START_PLATFORM_CONTROLLER again to resume closed-loop platform control). Motors used outside this platform are not affected. The motors spin down without resistance.\
Properties:
- None

Errors:

- `INVALID_LENGTH` (4): The complete message has an invalid header or payload length.
- `INVALID_ARGUMENT` (2): A field is invalid, out of range, or inconsistent with the pending request.
- `INTERNAL_ERROR` (6): The controller could not execute or encode the operation.

### INIT (0x70)
Direction: `client_to_controller`\
Description: Exchange identity and start per-connection clock initialization. Wait for READY before odometry. Clients with wall-clock capability answer controller TIME_SYNC_REQUEST messages; other clients use uptime. Repeating INIT restarts clock setup without altering motor state.\
Properties:
- sdk_type (uint8_t): 0 unknown, 1 Python, 2 JavaScript, 3 Arduino.
- client_version_major (uint8_t): Client library major version.
- client_version_minor (uint8_t): Client library minor version.
- client_version_patch (uint8_t): Client library patch version.
- protocol_major (uint8_t): Client protocol major version.
- protocol_minor (uint8_t): Minimum required protocol minor version.
- protocol_patch (uint8_t): Client protocol patch version, informational; compatibility is determined by major and minor.
- client_capabilities (uint8_t): Bit 0 (1): valid wall-clock time available. Bit 1 (2): client supports telemetry subscriptions. Combine as 3 for both; 0 for neither. Other bits must be zero. Declaring support does not start time synchronization or create subscriptions.
Response (`controller_to_client`):
 - identity (init_response): Board revision, protocol version and firmware build identity. Failures use ERROR with the failed command ID.

Errors:

- `INVALID_LENGTH` (4): The complete message has an invalid header or payload length.
- `INVALID_ARGUMENT` (2): A field is invalid, out of range, or inconsistent with the pending request.
- `INTERNAL_ERROR` (6): The controller could not execute or encode the operation.
- `INCOMPATIBLE_PROTOCOL` (1): The requested protocol version is incompatible with this firmware.
- `TIME_SYNC_FAILED` (8): Initial time sync exhausted its attempts without a valid sample.

### ERROR (0x7F)
Direction: `controller_to_client`\
Description: Response-only error message. Payload: failed command ID (uint8_t), error code (uint8_t). Uses the common length and command header; never accepted as a request.\
Properties:
- failed_command (uint8_t): Command ID of the failed request; zero if unavailable.
- error_code (uint8_t): Shared nonzero protocol error code.

Errors:

- None; this message does not receive an error reply in its declared direction.

### TIME_SYNC_REQUEST (0x71)
Direction: `controller_to_client`\
Description: Controller-initiated timing request. Client echoes the header message ID in TIME_SYNC_RESPONSE.\
Properties:
- None

Errors:

- None; this message does not receive an error reply in its declared direction.

### TIME_SYNC_RESPONSE (0x72)
Direction: `client_to_controller`\
Description: Reply to a pending controller timing request; consumed without an ACK.\
Properties:
- host_receive_us (uint64_t): Client wall-clock Unix microseconds when TIME_SYNC_REQUEST was received.
- host_send_us (uint64_t): Client wall-clock Unix microseconds immediately before its reply is sent.

Errors:

- `INVALID_LENGTH` (4): The complete message has an invalid header or payload length.
- `INVALID_ARGUMENT` (2): A field is invalid, out of range, or inconsistent with the pending request.

### READY (0x73)
Direction: `controller_to_client`\
Description: Initial clock setup completed; echoes INIT message ID. Client can now request odometry.\
Properties:
- clock_mode (uint8_t): 0 controller uptime; 1 synchronized Unix wall time.

Errors:

- None; this message does not receive an error reply in its declared direction.

### SET_TIME_SYNC_INTERVAL (0x74)
Direction: `client_to_controller`\
Description: Set independent time-sync refresh interval for this connection. Default 30000 ms.\
Properties:
- interval_ms (uint32_t): Periodic resynchronization interval in milliseconds.
  - Range: 1000 to 3600000

Errors:

- `INVALID_LENGTH` (4): The complete message has an invalid header or payload length.
- `INVALID_ARGUMENT` (2): A field is invalid, out of range, or inconsistent with the pending request.
- `INTERNAL_ERROR` (6): The controller could not execute or encode the operation.
- `INIT_REQUIRED` (15): Send a valid INIT request before performing this connection-dependent operation.

### GET_TIME_STATUS (0x75)
Direction: `client_to_controller`\
Description: Read this connection's clock mode, quality, interval and age.\
Properties:
- None
Response (`controller_to_client`):
 - clock (time_status): Clock status. Quality 0 unready, 1 valid, 2 stale.

Errors:

- `INVALID_LENGTH` (4): The complete message has an invalid header or payload length.
- `INVALID_ARGUMENT` (2): A field is invalid, out of range, or inconsistent with the pending request.
- `INTERNAL_ERROR` (6): The controller could not execute or encode the operation.

