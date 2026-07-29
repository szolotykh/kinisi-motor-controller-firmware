# Kinisi motor controller commands

Version: 1.3.0
---

## Commands
### INITIALIZE_MOTOR (0x01)
Description: This command initializes a motor and prepares it for use. Ignored if the motor is currently owned by an active platform (one of its wheels), so platform wheels are not reconfigured out from under the platform.\
Properties:
- motor_index (uint8_t): The index of the motor to initialize.
  - Range: 0 to 3
- is_reversed (bool): Whether or not the motor is reversed.

### SET_MOTOR_SPEED (0x02)
Description: This command sets the speed of the specified motor in PWM. Ignored if the motor is currently owned by an active platform (one of its wheels); use the platform velocity commands to drive platform wheels.\
Properties:
- motor_index (uint8_t): The index of the motor to set the speed for.
  - Range: 0 to 3
- pwm (double): The speed of the motor.
  - Range: -100.0 to 100.0

### STOP_MOTOR (0x03)
Description: Coasts the motor to a stop: both H-bridge outputs are driven low, leaving the motor terminals open (high impedance) so it free-wheels and spins down gradually under its own friction. This also stops that motor's closed-loop speed controller if one is running (started via INITIALIZE_MOTOR_CONTROLLER), so the PID loop cannot re-drive the motor; to command the motor by target speed again you must re-initialize its controller. This is a single-motor command and is ignored if the motor is currently owned by an active platform (one of its wheels); to stop a platform, use STOP_PLATFORM_CONTROLLER, COAST_PLATFORM or BRAKE_PLATFORM instead. Use STOP_MOTOR for a soft, low-stress stop; use BRAKE_MOTOR when you need the motor to hold position and stop quickly.\
Properties:
- motor_index (uint8_t): The index of the motor to set the speed for.
  - Range: 0 to 3

### BRAKE_MOTOR (0x04)
Description: Actively brakes the motor (short brake): both H-bridge outputs are driven high, shorting the motor terminals together so the motor's own back-EMF resists rotation and it stops quickly and holds position. This also stops that motor's closed-loop speed controller if one is running (started via INITIALIZE_MOTOR_CONTROLLER), so the PID loop cannot re-drive the motor; to command the motor by target speed again you must re-initialize its controller. This is a single-motor command and is ignored if the motor is currently owned by an active platform (one of its wheels); to brake a platform, use BRAKE_PLATFORM (or STOP_PLATFORM_CONTROLLER / COAST_PLATFORM) instead. Use BRAKE_MOTOR for a fast, holding stop; use STOP_MOTOR to let the motor coast freely instead.\
Properties:
- motor_index (uint8_t): The index of the motor to set the speed for.
  - Range: 0 to 3

### INITIALIZE_MOTOR_CONTROLLER (0x05)
Description: This command sets the controller for the specified motor. Ignored if the motor is currently owned by an active platform (one of its wheels), so it cannot create a competing controller on a platform wheel.\
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

### SET_MOTOR_TARGET_SPEED (0x06)
Description: This command sets the target speed for the specified motor in radians. Ignored if the motor is currently owned by an active platform (one of its wheels); use SET_PLATFORM_TARGET_VELOCITY to drive platform wheels.\
Properties:
- motor_index (uint8_t): The index of the motor to set the target velocity for.
  - Range: 0 to 3
- speed (double): The speed of the motor.

### RESET_MOTOR_CONTROLLER (0x07)
Description: This command resets the closed-loop controller for the specified motor: it clears the accumulated PID state (integrator windup, derivative history and internal output) and re-zeros the target speed, while keeping the controller running with its existing tuning (kp/ki/kd). Use it to recover from integrator windup or to bring a motor cleanly to a stop without deleting and re-initializing the controller. No effect if no controller is running for that motor, and ignored if the motor is currently owned by an active platform (one of its wheels).\
Properties:
- motor_index (uint8_t): The index of the motor to reset the controller for.
  - Range: 0 to 3

### GET_MOTOR_CONTROLLER_STATE (0x08)
Description: This command gets the state of the controller for the specified motor.\
Properties:
- motor_index (uint8_t): The index of the motor to get the state for.
  - Range: 0 to 3
Response: 
 - motor_controller_state (object): The state of the controller for the specified motor.

### DELETE_MOTOR_CONTROLLER (0x09)
Description: This command deletes the controller for the specified motor. Ignored if the motor is currently owned by an active platform (one of its wheels); use STOP_PLATFORM_CONTROLLER to stop the platform controller instead.\
Properties:
- motor_index (uint8_t): The index of the motor to delete the controller for.
  - Range: 0 to 3

### INITIALIZE_ENCODER (0x11)
Description: This command initializes an encoder and prepares it for use.\
Properties:
- encoder_index (uint8_t): The index of the encoder to initialize.
  - Range: 0 to 3
- encoder_resolution (double): Encoder resolution in ticks per revolution. The value can not be negative or zero.
- is_reversed (bool): Whether or not the encoder is reversed.

### GET_ENCODER_VALUE (0x12)
Description: This command retrieves the current value of the encoder.\
Properties:
- encoder_index (uint8_t): The index of the encoder to retrieve the value for.
  - Range: 0 to 3
Response: 
 - encoderValue (uint16_t): The current value of the encoder.

### START_ENCODER_ODOMETRY (0x13)
Description: This command starts the odometry calculation for the specified encoder.\
Properties:
- encoder_index (uint8_t): The index of the encoder to start the odometry calculation for.
  - Range: 0 to 3

### RESET_ENCODER_ODOMETRY (0x14)
Description: This command resets the odometry calculation for the specified encoder.\
Properties:
- encoder_index (uint8_t): The index of the encoder to reset the odometry calculation for.
  - Range: 0 to 3

### STOP_ENCODER_ODOMETRY (0x15)
Description: This command stops the odometry calculation for the specified encoder.\
Properties:
- encoder_index (uint8_t): The index of the encoder to stop the odometry calculation for.
  - Range: 0 to 3

### GET_ENCODER_ODOMETRY (0x16)
Description: This command retrieves the odometry of the specified encoder.\
Properties:
- encoder_index (uint8_t): The index of the encoder to retrieve the odometry for.
  - Range: 0 to 3
Response: 
 - odometry (double): The odometry of the encoder in radians.

### INITIALIZE_GPIO_PIN (0x20)
Description: This command initializes a digital pin and prepares it for use.\
Properties:
- pin_number (uint8_t): The number of the pin to initialize.
- mode (uint8_t): Set digital pin as input or output. Modes: 0 = INPUT_PULLDOWN, 1 = INPUT_PULLUP, 2 = INPUT_NOPULL, 3 = OUTPUT.

### SET_GPIO_PIN_STATE (0x21)
Description: This command sets the specified pin to a state.\
Properties:
- pin_number (uint8_t): The number of the pin to set to a state.
- state (uint8_t): The state of the pin. 0 = LOW, 1 = HIGH.

### GET_GPIO_PIN_STATE (0x22)
Description: This command gets the state of the specified pin.\
Properties:
- pin_number (uint8_t): The number of the pin to get the state for.
Response: 
 - state (uint8_t): The state of the pin. 0 = LOW, 1 = HIGH.

### TOGGLE_GPIO_PIN_STATE (0x23)
Description: This command toggles the specified pin.\
Properties:
- pin_number (uint8_t): The number of the pin to toggle.

### SET_STATUS_LED_STATE (0x25)
Description: This command sets the status LED to a state.\
Properties:
- state (uint8_t): The state of the status LED. 0 = OFF, 1 = ON.

### TOGGLE_STATUS_LED_STATE (0x26)
Description: This command toggles the status LED.\
Properties:
- None

### INITIALIZE_MECANUM_PLATFORM (0x30)
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

### INITIALIZE_OMNI_PLATFORM (0x31)
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

### INITIALIZE_DIFFERENTIAL_PLATFORM (0x32)
Description: This command initializes a differential (2-wheel) platform and prepares it for use. It uses motor and encoder index 0 for the left wheel and index 1 for the right wheel, which correspond to the is_reversed_0/1 and is_encoder_reversed_0/1 parameters. Motor indices 2 and 3 are not used by this platform and stay free for other purposes.\
Properties:
- is_reversed_0 (bool): Determines if motor 0 is reversed.
- is_reversed_1 (bool): Determines if motor 1 is reversed.
- is_encoder_reversed_0 (bool): Reverses encoder 0 counting direction, independently of motor 0's is_reversed_0. Set so the closed-loop feedback is negative (flip if the wheel runs away when given a target speed).
- is_encoder_reversed_1 (bool): Reverses encoder 1 counting direction, independently of motor 1's is_reversed_1. Set so the closed-loop feedback is negative (flip if the wheel runs away when given a target speed).
- wheel_diameter (double): Diameter of the robot wheels in meters.
- wheel_base (double): Distance between the two wheels in meters.
- encoder_resolution (double): Encoder resolution in ticks per revolution. The value can not be negative. If platform does not have encoders, the value should be set to zero.

### SET_PLATFORM_VELOCITY (0x40)
Description: This command sets the velocity for the platform in PWM.\
Properties:
- x (double): X component of platform velocity in PWM
  - Range: -100.0 to 100.0
- y (double): Y component of platform velocity in PWM
  - Range: -100.0 to 100.0
- t (double): Theta component of platform velocity in PWM
  - Range: -100.0 to 100.0

### START_PLATFORM_CONTROLLER (0x41)
Description: This command sets the controller for the platform.\
Properties:
- kp (double): Proportional constant of PID
- ki (double): Integral constant of PID
- kd (double): Derivative constant of PID
- integral_limit (double): Integral limit of PID controller. The value can not be negative or zero. If the value is zero or negative, the integral limit is disabled.

### SET_PLATFORM_TARGET_VELOCITY (0x42)
Description: This command set the target velocity for the platform in meters per second.\
Properties:
- x (double): X component of platform velocity in meters per second
- y (double): Y component of platform velocity in meters per second
- t (double): Theta component of platform velocity in radians per second

### GET_PLATFORM_CURRENT_VELOCITY (0x43)
Description: This command gets the current velocity of the platform in meters per second.\
Properties:
- None
Response: 
 - platform_velocity (object): The current velocity of the platform in meters per second.

### STOP_PLATFORM_CONTROLLER (0x44)
Description: This command stops the controller for the platform.\
Properties:
- None

### START_PLATFORM_ODOMETRY (0x45)
Description: This command starts the odometry calculation for the platform.\
Properties:
- None

### RESET_PLATFORM_ODOMETRY (0x46)
Description: This command resets the odometry calculation for the platform.\
Properties:
- None

### STOP_PLATFORM_ODOMETRY (0x47)
Description: This command stops the odometry calculation for the platform.\
Properties:
- None

### GET_PLATFORM_ODOMETRY (0x48)
Description: This command retrieves the odometry of the platform in meters and radians.\
Properties:
- None
Response: 
 - platform_odometry (object): The odometry of the platform in meters and radians.

### BRAKE_PLATFORM (0x49)
Description: This command actively brakes all of this platform's wheel motors (short brake) so they resist motion and hold position, and stops the platform velocity controller if it is running (you must call START_PLATFORM_CONTROLLER again to resume closed-loop platform control). Motors used outside this platform are not affected. The motors resist motion until a new command is issued.\
Properties:
- None

### COAST_PLATFORM (0x4A)
Description: This command lets all of this platform's wheel motors coast freely (high impedance) so they spin down without resistance, and stops the platform velocity controller if it is running (you must call START_PLATFORM_CONTROLLER again to resume closed-loop platform control). Motors used outside this platform are not affected. The motors spin down without resistance.\
Properties:
- None

