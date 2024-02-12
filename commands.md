# Kinisi motor controller commands

Version: 1.0.3
---

## Commands
### INITIALIZE_MOTOR (0x01)
Description: This command initializes a motor and prepares it for use.
Properties:
- motor_index (uint8_t): The index of the motor to initialize.
  - Range: 0 to 3
- is_reversed (bool): Whether or not the motor is reversed.

### SET_MOTOR_SPEED (0x02)
Description: This command sets the speed of the specified motor in PWM.
Properties:
- motor_index (uint8_t): The index of the motor to set the speed for.
  - Range: 0 to 3
- pwm (double): The speed of the motor.
  - Range: -100.0 to 100.0

### STOP_MOTOR (0x03)
Description: This command stops motor by setting its speed to 0.
Properties:
- motor_index (uint8_t): The index of the motor to set the speed for.
  - Range: 0 to 3

### BRAKE_MOTOR (0x04)
Description: This command brakes motor.
Properties:
- motor_index (uint8_t): The index of the motor to set the speed for.
  - Range: 0 to 3

### INITIALIZE_MOTOR_CONTROLLER (0x05)
Description: This command sets the controller for the specified motor.
Properties:
- motor_index (uint8_t): The index of the motor to set the controller for.
  - Range: 0 to 3
- is_reversed (bool): Whether or not the motor is reversed.
- encoder_index (uint8_t): The index of the encoder to use for the controller.
  - Range: 0 to 3
- encoder_resolution (double): Encoder resolution in ticks per revolution. The value can not be negative or zero.
- kp (double): Proportional constant of PID
- ki (double): Integral constant of PID
- kd (double): Derivative constant of PID
- integral_limit (double): Integral limit of PID controller. The value can not be negative or zero. If the value is zero or negative, the integral limit is disabled.

### SET_MOTOR_TARGET_SPEED (0x06)
Description: This command sets the target speed for the specified motor in radians.
Properties:
- motor_index (uint8_t): The index of the motor to set the target velocity for.
  - Range: 0 to 3
- speed (double): The speed of the motor.

### RESET_MOTOR_CONTROLLER (0x07)
Description: This command resets the controller for the specified motor.
Properties:
- motor_index (uint8_t): The index of the motor to reset the controller for.
  - Range: 0 to 3

### GET_MOTOR_CONTROLLER_STATE (0x08)
Description: This command gets the state of the controller for the specified motor.
Properties:
- motor_index (uint8_t): The index of the motor to get the state for.
  - Range: 0 to 3
Response: 
 - motor_controller_state (object): The state of the controller for the specified motor.

### DELETE_MOTOR_CONTROLLER (0x09)
Description: This command deletes the controller for the specified motor.
Properties:
- motor_index (uint8_t): The index of the motor to delete the controller for.
  - Range: 0 to 3

### INITIALIZE_ENCODER (0x11)
Description: This command initializes an encoder and prepares it for use.
Properties:
- encoder_index (uint8_t): The index of the encoder to initialize.
  - Range: 0 to 3
- encoder_resolution (double): Encoder resolution in ticks per revolution. The value can not be negative or zero.
- is_reversed (bool): Whether or not the encoder is reversed.

### GET_ENCODER_VALUE (0x12)
Description: This command retrieves the current value of the encoder.
Properties:
- encoder_index (uint8_t): The index of the encoder to retrieve the value for.
  - Range: 0 to 3
Response: 
 - encoderValue (uint16_t): The current value of the encoder.

### INITIALIZE_GPIO_PIN (0x20)
Description: This command initializes a digital pin and prepares it for use.
Properties:
- pin_number (uint8_t): The number of the pin to initialize.
- mode (uint8_t): Set digital pin as input or output. Modes: 0 = INPUT_PULLDOWN, 1 = INPUT_PULLUP, 2 = INPUT_NOPULL, 3 = OUTPUT.

### SET_GPIO_PIN_STATE (0x21)
Description: This command sets the specified pin to a state.
Properties:
- pin_number (uint8_t): The number of the pin to set to a state.
- state (uint8_t): The state of the pin. 0 = LOW, 1 = HIGH.

### GET_GPIO_PIN_STATE (0x22)
Description: This command gets the state of the specified pin.
Properties:
- pin_number (uint8_t): The number of the pin to get the state for.
Response: 
 - state (uint8_t): The state of the pin. 0 = LOW, 1 = HIGH.

### TOGGLE_GPIO_PIN_STATE (0x23)
Description: This command toggles the specified pin.
Properties:
- pin_number (uint8_t): The number of the pin to toggle.

### SET_STATUS_LED_STATE (0x25)
Description: This command sets the status LED to a state.
Properties:
- state (uint8_t): The state of the status LED. 0 = OFF, 1 = ON.

### TOGGLE_STATUS_LED_STATE (0x26)
Description: This command toggles the status LED.
Properties:
- None

### INITIALIZE_MECANUM_PLATFORM (0x30)
Description: This command initializes a mecanum platform and prepares it for use.
Properties:
- is_reversed_0 (bool): Determins if motor 0 is reversed.
- is_reversed_1 (bool): Determins if motor 1 is reversed.
- is_reversed_2 (bool): Determins if motor 2 is reversed.
- is_reversed_3 (bool): Determins if motor 3 is reversed.
- length (double): Length of the platform in meters.
- width (double): Width of the platform in meters.
- wheels_diameter (double): Diameter of the robot wheels in meters.

### INITIALIZE_OMNI_PLATFORM (0x31)
Description: This command initializes a omni platform and prepares it for use.
Properties:
- is_reversed_0 (bool): Determins if motor 0 is reversed.
- is_reversed_1 (bool): Determins if motor 1 is reversed.
- is_reversed_2 (bool): Determins if motor 2 is reversed.
- wheels_diameter (double): Diameter of the robot wheels in millimeters.
- robot_radius (double): Distance berween the center of the robot and the center of the wheels in millimeters.

### SET_PLATFORM_VELOCITY (0x40)
Description: This command sets the velocity for the platform in PWM.
Properties:
- x (double): X component of platform velocity in PWM
  - Range: -100.0 to 100.0
- y (double): Y component of platform velocity in PWM
  - Range: -100.0 to 100.0
- t (double): Theta component of platform velocity in PWM
  - Range: -100.0 to 100.0

### SET_PLATFORM_CONTROLLER (0x41)
Description: This command sets the controller for the platform.
Properties:
- kp (double): Proportional constant of PID
- ki (double): Integral constant of PID
- kd (double): Derivative constant of PID
- encoder_resolution (double): Encoder resolution in ticks per revolution. The value can not be negative or zero.
- integral_limit (double): Integral limit of PID controller. The value can not be negative or zero. If the value is zero or negative, the integral limit is disabled.

### SET_PLATFORM_TARGET_VELOCITY (0x42)
Description: This command set the target velocity for the platform in meters per second.
Properties:
- x (double): X component of platform velocity in meters per second
- y (double): Y component of platform velocity in meters per second
- t (double): Theta component of platform velocity in radians per second

### GET_PLATFORM_CURRENT_VELOCITY (0x43)
Description: This command gets the current velocity of the platform in meters per second.
Properties:
- None
Response: 
 - platform_velocity (object): The current velocity of the platform in meters per second.

