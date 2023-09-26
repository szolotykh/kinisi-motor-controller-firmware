# Kinisi motor controller commands

Version: 1.0.1
---

## Commands
### INITIALIZE_MOTOR (0x01)
Description: This command initializes a motor and prepares it for use.
Properties:
- motor_index (uint8_t): The index of the motor to initialize.
  - Range: 0 to 3

### SET_MOTOR_SPEED (0x02)
Description: This command sets the speed of the specified motor.
Properties:
- motor_index (uint8_t): The index of the motor to set the speed for.
  - Range: 0 to 3
- direction (uint8_t): The direction of the motor.
  - Range: 0 to 1
- speed (uint16_t): The speed of the motor.
  - Range: 0 to 840

### INITIALIZE_MOTOR_CONTROLLER (0x03)
Description: This command sets the controller for the specified motor.
Properties:
- motor_index (uint8_t): The index of the motor to set the controller for.
  - Range: 0 to 3
- kp (double): Proportional constant of PID
- ki (double): Integral constant of PID
- kd (double): Derivative constant of PID

### SET_MOTOR_TARGET_VELOCITY (0x04)
Description: This command sets the target velocity for the specified motor.
Properties:
- motor_index (uint8_t): The index of the motor to set the target velocity for.
  - Range: 0 to 3
- direction (uint8_t): The direction of the motor.
  - Range: 0 to 1
- speed (uint16_t): The speed of the motor.
  - Range: 0 to 840

### DELETE_MOTOR_CONTROLLER (0x05)
Description: This command deletes the controller for the specified motor.
Properties:
- motor_index (uint8_t): The index of the motor to delete the controller for.
  - Range: 0 to 3

### INITIALIZE_ENCODER (0x11)
Description: This command initializes an encoder and prepares it for use.
Properties:
- encoder_index (uint8_t): The index of the encoder to initialize.
  - Range: 0 to 3

### GET_ENCODER_VALUE (0x12)
Description: This command retrieves the current value of the encoder.
Properties:
- encoder_index (uint8_t): The index of the encoder to retrieve the value for.
  - Range: 0 to 3
Response: 
 - encoderValue (uint32_t): The current value of the encoder.

### INITIALIZE_GPIO_PIN (0x20)
Description: This command sets the specified pin to a low state.
Properties:
- pin_number (uint8_t): The number of the pin to initialize.
- mode (uint8_t): Set digital pin as input or output. Awailable modes: 0 = INPUT, 0 = INPUT_PULLDOWN, 1 = INPUT_PULLUP, 2 = INPUT_NOPULL, 3 = OUTPUT.

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

### INITIALIZE_PLATFORM (0x30)
Description: This command initializes the platform and prepares it for use.
Properties:
- None

### SET_PLATFORM_VELOCITY_INPUT (0x31)
Description: This command sets the velocity input for the platform.
Properties:
- x (int8_t): X component of platform velocity input.
- y (int8_t): Y component of platform velocity input.
- t (int8_t): Theta component of platform velocity input.

### SET_PLATFORM_CONTROLLER (0x32)
Description: This command sets the controller for the platform.
Properties:
- None

