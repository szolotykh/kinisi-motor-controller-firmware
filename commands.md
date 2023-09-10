# Kinisi motor controller commands

Version: 1.0.0
---
## Types
### MotorIndex
- Motor0
- Motor1
- Motor2
- Motor3

### EncoderIndex
- Encoder0
- Encoder1
- Encoder2
- Encoder3


## Commands
### INITIALIZE_MOTOR (0x01)
Description: This command initializes a motor and prepares it for use.
Properties:
- motorIndex (MotorIndex): The index of the motor to initialize.

### SET_MOTOR_SPEED (0x02)
Description: This command sets the speed of the specified motor.
Properties:
- motorIndex (MotorIndex): The index of the motor to set the speed for.
- direction (uint16_t): The direction of the motor.
  - Range: 0 to 1
- speed (uint8_t): The speed of the motor.
  - Range: 0 to 840

### SET_MOTOR_CONTROLLER (0x03)
Description: This command sets the controller for the specified motor.
Properties:
- motorIndex (MotorIndex): The index of the motor to set the controller for.
- kp (double): Proportional constant of PID
- ki (double): Integral constant of PID
- kd (double): Derivative constant of PID

### SET_MOTOR_TARGET_VELOCITY (0x04)
Description: This command sets the target velocity for the specified motor.
Properties:
- motorIndex (MotorIndex): The index of the motor to set the target velocity for.
- direction (uint16_t): The direction of the motor.
  - Range: 0 to 1
- speed (uint8_t): The speed of the motor.
  - Range: 0 to 840

### DELETE_MOTOR_CONTROLLER (0x05)
Description: This command deletes the controller for the specified motor.
Properties:
- motorIndex (MotorIndex): The index of the motor to delete the controller for.

### INITIALIZE_ENCODER (0x11)
Description: This command initializes an encoder and prepares it for use.
Properties:
- encoderIndex (EncoderIndex): The index of the encoder to initialize.

### GET_ENCODER_VALUE (0x12)
Description: This command retrieves the current value of the encoder.
Properties:
- encoderIndex (EncoderIndex): The index of the encoder to retrieve the value for.
Response: 
 - encoderValue (uint32_t): The current value of the encoder.

### SET_PIN_LOW (0x20)
Description: This command sets the specified pin to a low state.
Properties:
- pinNumber (uint8_t): The number of the pin to set to a low state.

### SET_PIN_HIGH (0x21)
Description: This command sets the specified pin to a high state.
Properties:
- pinNumber (uint8_t): The number of the pin to set to a high state.

### TOGGLE_PIN (0x22)
Description: This command toggles the specified pin.
Properties:
- pinNumber (uint8_t): The number of the pin to toggle.

### SET_STATUS_LED_OFF (0x23)
Description: This command turns off the status LED.
Properties:
- None

### SET_STATUS_LED_ON (0x24)
Description: This command turns on the status LED.
Properties:
- None

### TOGGLE_STATUS_LED (0x25)
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

