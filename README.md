Kinisi motor controller firmware - In development
============
The Kinisi motor controller firmware repository is a collection of software that controls the operations of Kinisi motor controllers. It includes the firmware source code, build scripts, and related documentation. The repository is designed to be a central hub for developers, hobbyists, and engineers who are looking to customize, extend, or debug their Kinisi motor controller systems. The firmware is written in C language and is optimized for performance and reliability. The repository is open-source and community-driven, allowing anyone to contribute their ideas and improvements to the codebase. Whether you're working on a custom robotics project or building a new consumer product, the Kinisi motor controller firmware repository is the perfect starting point for all your motor control needs.

Hardware project can be find [here](https://github.com/szolotykh/kinisi-motor-controller-board).

## Motor controller commands discription

- **INITIALIZE_MOTOR 0x01:**\
Description: This command initializes a motor and prepares it for use.\
Properties:
    - motorIndex: Index of the motor (0 - 4)
    
- **SET_MOTOR_SPEED 0x02:**\
Description: This command sets the speed of the specified motor.\
Properties:
    - motorIndex (uint8_t): Index of the motor (0 - 4)
    - direction (uint16_t): Direction of the motor (0 or 1)
    - speed(uint8_t): Speed of the motor in PWM (0 - 840)

- **MOTOR_SET_CONTROLLER 0x03:**\
Description: This command sets the controller for the specified motor.\
Properties:
    - motorIndex (uint8_t): Index of the motor (0 - 4)
    - kp (double): Proportional constant of PID
    - ki (double): Integral constant of PID
    - kd (double): Derivative constant of PID

- **MOTOR_SET_TARGET_VELOCITY 0x04:**
Description: This command sets the target velocity for the specified motor
Properties:
    - motorIndex (uint8_t): Index of the motor (0 - 4)

- **MOTOR_DEL_CONTROLLER 0x05:**
Description: This command deletes the controller for the specified motor.\
Properties:
    - motorIndex (uint8_t): Index of a motor (0 - 4)

- **INITIALIZE_ENCODER 0x11:**\
Description: This command initializes an encoder and prepares it for use.\
Properties:
    - encoderIndex (uint8_t): Index of a encoder (0 - 4)

- **GET_ENCODER_VALUE 0x12:**\
Description: This command retrieves the current value of the encoder.\
Properties:
    - encoderIndex (uint8_t): Index of a encoder (0 - 4)

Response: Encoder value (uint32_t)

- **PIN_LOW 0x20:**\
Description: This command sets the specified pin to a low state.\
Properties:
    - Pin number: An integer value used to identify which pin needs to be set low.

- **PIN_HIGH 0x21:**\
Description: This command sets the specified pin to a high state.
Properties:
    - Pin number: An integer value used to identify which pin needs to be set high.

- **PIN_TOGGLE 0x22:**\
Description: This command toggles the specified pin.\
Properties:
    - Pin number: An integer value used to identify which pin needs to be toggled.

- **STATUS_LED_OFF 0x23:**\
Description: This command turns off the status LED.\
Properties: None

- **STATUS_LED_ON 0x24:**\
Description: This command turns on the status LED.\
Properties: None

- **STATUS_LED_TOGGLE 0x25:**\
Description: This command toggles the status LED.\
Properties: None

- **PLATFORM_INITIALIZE 0x30:**\
Description: This command initializes the platform and prepares it for use.\
Properties: None

- **PLATFORM_SET_VELOCITY_INPUT 0x31:**\
Description: This command sets the velocity input for the platform.
Properties:
    - x (int8_t): X component of platform velocity
    - y (int8_t): Y component of platform velocity
    - t (int8_t): T component of platform velocity
    
- **PLATFORM_SET_CONTROLLER 0x32:**\
Description: This command sets the controller for the platform.\
Properties: None