Kinisi motor controller firmware - In development
============
The Kinisi motor controller firmware repository is a collection of software that controls the operations of Kinisi motor controllers. It includes the firmware source code, build scripts, and related documentation. The repository is designed to be a central hub for developers, hobbyists, and engineers who are looking to customize, extend, or debug their Kinisi motor controller systems. The firmware is written in C language and is optimized for performance and reliability. The repository is open-source and community-driven, allowing anyone to contribute their ideas and improvements to the codebase. Whether you're working on a custom robotics project or building a new consumer product, the Kinisi motor controller firmware repository is the perfect starting point for all your motor control needs.

Hardware project can be find [here](https://github.com/szolotykh/kinisi-motor-controller-board).

## Motor controller commands discription
There are number of commands that can be send to the motor controller to control motor speed and direction as well as to read encoder values. There are also commands to set PID controller for the motor to control its speed and position. Commands can be send to the motor controller via serial port.

Discription of commands can be find [here](commands.md). \
JavaScript code and client with UI interface to control motor controller can be find [here](https://github.com/szolotykh/jskinisi).\
Python code to control motor controller can be find [here](https://github.com/szolotykh/pykinisi).

## Motor PWM configuration

`lib/hardware/motor_pwm_config.h` defines the common 100 kHz PWM frequency
and 0–840 compare-value range for all motors. Percentage commands are still
converted to that range; direction is handled separately. The timer counts
0–839, with compare value 840 representing a continuously high output.

`hw_timer_input_clock_hz()` in `lib/hardware/hw_timers.c` reads the active
STM32F405 RCC clocks and accounts for the APB timer clock multiplier. Motor
initialization derives the prescaler automatically:

| Timer input clock | Divider | PSC register | ARR register | PWM frequency |
| --- | --- | --- | --- | --- |
| 84 MHz | 1 | 0 | 839 | 100 kHz |
| 168 MHz | 2 | 1 | 839 | 100 kHz |

For a board revision that changes motor timers, update the timer/channel/pin
assignments in `include/hw_config.h`. Supported timers on either APB bus use
the same PWM configuration automatically. Timer handles, alternate functions,
clock enables, available channels, and conflicts with encoder/system timers
must still be checked for the new board. A different MCU family requires
reviewing the clock helper's STM32F405-specific rules.

An unknown timer or a clock without an exact valid prescaler invokes the
existing fatal initialization error handler rather than silently reducing
resolution or changing frequency. Clock configuration must remain stable
after motor initialization. Check frequency, duty cycle, direction, stop,
and brake outputs on hardware before normal operation.

## Generating Commands from commands.json

To generate commands from `commands.json`, follow these steps:

1. **Ensure you have Python installed**: The script requires Python 3.x to run.

2. **Navigate to the `tools` directory**: Open a terminal and navigate to the `tools` directory in your project.

3. **Run the `update-commands.py` script**: Execute the following command in the terminal:
    ```sh
    cd tools
    python update-commands.py
    ```

This script will:
- Generate the command file `commands.h` from `commands.json` using the `generator.py` script.
- Generate the documentation file `commands.md` from `commands.json` using the `generator.py` script.

## Building the Project

To build the project for the `genericSTM32F405RG` configuration, follow these steps:

1. **Ensure you have PlatformIO installed**: The build process requires PlatformIO.

2. **Navigate to the project directory**: Open a terminal and navigate to the root directory of your project.

3. **Run the build command**: Execute the following command in the terminal:
    ```sh
    pio run -e genericSTM32F405RG
    ```

This command will build the project for the `genericSTM32F405RG` configuration.

## Running Tests

To run all the tests for the project, follow these steps:

1. **Ensure you have PlatformIO installed**: The testing process requires PlatformIO.

2. **Navigate to the project directory**: Open a terminal and navigate to the root directory of your project.

3. **Run the test command**: Execute the following command in the terminal:
    ```sh
    pio test -e test_native -e test_platforms
    ```

This command will run:
- Unit tests for utilities, commands, and controller components under the `test_native` environment
- Platform-specific tests for omni, mecanum, and differential platforms under the `test_platforms` environment

You can also run specific test suites by using the `-f` flag:
```sh
pio test -e test_native -f test_commands
```

## Links
- [Kinisi Motion Controller firmware](https://github.com/szolotykh/kinisi-motor-controller-firmware)
- [Kinisi Motion Controller hardware](https://github.com/szolotykh/kinisi-motor-controller-board)
- [JavaScipt package for kinisi motor controller](https://github.com/szolotykh/jskinisi)
- [Python package for kinisi motor controller](https://github.com/szolotykh/pykinisi)
