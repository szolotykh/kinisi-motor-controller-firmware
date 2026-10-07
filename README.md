Kinisi motor controller firmware - In development
============
The Kinisi motor controller firmware repository is a collection of software that controls the operations of Kinisi motor controllers. It includes the firmware source code, build scripts, and related documentation. The repository is designed to be a central hub for developers, hobbyists, and engineers who are looking to customize, extend, or debug their Kinisi motor controller systems. The firmware is written in C language and is optimized for performance and reliability. The repository is open-source and community-driven, allowing anyone to contribute their ideas and improvements to the codebase. Whether you're working on a custom robotics project or building a new consumer product, the Kinisi motor controller firmware repository is the perfect starting point for all your motor control needs.

Hardware project can be find [here](https://github.com/szolotykh/kinisi-motor-controller-board).

## Motor controller commands discription
There are number of commands that can be send to the motor controller to control motor speed and direction as well as to read encoder values. There are also commands to set PID controller for the motor to control its speed and position. Commands can be send to the motor controller via serial port.

Discription of commands can be find [here](commands.md). \
JavaScript code and client with UI interface to control motor controller can be find [here](https://github.com/szolotykh/jskinisi).\
Python code to control motor controller can be find [here](https://github.com/szolotykh/pykinisi).

API v2 is not compatible with API v1. Clients must support API v2 to use this firmware.

Firmware 2.3.1 corrects the shared velocity PID; see [velocity PID migration](docs/velocity-pid.md) before reusing gains.

Protocol 2.3 adds full motor and platform position PID control above the existing velocity
controllers. Motor angles use continuous radians; platform `(x,y,t)` uses meters,
meters and radians. See [position control setup and commands](docs/position-control.md).

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

## Board 0.4.0 basic support

Build with `pio run -e kinisi_v040` for the STM32F405VGT6 board.
The original `genericSTM32F405RG` environment continues to select MP_V3.
INIT reports board version 0.4.0 for the new environment.

MP_V4 supports four DRV8873S motors using SPI3 and TIM1 EN PWM,
PH direction, DISABLE coast/stop, and EN-low brake. Channels 1–3 use
complementary outputs with inverted polarity; motor 0 uses regular channel 4.
Startup holds every driver disabled and asleep. Motor initialization wakes its
driver, writes and reads back PH/EN mode, and starts zero-duty PWM while
remaining disabled. SPI failure leaves that motor uninitialized. A speed
command with an asserted nFAULT coasts the motor. Current regulation retains
the driver's factory default (6.5 A nominal); this is not a board current rating.
Zero speed coasts; explicit brake uses the PH/EN high-side brake state.

The four encoder mappings, eight external GPIOs, and PB14 status LED follow
the new board. Sensor/FRAM support, current ADC measurements, asynchronous
fault reporting, and UART support are outside this initial motor-control scope.
Fault inputs are checked on speed commands, not monitored by a background task.

Before powered use, verify SPI readback, physical EN duty/polarity at zero and
100%, motor direction/reversal, coast/brake, encoder counts, and fault behavior
on all four channels. Compilation and host tests do not verify these electrical
behaviors. Driver configuration follows the
[TI DRV8873 datasheet](https://www.ti.com/lit/ds/symlink/drv8873.pdf).
