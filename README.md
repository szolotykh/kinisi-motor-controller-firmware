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

## Communication protocol versions

The communication protocol uses `major.minor.patch`. A major release introduces
breaking changes, a minor release adds features while preserving existing
commands, and a patch release provides compatible fixes. The client and firmware
must support the same protocol major; the firmware must also meet the client's
minimum required minor. Optional commands are checked against the firmware's
reported protocol version. Patch differences do not block connection.

Board revision, client library release and firmware build identity are separate
from the communication protocol version. See the
[protocol versioning policy](docs/initialization.md#protocol-versioning-and-compatibility)
for command evolution rules and compatibility examples.

Board definitions also generate a compact [client profile bundle](docs/board-profiles.md)
for resource counts and hardware features. Clients select a profile using the
board identity reported during initialization and check protocol support separately.

See [position control](docs/position-control.md) and [velocity PID tuning](docs/velocity-pid.md) for setup details.

See [motor current sensing](docs/current-sensing.md) for measurements and
[motor fault reporting](docs/motor-faults.md) for driver diagnostics.

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

For board v0.4 (STM32F405VGT6), use `pio run -e v0_4`. The original
`genericSTM32F405RG` environment selects v0.3. See
[board v0.4 support](docs/board-v0.4.md) for motor configuration and validation.

## Board configuration

Hardware definitions for v0.1 through v0.4 live in [boards](boards/README.md).
Edit the board JSON and generate the C configuration with:

```sh
python tools/board_config_generator/generator.py
```

This produces `include/hw_config.h` (C structures and compile-time selectors)
and `include/board_revision.h` (board identity). Both files are checked in;
PlatformIO regenerates them before firmware builds. Use `--check` to detect
stale output without changing files. JSON is never parsed on the MCU.

Explicit build targets are `v0_1`, `v0_2`, `v0_3`, and `v0_4`:

```sh
pio run -e v0_1 -e v0_2 -e v0_3 -e v0_4
```

The original `genericSTM32F405RG` target remains v0.3. Builds reject conflicting
board selections and an MCU that does not match the selected JSON definition.

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
