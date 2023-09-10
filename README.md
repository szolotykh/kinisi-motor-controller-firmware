Kinisi motor controller firmware - In development
============
The Kinisi motor controller firmware repository is a collection of software that controls the operations of Kinisi motor controllers. It includes the firmware source code, build scripts, and related documentation. The repository is designed to be a central hub for developers, hobbyists, and engineers who are looking to customize, extend, or debug their Kinisi motor controller systems. The firmware is written in C language and is optimized for performance and reliability. The repository is open-source and community-driven, allowing anyone to contribute their ideas and improvements to the codebase. Whether you're working on a custom robotics project or building a new consumer product, the Kinisi motor controller firmware repository is the perfect starting point for all your motor control needs.

Hardware project can be find [here](https://github.com/szolotykh/kinisi-motor-controller-board).

## Motor controller commands discription
There are number of commands that can be send to the motor controller to control motor speed and direction as well as to read encoder values. There are also commands to set PID controller for the motor to control its speed and position. Commands can be send to the motor controller via serial port.

Discription of commands can be find [here](commands.md). \
JavaScript code and client with UI interface to control motor controller can be find [here](https://github.com/szolotykh/jskinisi).\
Python code to control motor controller can be find [here](https://github.com/szolotykh/pykinisi).