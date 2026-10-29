# Motor PWM frequency

`GET_PWM_FREQUENCY` (0x57) returns a uint32 frequency in hertz.
`SET_PWM_FREQUENCY` (0x58) accepts a uint32 hertz value and returns the applied value.
Neither request has a motor index. Both require INIT/READY and protocol 2.13.
They work before motor initialization and do not wake motor drivers.

`commands.json` defines `pwm_frequencies`, the generated enum catalog:

| Enum | Hertz |
| --- | ---: |
| KHZ_10 | 10000 |
| KHZ_20 | 20000 |
| KHZ_25 | 25000 |
| KHZ_50 | 50000 |
| KHZ_100 | 100000 |

Each board JSON defines `motors.pwm_frequency_hz` as its boot default and
`motors.supported_pwm_frequencies_hz` as its allowed values. The public profile
contains `pwm_frequency: {default_hz, supported_hz}` without timer or pin assignments.
SDKs expose `PwmFrequency` (Arduino: `KinisiPwmFrequency`) and profile helpers.

## Applying a frequency

Validate the enum, board support and every motor timer's clock divisor before
changing outputs. Unsupported values return `INVALID_ARGUMENT` without stopping
motors or changing the previous selection. An accepted SET cancels platform
position/velocity control, stops all individual motor controllers, and coasts all
motors. Even applying the existing value stops motion.

Each initialized timer is updated once, including timers shared by motors. A timer
update event loads the prescaler and zero-duty compares together. The period remains
840 ticks, preserving the speed percentage scale and complementary output polarity.
Uninitialized motors use the selected frequency when initialized later.

The selected frequency survives motor initialization, INIT, and disconnect/reconnect.
Reboot restores the board default (100 kHz in the supplied definitions). The motors
remain stopped after an update until explicitly commanded. Driver configuration,
current regulation, encoder timers, and PID calculation frequency are unchanged.
This setting controls motor PWM switching; `SET_CONTROLLER_FREQUENCY` independently
controls the software calculation rate.

## Validation

Host tests cover the supported rates across all boards and APB clock divisions,
shared timers, invalid input, initialized and uninitialized motors, preserved duty
resolution and INIT/READY gating. Measure the PWM output on physical hardware when
validating a board; a successful build does not establish its thermal performance.
