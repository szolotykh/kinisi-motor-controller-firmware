# Motor PWM configuration

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
