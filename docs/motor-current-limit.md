# Motor current-limit configuration

`GET_MOTOR_CURRENT_LIMIT(motor_index)` reads the applied driver setting.
`SET_MOTOR_CURRENT_LIMIT(motor_index, enabled, threshold_ma)` applies a supported
nominal threshold in integer milliamps. Both return `{enabled, threshold_ma}`.
The commands require protocol 2.11, a ready connection, and an initialized,
awake motor driver. A GET does not stop, wake, or reconfigure the motor.

Board JSON contains each motor's allowed values, disable capability, and defaults.
The generator emits firmware configuration plus public `current_limit_options`:

```json
{
  "type": "discrete",
  "supported_thresholds_ma": [4000, 5400, 6500, 7000],
  "can_disable": true
}
```

The DRV8873S supports those four nominal thresholds. They have hardware tolerance;
they are regulation trip levels, not motor-current ratings or guaranteed exact
ceilings. See the [TI datasheet, Tables 8 and 27](https://www.ti.com/lit/ds/symlink/drv8873.pdf).
Firmware maps milliamps to IC4's ITRIP_LVL bits. Enabling/disabling applies to both
outputs. It preserves open-load configuration and leaves IC2 overcurrent protection
unchanged. No register enum is sent over the wire.

## Applying a setting

SET validates the requested value before changing motor state. It rejects
platform-owned motors with `MOTOR_OWNED`; release platform ownership first.
An accepted SET stops the motor and its controller, clears PWM, writes the register,
and verifies readback. The motor remains stopped after either success or an I/O
failure. Restart requires an explicit motion command. There is no automatic retry.

Settings are volatile. Motor initialization applies the enabled state and threshold
from board JSON; initialization fails if they cannot be verified. Driver power loss
resets its registers and requires motor reinitialization. Fault clearing preserves
current-limit settings. GET always reads the driver rather than a cached requested value.

Both enabled and disabled requests must specify an allowed threshold. Unsupported
values (such as 5000 on the current board) return `INVALID_ARGUMENT`; firmware
never rounds to a different threshold. Missing hardware returns
`SENSOR_NOT_AVAILABLE` after the motor prerequisite. Uninitialized motors return
`MOTOR_NOT_INITIALIZED`. A sleeping driver, unavailable motor power, malformed SPI
response, asymmetric output regulation, or failed verification returns `INTERNAL_ERROR`.
Before INIT/READY, session errors take precedence. SET checks motor ownership
before motor initialization, consistently with other direct motor commands.

The GET request is 5 bytes, SET request is 8 bytes, and either response is 7 bytes,
including framing. All fit the default Arduino Wire buffer.

## Client access

SDKs bundle the public profiles and select one by exact board identity. JavaScript
and Arduino expose `getMotorCurrentLimitOptions(index)` and
`supportsMotorCurrentLimit(index)`; Python uses the corresponding snake_case names.
Options describe hardware independently of protocol support. Support checks combine
hardware and firmware availability. Unknown boards return no options; explicit
commands on compatible unknown boards defer validation to firmware.

The JavaScript Motor tab reads the active setting and builds a dropdown from these
values. It does not initialize or restart a motor automatically. SDKs validate
known-profile settings before I/O, and firmware independently validates every request.
