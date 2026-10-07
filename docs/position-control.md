# Position control (protocol 2.3.0)

Position control is a separate PID outer loop feeding the existing velocity PID.
Tune and verify the velocity loop first, then initialize position control.
Firmware 2.3.1 requires velocity gain retuning; see [velocity PID migration](velocity-pid.md). The
outer loop has positive Kp and speed limits, plus nonnegative Ki, Kd, integral
contribution limits and position tolerances. It runs at the shared controller frequency (`SET_CONTROLLER_FREQUENCY`,
10 Hz by default). It does not change the velocity PID gains.

## Units and frames

- Motor position: continuous radians in the configured encoder's direction. One
  turn is `2*pi`; `4*pi` requests two turns from the origin, without angle wrapping.
  Encoder counts per revolution must describe the shaft whose angle is commanded,
  including gearing. Initialization and reset establish zero at the current count.
- Platform position: `(x, y, t)` in the existing odometry world frame, in meters,
  meters and radians. `+x` is forward and `+y` left at heading zero; positive heading
  is counterclockwise. Heading error is wrapped to the shortest path.
- Platform velocity limits: translation magnitude in m/s and rotation in rad/s.
  Limits apply to requested velocity, not measured overshoot or acceleration.

## Motor commands

| Command | ID | Payload in order |
| --- | --- | --- |
| `INITIALIZE_MOTOR_POSITION_CONTROLLER` | `0x0C` | motor index (`uint8`), kp, max_speed, tolerance (`double` each) |
| `INITIALIZE_MOTOR_POSITION_PID_CONTROLLER` | `0x10` | motor_index (uint8), kp, max_speed, tolerance, ki, kd, integral_limit (double) |
| `RESET_MOTOR_POSITION` | `0x0D` | motor index |
| `SET_MOTOR_POSITION` | `0x0E` | motor index, position (`double`) |
| `GET_MOTOR_POSITION` | `0x0F` | motor index; response is one `double` in radians |

Sequence (pseudocode; these are wire commands, not new SDK methods):

```text
INITIALIZE_MOTOR_CONTROLLER(motor_index=0, encoder_index=0, ...tuned velocity settings...)
INITIALIZE_MOTOR_POSITION_PID_CONTROLLER(motor_index=0, kp=2, max_speed=1, tolerance=0.02, ki=0, kd=0, integral_limit=1)
SET_MOTOR_POSITION(motor_index=0, position=6.283185307179586)
GET_MOTOR_POSITION(motor_index=0)
RESET_MOTOR_POSITION(motor_index=0)
```

The example position tuning is illustrative and needs hardware validation.
Initialization holds the new zero position. Reset replaces the current position
and target with zero, clears velocity PID history and output, and holds zero.
Neither operation resets independent encoder odometry. The getter returns the
latest controller sample, not an immediate hardware sample.

`SET_MOTOR_TARGET_SPEED` and `RESET_MOTOR_CONTROLLER` suspend position mode while
retaining its origin and tuning. A new `SET_MOTOR_POSITION` resumes it. Stop,
brake, delete, raw PWM, motor reinitialization, or velocity-controller
reinitialization discard position tuning; initialize it again before reuse.
Direct position commands are rejected for platform-owned motors.

## Platform commands

| Command | ID | Payload in order (all `double`) |
| --- | --- | --- |
| `INITIALIZE_PLATFORM_POSITION_CONTROLLER` | `0x4B` | linear_kp, angular_kp, max_linear_speed, max_angular_speed, position_tolerance, heading_tolerance |
| `INITIALIZE_PLATFORM_POSITION_PID_CONTROLLER` | `0x4E` | linear_kp, angular_kp, max_linear_speed, max_angular_speed, position_tolerance, heading_tolerance, linear_ki, linear_kd, linear_integral_limit, angular_ki, angular_kd, angular_integral_limit |
| `RESET_PLATFORM_POSITION` | `0x4C` | none |
| `SET_PLATFORM_POSITION` | `0x4D` | x, y, t |

```text
INITIALIZE_OMNI_PLATFORM(...geometry, encoder resolution and direction flags...)
START_PLATFORM_CONTROLLER(...tuned velocity PID settings...)
INITIALIZE_PLATFORM_POSITION_PID_CONTROLLER(1, 2, 0.2, 0.5, 0.01, 0.03, 0, 0, 0.2, 0, 0, 0.5)
wait for a fresh platform odometry sample
SET_PLATFORM_POSITION(x=1, y=0, t=1.5707963267948966)
GET_PLATFORM_ODOMETRY()
```

Complete the existing `INIT`/`READY` handshake before timestamped odometry reads.
Position initialization requires all wheel velocity controllers and encoders, and
valid positive platform geometry. It starts odometry if necessary, preserves an
existing odometry frame, and zeros velocity targets. It remains idle until a pose
target is accepted. Read current pose through `GET_PLATFORM_ODOMETRY`.

Omni and mecanum bases translate and rotate together. Differential bases steer
toward the point, move forward as the bearing aligns, then align to the final
heading within translation tolerance. This is local pose control from wheel
odometry: it provides no obstacle avoidance, global localization, trajectory
planning or acceleration limiting. Wheel slip will affect accuracy.

`RESET_PLATFORM_POSITION` cancels the old target, zeros velocity targets and
resets platform odometry to `(0,0,0)`, retaining position tuning. Wait for a fresh
sample before setting another target. Existing `RESET_PLATFORM_ODOMETRY`, stopping
or restarting odometry, wheel feedback reconfiguration, platform reconfiguration,
velocity overrides, stop, coast and brake cancel platform position control.
Initialize it again before another pose command. Connection-loss/watchdog stop
also cancels position control through the existing stop path.

Missing/nonfinite feedback or a sample older than three configured odometry
periods zeros platform velocity targets and cancels the active target. Fresh
feedback alone cannot restart motion; send a new target. Counter tracking requires
less than half a 16-bit encoder range of movement per controller sample.

## Protocol and verification

Payloads use existing little-endian packed fields and IEEE-754 64-bit doubles.
IDs are additive; older 2.x clients remain compatible. New commands require a
firmware reporting protocol 2.2 or later; PID initialization requires 2.3 or later. Successful setters receive the usual
empty ACK. Missing velocity or position initialization returns
`CONTROLLER_NOT_INITIALIZED`. See [the generated command reference](../commands.md)
for complete errors, lengths and validation rules.

Host checks:

```text
python test/test_position/run_tests.py
python test/test_time_sync/run_tests.py
python test/test_initialization/run_tests.py
```

The tests exercise real controller tasks/dispatch with mocked hardware: motor
encoder mapping and rollover, multi-turn targets, reset, limits, overrides,
platform frames, all three platform kinematics, stale feedback, stop lifecycle,
wire framing and prerequisite errors. Firmware builds and host tests do not
replace powered tuning or motion validation. No hardware was flashed for this change.

## PID state and compatibility

Protocol 2.3 adds the PID initialization commands above; the original 2.2
initialization commands keep their wire format and set Ki, Kd and integral limits
to zero. Kp is in 1/s, Ki in 1/s^2, and Kd is dimensionless. Integral limits bound
the integral contribution in rad/s (motor/heading) or m/s (translation).
Zero Ki or Kd disables that term. The integral uses conditional anti-windup at
speed saturation. Derivatives are low-pass filtered with a 20 ms time constant.
The first update after a changed target has no derivative kick. Gains are
independent of the inner velocity PID.

Motor PID uses the configured controller period. Platform PID uses elapsed
monotonic time. New targets, initialization, resets and resuming after an override
clear position PID state; repeating the same active target preserves it. Entering
tolerance clears history and requests zero speed. Stale feedback cancels platform
motion and clears PID state. Invalid timing cannot integrate or differentiate.

Omni/mecanum translation uses independent world-frame X/Y histories with shared
gains and a vector speed limit; heading uses its own history and wrapped angular
derivatives. Differential translation uses distance, steering uses bearing until
arrival, and switching to final heading clears heading history.
