# Velocity PID correction in firmware 2.3.1

This applies to individual motors and all platform wheel velocity controllers.
Position controllers remain separate outer loops that request velocity.

## Output and units

The controller now calculates `PWM = clamp(P + I + D, -100, 100)` directly.
Earlier firmware added the complete PID result to the previous PWM on every
cycle, introducing unintended extra accumulation. Existing gains are not
equivalent under the corrected equation and must be retuned after upgrading.

Velocity error is measured in rad/s regardless of the UI's display units:

| Setting | Units |
| --- | --- |
| Kp | PWM percentage points / (rad/s) |
| Ki | PWM percentage points / rad |
| Kd | PWM percentage points / (rad/s²) |
| Integral contribution limit | PWM percentage points, 0–100 |

An integral limit of 30 bounds the integral term to ±30 percentage points;
it does not limit speed or the total output to 30%. Zero now disables integral
action. Previously zero (and negative values) disabled the configured integral
bound. Negative or nonfinite gains and integral limits outside 0–100 are rejected.
Ki=0 also disables integral action; Kd=0 disables derivative action.

## Integration, differentiation and reset

The integral uses elapsed controller-period time and trapezoidal integration
(the first update uses the current error as its baseline). Conditional
anti-windup rejects integral changes that push output farther beyond ±100%,
while allowing changes that unwind saturation.

The derivative acts on measured velocity to avoid target-step kicks. A stable
backward-Euler low-pass filter uses a 10 ms time constant. The previous filter
had the wrong sign on its recursive term. Initialization and reset establish a
new derivative baseline on the next sample instead of assuming measured speed
started at zero. Frequency changes use the updated period in both I and D.

A zero velocity target retains the explicit stop behavior: zero PWM and cleared
integral/derivative history. Nonfinite feedback or invalid timing also clears
runtime state and returns zero output. All shared manager access remains under
the existing controller mutex.

## Migration and validation

Start retuning with Ki=Kd=0, a low Kp, and a small velocity target. Tune velocity
tracking before enabling the position loop. Choose the integral contribution
limit in PWM units; position-loop integral limits remain velocity units.

Do not reuse the old UI defaults (`Kp=0.1, Ki=0`): at rest with a 2.5 rad/s
target they command only 0.25% PWM indefinitely. Even `Kp=1, Ki=0` commands
only 2.5%, often below motor breakaway friction. An integral limit alone does
not enable integration; Ki must also be positive.

The client now starts with `Kp=1, Ki=1, Kd=0, integral_limit=100` for firmware
2.3.1 and later protocol-2 releases. I can supply the full motor output at zero
speed error; total PWM remains capped at +/-100% with anti-windup.
These are starting values, not a tuned
motor profile. At a stationary 2.5 rad/s target, I grows by 2.5 PWM percentage
points per second up to the selected limit. If that limit is below the output
needed to overcome friction or load, the motor can still remain stalled.
Earlier firmware retains its earlier UI defaults. Reinitialize to apply edits,
then set the target again; neither loading the page nor editing gains sends
motor commands.

Host tests cover P-only output, timed integration, bounds, positive/negative
saturation, unwinding, filtered derivative, target-step/initialization behavior,
zero/reset, invalid input and a simulated motor at multiple loop rates. These
checks do not replace tuning and validation on the actual motor and load.
Startup regressions also cover simulated breakaway/running friction, the old
defaults, the reported Kp=1/Ki=0 settings, insufficient integral headroom, and
the new starting gains at 10 Hz and 100 Hz in both directions.
