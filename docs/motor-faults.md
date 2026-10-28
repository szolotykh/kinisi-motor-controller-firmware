# Motor fault reporting

`GET_MOTOR_FAULTS(motor_index)` reads diagnostic state for one motor.
`SUBSCRIBE` with `MOTOR_FAULTS` streams the same fields with a completion
timestamp and clock metadata. Indices are 0–3, the minimum periodic interval
is 100 ms, and a connection watchdog must be enabled. Both require protocol 2.8.
The public board profile's per-motor `fault_reporting` flag identifies fitted
hardware. Boards without it return `SENSOR_NOT_AVAILABLE`.

Reads never initialize, wake, enable, brake, stop, or clear a motor driver.
No motor controller or odometry initialization is required to make the request.
The existing motor-initialization command wakes and reconfigures the driver;
it also clears latched faults through the sleep control. Read diagnostics before
reinitializing when investigating a fault.

## Clearing latched faults

`CLEAR_MOTOR_FAULTS(motor_index)` stops the selected motor and its active
controller, clears latched flags through the driver's SPI clear bit, and returns
the same `motor_fault_status` object as `GET_MOTOR_FAULTS`. The motor stays
disabled with zero PWM. This command requires protocol 2.10, a ready connection,
an initialized motor, and an awake driver. It does not initialize or wake a driver.
Before connection initialization it returns `INIT_REQUIRED`; before `READY` it
returns `CLOCK_NOT_READY`. Both checks happen before hardware changes.

Faults are not cleared automatically by telemetry reads or subscriptions. Read
diagnostics before clearing if they are needed for troubleshooting. Some driver
conditions recover automatically; others retain latched flags until cleared.
A successful command means the clear and readback completed, not that every
fault disappeared. Active conditions may remain or recur, so inspect the returned
status before deciding to restart. Motion requires a separate explicit command.

The implementation preserves IC3's other bits and does not change current
regulation, protection, or control settings. It verifies the self-clearing bit
and reads fresh diagnostics. Missing power, a sleeping driver, or SPI/readback
failure returns `INTERNAL_ERROR`; after execution begins the motor remains
stopped even on failure. An uninitialized motor returns `MOTOR_NOT_INITIALIZED`.
An initialized motor without fault hardware returns `SENSOR_NOT_AVAILABLE`.
Platform ownership takes precedence: a platform-owned motor returns `MOTOR_OWNED`
before any hardware changes, consistent with the other direct motor commands.
Release the platform before clearing individual motors.

An active `ON_CHANGE` subscription reports changed diagnostic values at its next
fresh sample. The clear response supplies the immediate result; an unchanged
state does not generate an extra event. A complete clear request is 5 bytes and
its response is 14 bytes, fitting the default Arduino Wire buffer.

## Interpreting a result

The pull response contains ten one-byte booleans (10 payload bytes, 14 bytes
including the protocol header). A subscription adds the ten-byte timestamp/clock
prefix and two-byte topic/index selector, making a complete 26-byte frame.

| Field | Meaning |
| --- | --- |
| `sleep_requested` | The sleep control is asserted. Diagnostic reads leave it unchanged. |
| `status_valid` | Both diagnostic registers returned valid SPI frames. |
| `fault_active` | The fault input is asserted, or a valid register reports the global fault flag. |
| `overtemperature_warning` | Driver temperature warning. |
| `undervoltage` | Reported driver-supply undervoltage. |
| `charge_pump_undervoltage` | Reported charge-pump undervoltage. |
| `overcurrent` | Overcurrent reported globally or for either output transistor pair. |
| `overtemperature_shutdown` | Reported thermal shutdown. |
| `open_load` | Open-load diagnostic reported on either output. |
| `current_regulation` | Either output reports current regulation; this is separate from overcurrent protection. |

When `status_valid` is false, the detailed flags are unknown. They are encoded as
false to avoid carrying an earlier sample forward. A clear fault input alone
does not establish that the driver is powered or healthy. A sleeping driver is
reported without SPI traffic. An invalid response can mean missing motor power,
a communication problem, or a driver failure; it does not identify the exact cause.

For the DRV8873S, firmware reads the FAULT and DIAG registers and validates the
SPI status-header bits and the reserved FAULT bit. All-zero and all-one responses
are rejected. The two registers are read sequentially, so a snapshot can span a
transition. Some flags are latched and can describe an earlier fault. Polling can
miss transient conditions that the driver does not latch. It does not create an
event history or change the existing protection behavior.

Open-load detection depends on its configuration and operating conditions;
no reported open load is not proof that a motor is connected. See the
[DRV8873 datasheet, status registers and protection behavior](https://www.ti.com/lit/ds/symlink/drv8873.pdf).

## Validation

The motor-control host tests exercise all four board configurations, individual
status bits, asleep/unavailable drivers, invalid SPI headers, recovery, and
unchanged motor outputs and fault registers during reads. Clear tests cover all
four motors, preserved configuration, stopped outputs, persistent conditions,
and failures at each SPI transfer. Protocol tests cover fault-event
framing, four simultaneous streams, cadence, and unsubscribe. Hardware fault
injection and electrical validation are separate from these host tests.

Use `ON_CHANGE` to send an initial snapshot and subsequent diagnostic changes.
See [subscription delivery modes](connection-monitoring.md#subscription-delivery-modes)
for sampling intervals, transport behavior, and compatibility.


For prompt notification, use ON_EVENT with the independent delivery settings in
[trigger and delivery settings](connection-monitoring.md#trigger-and-delivery-settings).
Interrupt handling never reads SPI or clears faults. An initial snapshot and the
100 ms reconciliation still work when no new pin edge arrives. Event snapshots
record diagnostic-read completion time, not interrupt time. Driver initialization
enables thermal-warning reporting on nFAULT; it does not disable any protection.
