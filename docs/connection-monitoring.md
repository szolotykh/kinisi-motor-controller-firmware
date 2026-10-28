# Connection monitoring and telemetry

Protocol 2.1 adds a configurable connection watchdog and odometry subscriptions. Protocol 2.0 clients can still initialize the controller; monitoring remains disabled until explicitly configured. The generated [command reference](../commands.md) defines the wire fields and errors.

## Heartbeat

After `INIT` and `READY`, enable monitoring with `SET_HEARTBEAT_CONFIG(enabled=true, timeout_ms=500)`. The controller measures inactivity using board uptime, independently of time synchronization. Any command with a valid header, message ID, payload length and field values refreshes activity, even if the operation returns a resource error. An accepted time-sync response also refreshes activity. Malformed or unknown messages do not.

Clients should send `PING` after one fifth of the configured timeout without another outgoing message: 100 ms for the default 500 ms timeout. PING receives an ordinary correlated ACK. `GET_HEARTBEAT_CONFIG` returns the current settings. Supported timeouts are 100–60,000 ms. Disabling monitoring also removes that connection's subscriptions. Each successful INIT starts with monitoring disabled and the default timeout restored.

On expiry, firmware stops closed-loop controllers, clears their targets and coasts every initialized motor. USB disconnect notifications use the same stop path, even when heartbeat monitoring is disabled. Motor resources are shared between USB and I2C, so this invalidates both sessions and discards their queued commands and subscriptions. A fresh INIT is required before further operations; stop and brake requests remain available. Reconnection alone does not restore motor output or restart a speed controller.

The heartbeat detects loss of client communication. It does not impose a lifetime on motor setpoints while a client continues sending valid traffic.

## Generic subscriptions

The protocol schema defines a stable uint8 topic enum and maps each topic to a
packed sample object. The command generator emits the C enum, payload types,
sizes, selector validation and this mapping in the command reference.

| Topic | ID | Index | Sample object | Minimum sampling interval |
| --- | ---: | --- | --- | --- |
| ENCODER_ODOMETRY | 1 | Encoder index | encoder_odometry_sample | Twice the calculation period |
| PLATFORM_ODOMETRY | 2 | 0 | platform_odometry_sample | Twice the calculation period |
| TEMPERATURE | 3 | Sensor index | temperature_sample | 250 ms |
| IMU | 4 | Sensor index | imu_telemetry_sample | 10 ms |
| MOTOR_CURRENT | 5 | Motor index | motor_current_sample | 20 ms |
| MOTOR_FAULTS | 6 | Motor index | motor_fault_sample | 100 ms |

Generic subscriptions require protocol 2.6; motor current requires 2.7 and
motor faults require 2.8. See [motor current sensing](current-sensing.md) and
[motor fault reporting](motor-faults.md) for their data contracts.

Send SUBSCRIBE(topic_id, index, interval_ms) after INIT/READY and enabling the
heartbeat. It returns an ordinary ACK or ERROR. Intervals are integer
milliseconds, at most 3,600,000. Odometry must already be running. Sensor
availability is checked through the same reader as GET. Initial sample-not-ready
is accepted; a hardware I/O failure at subscription creation returns an error.

One slot is maintained per topic/index per connection. Repeating SUBSCRIBE
replaces the interval only after validation succeeds. The current bounded
storage reserves five odometry slots and eleven sensor slots; exhausting the
sensor slots returns SUBSCRIPTION_LIMIT. There is no allocation or renewal.
UNSUBSCRIBE(topic_id, index) is idempotent and does not stop calculation.
Disabling heartbeat, INIT, disconnect and watchdog expiry remove subscriptions.

TELEMETRY_EVENT (0x82) uses message ID zero and has no ACK. Its payload is two
uint8 fields, topic_id and index, followed by the exact object defined for that
topic. Timestamp and clock metadata are inside the sample, not repeated in the
envelope. Complete frame sizes are 24 bytes for encoder odometry, 40 for platform
odometry, 24 for temperature, 64 for IMU, 24 for motor current, and 26 for motor faults. I2C masters must provide buffers
large enough for the complete frame and continue using POLL_TELEMETRY.

Odometry preserves its original acquisition timestamp. Temperature and IMU
timestamps mark completion of the sensor register read, not the hardware
conversion instant. Sensor conversion rates are unchanged; accelerometer and
gyroscope samples are not synchronized. Successful sensor reads are cached for
the requested interval so transport backpressure cannot cause repeated bus I/O.
After a failed read, no old sample is published; delivery resumes on recovery.
Replies and synchronization keep priority, and telemetry never queues history.

Legacy SUBSCRIBE_ODOMETRY / UNSUBSCRIBE_ODOMETRY and the original event formats
remain supported. Both interfaces share the same odometry slots: the last
successful subscribe selects the interval and event format for that connection.
Either unsubscribe interface removes that source. Independent connections can
use different formats. All existing GET, START, STOP and RESET operations remain.

## Subscription delivery modes

`PERIODIC` (0) sends the latest sample at each interval. `ON_CHANGE` (1)
sends an initial snapshot, then another when the encoded value fields change.
Timestamp and clock metadata are excluded from the comparison. Fault appearance,
clearance, sleep, and diagnostic validity changes are included in the value fields.
Numeric measurements use exact encoded comparisons, with no deadband.

The interval controls how often firmware checks for changes; topic minimums still
apply. Faults can be checked every 100 ms. This is sampled detection, not an
interrupt-driven guarantee. A quiet change subscription remains active, and the
connection heartbeat runs independently. Hardware protection remains independent
of telemetry delivery.

Each connection keeps its own baseline. Replacing a subscription requests a fresh
initial snapshot. Failed reads pause delivery; the next successful read sends a
snapshot even if the value matches the previous one. A failed transmit does not
advance the baseline. Slow transports receive the latest state and can skip
intermediate transitions; subscriptions are not a fault history. On I2C, the
master must continue servicing `poll()` to retrieve queued samples.

Delivery modes require protocol 2.9. SDKs keep the original `SUBSCRIBE`
request for periodic mode, so existing applications and older compatible firmware
keep working. Other modes use `SUBSCRIBE_WITH_MODE` (0x83), with the same topic,
index and uint32 interval followed by a uint8 mode. Events and unsubscribe keep
their existing formats. Unsupported modes are rejected before transmission.

## Legacy odometry subscriptions

These commands and their legacy event messages are **retired since protocol
2.6.0**. They remain supported throughout protocol 2.x and may be removed in 3.0.0.
New code should use the generic subscription commands when firmware supports
them. See the [retirement policy](initialization.md#retired-commands).

`START_ENCODER_ODOMETRY` and `START_PLATFORM_ODOMETRY` continue to control background calculation. GET commands still return a single cached measurement. Neither subscribing nor unsubscribing changes calculation state.

Use `SUBSCRIBE_ODOMETRY(source, interval_ms)` after READY, with heartbeat monitoring enabled and the selected odometry running. Sources 0–3 select encoders; source 4 selects the platform. Subscribing again replaces that source's delivery interval. `UNSUBSCRIBE_ODOMETRY(source)` is idempotent. There is no renewal command.

The delivery interval must be at least twice the actual calculation period. For 20 ms calculations, 40 ms or longer is valid; 50 ms is accepted without rounding it to a calculation boundary. A change to calculation frequency is rejected if it would violate an existing subscription on either connection.

At a delivery deadline, firmware reads the latest completed sample and sends its original measurement timestamp. Calculation and delivery remain independent. Stopping or resetting calculation pauses delivery until a valid sample exists again. Events use message ID zero and do not receive ACKs. Encoder events include the encoder index; platform events contain the timestamped pose.

Scheduling is best effort: transmission and task scheduling introduce latency. A busy transport skips intermediate measurements and sends the latest available sample when it can proceed, without building a backlog or sending a catch-up burst. Normal command responses take priority. Select intervals appropriate for the link bandwidth and use measurement timestamps when computing elapsed time.

### I2C

An I2C slave cannot initiate a bus transfer. `POLL_TELEMETRY` lets a master clock out at most one due event followed by the request's ACK; when no sample is available, only the ACK is returned. The master must recognize events while waiting for ordinary responses as well. USB clients receive events without polling.

Arduino's `poll()` services this exchange and idle heartbeats. It must run frequently in the sketch loop. A 32-byte Wire buffer supports encoder events (23 bytes); platform events require at least 38 bytes. The Arduino SDK rejects a platform subscription locally when the configured Wire capacity is too small.

## USB disconnect detection

The USB disconnect callback and CDC deinitialization notify the command task; motor operations are deferred out of the USB interrupt. The heartbeat watchdog detects loss of client communication when no disconnect notification is received.

The current board does not support separate USB power sensing. Revision 3 uses PA9 for motor PWM; firmware does not read that pin as a USB power input.

## Trigger and delivery settings

`SUBSCRIBE_CONFIGURED` (protocol 2.12) separates when to read from what to send.
It sends an initial snapshot, then applies these settings:

| Setting | Values | Meaning |
| --- | --- | --- |
| `trigger` | `PERIODIC` (0), `ON_EVENT` (1) | Read on the polling schedule or on a source notification. |
| `delivery` | `ALL` (0), `ON_CHANGE` (1) | Send every triggered update, or only changed value fields. |
| `poll_interval_ms` | Integer milliseconds | Required for periodic triggering; zero for event triggering. Topic minimums apply. |
| `max_silence_ms` | Integer milliseconds | For ON_CHANGE, send a fresh snapshot after this long without a successful send, even if unchanged. Zero disables refresh. |

Nonzero intervals must meet the topic minimum and cannot exceed 3,600,000 ms.
ALL requires zero maximum silence. Timestamp and clock metadata do not count as
value changes. Odometry retains its acquisition timestamp; refresh does not start
or restart calculation. Heartbeat monitoring remains independent and required.

Only MOTOR_FAULTS currently supports ON_EVENT. The fault GPIO interrupt records a
notification; the command task reads diagnostics and sends them as soon as it can.
A 100 ms background check catches additional diagnostic changes while the fault
pin is already asserted, and recovery conditions without an interrupt. Driver
initialization enables thermal-warning reporting on the fault pin while preserving
protection settings. Current-regulation reporting on that pin remains unchanged.

ON_EVENT + ALL sends each observed notification batch, including unchanged values,
and changes found by reconciliation. An unchanged background check is not an event.
ON_EVENT + ON_CHANGE filters repeated values and can refresh after maximum silence.
The firmware stores latest state, not a lossless event queue: bursts can coalesce,
transient conditions may be missed, and bus/task delays affect delivery. Failed
reads do not resend old data. Backpressure does not acknowledge a change until sent.
Maximum silence is a best-effort reporting deadline, not a connection watchdog or
a guarantee during failed reads, unavailable measurements, or a blocked transport.

The request payload is `uint8 topic_id`, `uint8 index`, `uint32 poll_interval_ms`,
`uint8 trigger`, `uint8 delivery`, `uint32 max_silence_ms`, in that order. Events,
unsubscribe, session cleanup and capacity limits remain shared with the existing
subscription commands. Existing interval/mode APIs retain their wire behavior.
Configured subscriptions require 2.12 and are rejected locally on older firmware.

On I2C, the board cannot initiate transfers. Continue calling Arduino `poll()`;
event subscriptions request service every 10 ms (divided among active streams).
USB receives notifications without client polling. Hardware protection acts
independently of subscription and transport timing.
