# Connection monitoring and odometry streams

Protocol 2.1 adds a configurable connection watchdog and odometry subscriptions. Protocol 2.0 clients can still initialize the controller; monitoring remains disabled until explicitly configured. The generated [command reference](../commands.md) defines the wire fields and errors.

## Heartbeat

After `INIT` and `READY`, enable monitoring with `SET_HEARTBEAT_CONFIG(enabled=true, timeout_ms=500)`. The controller measures inactivity using board uptime, independently of time synchronization. Any command with a valid header, message ID, payload length and field values refreshes activity, even if the operation returns a resource error. An accepted time-sync response also refreshes activity. Malformed or unknown messages do not.

Clients should send `PING` after one fifth of the configured timeout without another outgoing message: 100 ms for the default 500 ms timeout. PING receives an ordinary correlated ACK. `GET_HEARTBEAT_CONFIG` returns the current settings. Supported timeouts are 100–60,000 ms. Disabling monitoring also removes that connection's subscriptions. Each successful INIT starts with monitoring disabled and the default timeout restored.

On expiry, firmware stops closed-loop controllers, clears their targets and coasts every initialized motor. USB disconnect notifications use the same stop path, even when heartbeat monitoring is disabled. Motor resources are shared between USB and I2C, so this invalidates both sessions and discards their queued commands and subscriptions. A fresh INIT is required before further operations; stop and brake requests remain available. Reconnection alone does not restore motor output or restart a speed controller.

The heartbeat detects loss of client communication. It does not impose a lifetime on motor setpoints while a client continues sending valid traffic.

## Odometry subscriptions

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
