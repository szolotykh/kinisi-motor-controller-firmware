# Shared command responses (protocol 2.0)

Every message definition in `commands.json` declares `direction` as
`client_to_controller` or `controller_to_client`, including nested `response`
definitions. This is schema metadata for generation and documentation, not a
wire-header field. A nested response travels in the opposite direction to its
request. Controller-initiated TIME_SYNC_REQUEST uses `controller_to_client`;
TIME_SYNC_RESPONSE uses `client_to_controller`.

Ordinary requests dispatched over USB or I2C receive one reply. INIT additionally
receives READY after clock setup (or a setup ERROR). Successful TIME_SYNC_RESPONSE
messages are consumed without an acknowledgment:

```
[length: uint8][command ID: uint8][message_id: uint16][optional payload]
```

Length counts bytes after itself. Normal replies echo the request command.
An empty acknowledgment has length 3 and occupies four bytes.
There is no
status field in the common header.

Failures use the response-only command ERROR (0x7F):

```
[5][0x7F][message_id: uint16][failed command ID][error code]
```

Errors always occupy six bytes and have no additional payload. A zero-length
request uses failed command ID 0 because no ID is available. Sending
ERROR as a request returns UNKNOWN_COMMAND.

Error codes and per-command `errors` arrays are defined in `commands.json`.
The generator produces the C constants and the [error catalog and command lists](../commands.md#error-codes).
The ERROR wire layout stays unchanged; descriptions are documentation, not strings sent by firmware.

Odometry GET errors distinguish missing INIT, initial clock readiness, odometry
not running, and the first sample not yet being available. Stopping odometry
makes GET return ODOMETRY_NOT_INITIALIZED; restarting or resetting it requires
a new sample. Stop/reset and odometry frequency commands do not require clock
readiness. Repeated stop/delete/reset operations retain their harmless no-op
behavior, subject to motor ownership checks.

Example: malformed SET_MOTOR_SPEED with message ID 42 returns `05 7F 2A 00 02 04`. A successful
TOGGLE_STATUS_LED_STATE with message ID 42 returns `03 26 2A 00`. GET replies carry their command-specific payload after the header; odometry GETs
now include a timestamp, clock mode and clock quality. Commands previously without a reply now return an
empty acknowledgment. INIT returns its 15-byte identity payload only on success. Invalid or
incompatible INIT requests use the same fixed ERROR response as other
commands.

A success acknowledgment means the handler accepted the request, not that a motor
has reached its target or a physical operation completed. Existing lower-level
no-op behavior, such as resetting an absent controller, remains unchanged
unless a specific rejection is implemented. Timeouts, full receive queues,
incomplete frames and physical link failures can still prevent a response;
clients need a bounded timeout. An incomplete declared frame must finish
before the parser can interpret another frame; there is no magic resync marker.

## Client migration

This is a breaking change from protocol 1.x raw replies. Existing Python,
JavaScript, Arduino and ROS clients must be updated before using this firmware:

1. Use the shared request header: length, command, message ID, then payload.
2. Read one response length byte, then read exactly that many bytes, handling
   partial reads with a finite timeout.
3. Check length is at least three and match the echoed message ID. If command
   is 0x7F, require length five and decode the failed command ID and error code.
   Otherwise verify the command and decode its payload.
4. Consume the acknowledgment even for commands that previously had no reply.
5. INIT and wait for READY before odometry. Respond to TIME_SYNC_REQUEST both
   while waiting for a reply and while idle; do not wait for an ACK to your
   TIME_SYNC_RESPONSE.

## Message ID sequencing

Each request initiator maintains a 16-bit counter per connection. Start at 1,
increment for each new request, and wrap from 65535 back to 1. Zero is reserved
for unsolicited traffic and errors whose truncated header has no usable ID.
Ordinary requests with ID zero are rejected. Responses and ERROR messages
echo the initiating message ID; they do not allocate a new one.

`protocol_next_message_id(previous)` supplies the increment/wrap operation.
The sender must skip any ID still outstanding. Match incoming replies by
message ID plus expected command, discard late replies for retired IDs, and
clear pending requests when re-establishing a connection. Do not reuse a timed
out ID while its old reply could still arrive. Reusing IDs after a full wrap
cannot distinguish arbitrarily old delayed traffic; recover the connection
if that ambiguity is possible. IDs correlate replies, not deduplicate motor
actions: automatically retrying a non-idempotent command is not guaranteed safe.

The controller initiates [time sync](time-sync.md) using an independent counter.
Its timing requests and READY can arrive between ordinary replies. Client SDKs
must dispatch by command as well as ID and keep answering timing requests after
startup. [Odometry events](connection-monitoring.md) use message ID zero and can
also arrive between replies. Ordinary command dispatch remains serial.

Host tests cover framing, empty and invalid commands, invalid arguments,
reply payloads and acknowledgments, response size bounds, INIT errors, every
split position of concatenated requests, 255-byte declared requests, empty
frames, and queue-full behavior. Physical USB/I2C exchanges remain unverified.
