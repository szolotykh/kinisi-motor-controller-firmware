# Initial identity exchange

INIT (`0x70`) exchanges identity and starts [clock setup](time-sync.md) for the
connection. Clients must wait for READY before starting or reading odometry. Other commands
remain available without INIT until connection loss is latched. Repeating INIT resets
the clock, sync interval, watchdog settings and subscriptions. Reinitializing a monitored
session stops motion; it does not enable motors or reset odometry.
USB and I2C keep separate clock state.

Hardware identity comes from the selected board build (V3 by default), not
physical board detection. `board_model=1` identifies a Kinisi motor controller;
V3 is encoded as `0.3.0`. A build must select the correct board revision.
V1/V2/V3/V4 configurations are supported.

There is no existing semantic firmware release version. The response instead
reports the first 16 hexadecimal characters of the Git commit used to build
the firmware. Format the high and low words as eight hexadecimal digits each,
then concatenate. Source archives without Git report zero. Uncommitted changes
are not represented by this ID.
The protocol version in `commands.json` describes the command schema. It is
independent of the board revision, client library release and firmware build ID.

## Protocol versioning and compatibility

Protocol versions use `major.minor.patch`:

| Component | Release rule | Compatibility |
| --- | --- | --- |
| Major | Breaking changes to framing, existing request/response layouts, field units or command meaning. | The firmware and client must support the same protocol major. A mismatch is rejected. |
| Minor | Compatible additions, such as new commands or optional features. Existing command IDs, payloads and meanings remain supported. | Clients can use the commands supported by both sides. New commands require the firmware minor that introduced them. |
| Patch | Fixes that preserve the documented protocol interface and behavior. | Patch differences do not block connection. |

Within a major version, do not remove commands, reuse existing command IDs or
change an existing command's wire format or documented meaning. To introduce a
different format without breaking existing clients, add a new command ID and
retain the original command. Otherwise, increment the protocol major.

During INIT, the client declares its supported protocol major and the minimum
minor needed for its baseline functionality. The board reports its implemented
protocol version. Compatibility requires matching majors and a board minor at
least as high as the client's required minor. This check establishes compatibility;
it does not select a different wire format or translate between protocol majors.
The client must gate optional commands using the reported firmware version.

For example, a client whose baseline requires `2.1` can connect to firmware
reporting `2.5`. It can request temperature only with protocol `2.4` or later,
and IMU data only with `2.5` or later. A client that supports protocol major `2`
cannot establish a compatible session with firmware reporting major `1`.

Compare communication protocol versions when checking compatibility. Board
revision and client library package version are separate identities; their major
numbers do not have to match each other or the protocol major. Hardware feature
availability must also be checked separately: understanding a sensor command
does not imply that a sensor is fitted to the selected board.

### Retired commands

A command or event is active unless its `commands.json` entry contains `retired`:

```json
"retired": {
  "since": "2.6.0",
  "replacement": "SUBSCRIBE",
  "removal_major": 3
}
```

These are communication protocol versions. `since` records when retirement was
announced, `replacement` names an active message in the same direction, and
`removal_major` is the earliest major that may remove the retired message.
Retirement means supported for compatibility but discouraged for new code.
IDs, payloads and runtime behavior remain unchanged within the current major.
Generators keep retired messages and annotate documentation and SDK APIs; they
never remove them automatically. Removal requires a deliberate major-version
change. When supporting older firmware, clients may still use retired commands
if the replacement is unavailable.

The legacy odometry subscription commands and events are retired in protocol
`2.6.0`, remain supported throughout `2.x`, and may be removed starting with
`3.0.0`. Use generic `SUBSCRIBE`, `UNSUBSCRIBE` and `TELEMETRY_EVENT` instead.
Odometry GET, START, STOP and RESET commands remain active.

## Wire format

Requests and responses share `[length: uint8][command: uint8][message_id: uint16]`.
All multi-byte values are little-endian. Length counts bytes after itself.
INIT has an eight-byte request payload: length 11, total size 12 bytes.
Message ID exists only in the shared header, not the INIT payload.

| INIT payload offset | Field | Size |
| --- | --- | --- |
| 0 | SDK type: 0 unknown, 1 Python, 2 JavaScript, 3 Arduino | 1 |
| 1–3 | Client library version: major, minor, patch | 3 |
| 4 | Protocol major, currently 2 | 1 |
| 5 | Minimum protocol minor required by this client | 1 |
| 6 | Client protocol patch, informational | 1 |
| 7 | Client capabilities: bit 0 (1) valid wall clock, bit 1 (2) subscription support | 1 |

Capability values are 0 for neither, 1 for wall clock only, 2 for subscription
support only, and 3 for both. Subscription support declares that the client
can handle subscribed telemetry; it does not create a subscription. See
[connection monitoring and subscriptions](connection-monitoring.md) for the
commands available from protocol 2.1.

The successful reply has the [shared header](responses.md) and the following
15-byte identity payload: 19 bytes total, length 18, command 0x70 and the echoed message ID. There is no status or request ID in the INIT payload.

| INIT response payload offset | Field | Size |
| --- | --- | --- |
| 0 | Board model, 1 | 1 |
| 1–3 | Board version: major, minor, patch | 3 |
| 4–6 | Protocol version: major, minor, patch | 3 |
| 7 | Firmware Git prefix, high word | 4 |
| 11 | Firmware Git prefix, low word | 4 |

Successful requests return identity. Incompatible or invalid requests return
ERROR with the failed command ID and error code, without identity data.
Compatibility requires the same protocol major and a required minor no newer
than the board's. Protocol patch is reported for identification and does not
affect compatibility. Starting or reading odometry requires successful clock setup and READY. Complete INIT frames with truncated or oversized payloads return INVALID_LENGTH
without identity data. Clients should use a bounded receive timeout and retry or
report unsupported INIT for older firmware that does not answer. Protocol 1.x
clients cannot decode this firmware's framed replies without an update. Clients match replies and errors by header message ID and check the command.
See the shared response format for ID increment and wrap rules.

Client examples are maintained in the [Python project](https://github.com/szolotykh/pykinisi).
See [time sync](time-sync.md) for packet layouts, failure behavior, and uptime mode.

Test with `python test/test_initialization/run_tests.py` (set `CC` if needed),
then build with `pio run -e genericSTM32F405RG`. Hardware exchange validation
over USB/I2C remains required. Client SDK integration is separate work.
