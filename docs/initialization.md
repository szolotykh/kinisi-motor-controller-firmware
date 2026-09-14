# Initial identity exchange

INIT (`0x70`) exchanges identity and starts [clock setup](time-sync.md) for the
connection. Clients must wait for READY before starting or reading odometry. Other commands
remain available without INIT. Repeating INIT resets the connection clock and
sync interval, but does not enable motors, reset odometry, or subscribe to telemetry.
USB and I2C keep separate clock state.

Hardware identity comes from the selected board build (V3 by default), not
physical board detection. `board_model=1` identifies a Kinisi motor controller;
V3 is encoded as `0.3.0`. A build must select the correct board revision.
Only the existing V1/V2/V3 configurations on main are supported here; the
separate V3.1 board configuration must add its own identity when integrated.

There is no existing semantic firmware release version. The response instead
reports the first 16 hexadecimal characters of the Git commit used to build
the firmware. Format the high and low words as eight hexadecimal digits each,
then concatenate. Source archives without Git report zero. Uncommitted changes
are not represented by this ID.
Protocol version `2.0.0` describes the command schema, not a firmware release.

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
| 5 | Minimum protocol minor required, currently 0 | 1 |
| 6 | Protocol patch, informational, currently 0 | 1 |
| 7 | Client capabilities: bit 0 (1) valid wall clock, bit 1 (2) subscription support | 1 |

Capability values are 0 for neither, 1 for wall clock only, 2 for subscription
support only, and 3 for both. Subscription support declares that the client
can handle subscribed telemetry; it does not create a subscription. This
firmware implements INIT and time sync, but no subscription commands.

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
