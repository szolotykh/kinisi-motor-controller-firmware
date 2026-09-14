# Controller-driven time synchronization

Protocol 2.0 requires INIT and READY before starting or reading odometry. Other commands,
including motor stop/brake, remain available independently. No subscription is
required. Sync has its own schedule, defaulting to a burst every 30 seconds.

1. Client sends INIT with its SDK/protocol versions and capabilities.
2. Controller returns its existing identity response. Identity is not READY.
3. With the valid-wall-clock capability (bit 0), the controller sends three
   TIME_SYNC_REQUEST messages, one at a time. The client promptly answers each
   with its receive and send times in Unix microseconds, echoing the message ID.
4. Controller accepts the valid sample with the lowest corrected round-trip
   delay, installs the clock offset, then sends READY with the INIT message ID.
5. Client may now start/request odometry. A GET returns ODOMETRY_NOT_INITIALIZED if odometry is not running, or
   SAMPLE_NOT_AVAILABLE until the first measurement has actually been acquired.

Clients without valid wall time (for example an Arduino without an RTC) declare
bit 0 clear. The controller sends READY in uptime mode immediately after the
identity reply, skips sync exchanges, and reports microseconds since board boot.
Capability is explicit; SDK type does not determine clock availability.

## Messages

Every frame uses `[length:u8][command:u8][message_id:u16][payload]`, little-endian.
Length excludes itself. Controller IDs increment from 1 through 65535 and wrap
to 1; they are independent of client IDs. Match both the command and ID.

| Message | Direction | Payload | Total bytes |
| --- | --- | --- | --- |
| INIT `0x70` | Client -> controller | Existing 8-byte INIT payload | 12 |
| INIT reply `0x70` | Controller -> client | Existing 15-byte identity | 19 |
| TIME_SYNC_REQUEST `0x71` | Controller -> client | None | 4 |
| TIME_SYNC_RESPONSE `0x72` | Client -> controller | `host_receive_us:u64`, `host_send_us:u64` | 20 |
| READY `0x73` | Controller -> client | `clock_mode:u8` | 5 |
| SET_TIME_SYNC_INTERVAL `0x74` | Client -> controller | `interval_ms:u32`, 1000..3600000 | 8 |
| GET_TIME_STATUS `0x75` | Client -> controller | None | 4 |
| GET_TIME_STATUS reply | Controller -> client | `clock_mode:u8`, `clock_quality:u8`, `interval_ms:u32`, `last_sync_age_us:u64` | 18 |

SET_TIME_SYNC_INTERVAL receives an empty ACK and applies to the current
connection, without saving to flash. INIT resets it to 30000 ms. Successful
TIME_SYNC_RESPONSE messages are consumed without an ACK. Invalid length,
unexpected/expired ID, or unusable timestamps receive the shared ERROR format.
The client must continue servicing incoming timing requests between ordinary
commands, even when it does not request odometry.

## Timing and failures

The controller records C1 immediately before its nonblocking transport send,
and C4 when it dispatches the host response. The host provides H2 on receiving
the complete request and H3 immediately before sending its response:

```
corrected round trip = (C4 - C1) - (H3 - H2)
host offset         = midpoint(H2, H3) - midpoint(C1, C4)
reported sample     = controller acquisition time + host offset
```

The offset estimates equal transport delay in both directions. Software queue
and scheduling delays affect accuracy; microsecond units do not guarantee
microsecond synchronization. Replies must arrive within 1 second; negative
processing times, processing longer than the controller round trip, and
corrected round trips above 100 ms are rejected. A burst uses the best of up
to three attempts, including timed-out attempts. One valid sample is sufficient.
Scheduled sync takes priority over dispatching more ordinary requests, so a
continuous request stream cannot starve it.

If all initial attempts fail, the controller sends ERROR for INIT with code 8
(TIME_SYNC_FAILED), leaves odometry unavailable, and retries at the configured
interval. The client should report this failure with a bounded connection
timeout; it may issue INIT again to retry immediately. Repeated INIT resets the
connection's clock readiness and interval without changing motor state.

Later failed bursts retain the previous mapping. Clock quality becomes stale
when its age exceeds the configured interval plus 3 seconds. Later successful
sync restores valid quality. GET_TIME_STATUS reports quality 0 (unready), 1
(valid), or 2 (stale); its age is zero when unready or using uptime. Stale
timestamps retain wall-clock mode; they are not silently changed to uptime.

Clock mode is 0 (uptime) or 1 (Unix wall clock). The mapping can step forward
or backward at resync, including after host clock corrections; this first
implementation does not slew or estimate frequency drift. Control-loop timing
continues to use the board's monotonic clock. Stopping odometry makes subsequent GETs return ODOMETRY_NOT_INITIALIZED; frequency
configuration, reset, and stop operations do not require clock readiness.
Clients needing smooth elapsed
time must not derive motor-control intervals from wall-clock timestamps.

## Timestamped odometry

GET_ENCODER_ODOMETRY returns this 18-byte payload (22-byte frame):

```
timestamp_us:u64, clock_mode:u8, clock_quality:u8, angle:double
```

GET_PLATFORM_ODOMETRY returns this 34-byte payload (38-byte frame):

```
timestamp_us:u64, clock_mode:u8, clock_quality:u8, x:double, y:double, t:double
```

The odometry task timestamps the encoder acquisition batch and stores the pose
and acquisition time together under its mutex. GET returns that saved sample,
mapped using the requesting connection's clock; it never substitutes the GET
receipt time. Clock quality describes the mapping at reply time. Encoder
start/reset and platform start/reset invalidate the previous sample until the task
captures a new one. A new mapping
can therefore slightly change the reported wall timestamp of a previously
cached sample if it is fetched again after resync.

## Transport and platform limits

USB and I2C maintain independent connection clock mappings. Definite USB
disconnect/deinitialization clears the USB session and queued requests; clients
must INIT on every connection or reconnect. I2C has no connection-loss event,
so its master must send INIT when establishing a new logical session. Closing
an application without a USB disconnect is not reliably detectable here.

The I2C controller is a slave: the master must issue reads to receive identity,
timing requests, READY, and ordinary replies. Sync timeouts begin when the
transmit is accepted, not while waiting for a master read. Reads may clock
stretch until a frame is available; the master needs a bounded transaction
timeout. Read a complete frame in one transaction (the existing I2C send buffer
does not support a separate length-only read followed by a second transaction).
USB/I2C sends are nonblocking so one idle transport does not hold the command
task in a transmit loop.

The STM32 implementation reads the existing 1 MHz TIM14 counter plus the HAL
millisecond tick, extending its 32-bit rollover in software. TIM14 must retain
that configuration, its tick must stay active, and this clock must be read at
least once every 49 days (the normal command poll reads it every millisecond).
Interrupts must not be masked across multiple millisecond overflows. Board
revisions changing the HAL timebase must adapt `hw_clock_microseconds()`.

This work adds no telemetry subscription or motor-disconnect watchdog. Those
remain separate features; a time-sync failure is not a motor-stop mechanism.

## Validation

Run `python test/test_time_sync/run_tests.py` with a host C compiler in `CC`,
then `python test/test_initialization/run_tests.py` and
`pio run -e genericSTM32F405RG`. The new tests use virtual clocks and simulated
transport availability, the production odometry task with mocked encoders/RTOS,
and hardware tick/counter rollover.
Physical USB/I2C timing accuracy and reconnect behavior still need board tests.

Client examples are maintained in the [Python project](https://github.com/szolotykh/pykinisi).
Existing Python/JavaScript/Arduino/ROS SDK integration remains separate work.
