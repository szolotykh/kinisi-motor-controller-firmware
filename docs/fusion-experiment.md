# Encoder and IMU experiment

## Checking tracking inputs

On a READY model 1 v0.4.0 board with protocol 2.22 and the `feedback_source_health` capability, read `GET_FEEDBACK_SOURCE_HEALTH(source)` for each input. `ENCODER` reports the platform encoder acquisition, `IMU_ONLY` reports the physical BMI088 accelerometer and gyro pair, and `FUSED` reports the passive fused estimate. These reads work independently of the operator's selected telemetry source.

| Board observation | Operator interpretation |
| --- | --- |
| `HEALTHY` and `FRESH` | A usable source sample lies within its board-clock freshness budget. Check `COMPARISON_AVAILABLE` to see whether the board could cross-check motion. |
| `SUSPECT` or `UNAVAILABLE` | Read `reason`, `detail`, and the freshness and fault flags. Read the other source separately to see which observations are still available. |
| `DISAGREEMENT` | The measured motions conflict. Slip, externally imposed motion and a sensor problem can produce similar evidence. The board reports that uncertainty. |
| `PROVENANCE` with an old timestamp | The board remembers an acquisition. Its age and `FRESH` flag describe whether it remains current. |

`timestamp_us` uses board uptime. `age_us` uses the same clock. `UINT32_MAX` means the age is unknown or outside the representable interval. `GET_FEEDBACK_IMU_CHANNELS` gives separate acquisition times and ages for the accelerometer and gyro. Constant counts at rest, or low acceleration during straight constant-speed travel, can be normal.

For a position or velocity reading, obtain the requested source, active source, validity and origin with `GET_FEEDBACK_SOURCE_STATUS`. A physical IMU can remain healthy after its one-second diagnostic window expires. Start another window deliberately from a stationary board. Fused telemetry requires a confirmed mounting orientation, encoder odometry, and suitable fresh observations. A source change requires stopped shafts and disarmed outputs. After a source outage, reset or reconnect, read the current origin and validity before using coordinates. Telemetry selection and recovery never issue a motion command. [IMU-only tracking](imu-tracking.md) · [Wire reference](../commands.md)

This opt-in experiment requires a READY protocol 2.19+ session and exact model 1
hardware 0.4.0 with `capabilities.fusion_experiment: true`. Missing capabilities,
older firmware and unknown revisions are unavailable. Existing control defaults
are unchanged: reset selects legacy encoder feedback. The experiment is volatile
and is not included in `SAVE_CONFIGURATION`.

The experiment modes are `LEGACY_ENCODER` (0), `ENCODER_CORRECTION` (1) and
`FUSED` (2). Modes 1 and 2 use the same chassis correction for comparison;
mode 0 retains the release path. This is not a general odometry-source selector
and has no IMU-only mode. Comparative accuracy and physical performance remain
unmeasured until separately reported.

On a v0.4 controller with `capabilities.imu_mounting`, configure the physical
board orientation and the offset from the chassis tracking origin to the IMU.
Active fusion, passive fused telemetry and encoder/IMU comparison share that
installation. The passive observer accepts the installation as its mounting
declaration. The chassis +X axis points forward, +Y points left and +Z points
up. The chassis must move in a horizontal plane. [Installation and axis diagram](imu-mounting.md)

With an unconfigured physical mount, the legacy yaw convention still applies.
Install the board level and component side up, centered at the reference point.
`mounting_yaw_rad` is the signed angle of board +X about chassis +Z in [-pi, pi].
Gravity cannot determine yaw. A saved physical mount survives reboot. Control
mode, estimates and bias calibration start fresh.

The mounted consumers rotate all three acceleration and angular-rate axes into
the chassis frame. At a displaced chip they subtract tangential and centripetal
acceleration at the accelerometer source time. Two distinct gyro observations
are required after a reset. The fusion and comparison tasks allow up to 60 ms
between samples for this angular-rate slope, consistent with their maximum
encoder interval. Each input pair still requires both dies within 35 ms of the
encoder sample and at most 15 ms apart. An explicit centered installation does
not require an angular-rate slope.

Configure only with motion stopped and every commanded target and motor output
zero; initialized controllers at idle zero targets are allowed. Firmware rejects
any observed configured encoder movement in the preceding 100 ms, or a counter
change since a fresh sample. Stopped cold setup may ACK in WAITING. An ACK
confirms configuration, not healthy sensing or permission to resume an old target.
Both experimental modes require 100 ms of observed unchanged configured counters
for initial rest arming before accepting a motion target; fused bias calibration
may run within that window. Once armed, ordinary target updates remain accepted
while moving; each update does not require a new stationary window.

After an ordinary sensor fault, the source can requalify automatically. Wait for
healthy feedback, then issue a fresh target through the normal platform API.
On outage recovery, firmware reapplies the last fused-from-encoder world transform
to the current encoder pose. Reconfiguration is only needed for a deliberate mode
or mounting change; it does not replace the healthy-feedback and fresh-target
requirements. Board idle, driver inhibit, stop and watchdog behavior retain
precedence. No SDK read, reconnect or experiment setter sends a motion target.

The physical IMU producer waits 2 ms between passes; this is not a guaranteed
2 ms acquisition cadence. Pairing accepts only
acquisition windows entirely preceding the encoder timestamp, with each window
at most 5 ms wide, worst-case accel/gyro skew at most 15 ms, and age at most 35 ms.
These software acceptance limits do not establish physical timing or accuracy.

Diagnostics are explicit snapshots. Reads never set targets or change
configuration, but may enforce an expired feedback deadline, clear the live
target and coast. `GET_FUSION_STATUS` returns mode, health,
fault, flags, sequence, monotonic `timestamp_us`, and accel/gyro/encoder ages in
microseconds. An age of UINT32_MAX means unavailable. `GET_FUSION_VECTOR(kind)`
returns sequence plus x/y/z for encoder pose, encoder body twist, fused pose,
fused body twist or bias. Pose is relative metres/metres/radians; twist is
m/s, m/s, rad/s; bias is accel-X/accel-Y in m/s² and gyro-Z in rad/s.
Unavailable vectors return `SAMPLE_NOT_AVAILABLE`. Only associate a vector with
a status timestamp/health when their sequences match. Separate reads can cross
an update; retain both sequences and report that mismatch rather than presenting
a coherent snapshot. Logical health is not a physical accuracy guarantee.

`FusionFlag` values are ESTIMATE_VALID=1, BIAS_READY=2,
FRESH_TARGET_REQUIRED=4 and CORRECTION_ACTIVE=8. Health is DISABLED=0,
WAITING=1, HEALTHY=2 or FAULT=3. Fault labels distinguish mount, encoder, IMU,
freshness, timestamps, saturation, bias, discontinuity and inhibit conditions.
Read state after errors and retry only by explicit operator action.

## Wire contract

All fields are packed little endian. The four-byte frame envelope is included
in the frame sizes below.

| Command | ID | Request payload | Response payload | Response frame |
| --- | --- | --- | --- | --- |
| SET_FUSION_EXPERIMENT | 0x5F | uint8 mode, binary64 mounting yaw | ACK | 4 bytes |
| GET_FUSION_STATUS | 0x60 | none | four uint8 state fields, uint32 sequence, uint64 timestamp, three uint32 ages | 32 bytes |
| GET_FUSION_VECTOR | 0x61 | uint8 kind | uint32 sequence, three binary64 values | 32 bytes |
| GET_FUSION_CONFIGURATION | 0x62 | none | uint8 mode, binary64 mounting yaw | 13 bytes |

The named catalogs and exact field order are generated in `include/commands.h`
and `commands.md` from `commands.json`. The lightweight board guard is
`BOARD_HAS_FUSION_EXPERIMENT`, generated only for MP_V4. Source profiles
reject enabling it on another board. No private wiring enters client profiles.
