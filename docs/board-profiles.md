# Client board profiles

The board JSON files are the source for both the firmware C configuration and
the public client profiles. Running `python tools/board_config_generator/generator.py`
generates `include/board_profiles.json` alongside the C headers. PlatformIO's
pre-build generation and the generator's `--check` option cover all three outputs.
The firmware does not parse or transmit the JSON at runtime.

## Public format

The bundle has `schema_version: 1` and a `boards` array. Each profile contains:

- `board`: model number and complete hardware revision.
- `resources`: motor, encoder, GPIO and status LED counts.
- `motor_features`: per-motor IDs, current-sense/current-limit/fault-reporting flags,
  and optional `current_limit_options` with discrete thresholds in milliamps and `can_disable`.
- `sensors`: temperature sensor IDs and IMU IDs with accelerometer/gyroscope flags.

IDs match the corresponding board resource IDs used by protocol commands.
The bundle intentionally excludes MCU/build selectors, control models, pins,
peripherals, addresses, timers and electrical component values. Sensor chip types
and internal driver settings are firmware implementation details.

Current-sense and current-limit flags describe fitted hardware. They do not claim
that measurement or adjustable-current commands exist in a particular firmware.
Current readings require protocol 2.7, fault reporting requires 2.8, and current-limit
configuration requires 2.11. Applications must check firmware command availability
before exposing a control. See [current-limit configuration](motor-current-limit.md).

The currently generated resource counts are:

| Board | Motors | Encoders | GPIO | Status LEDs | Temperature sensors | IMUs |
| --- | --- | --- | --- | --- | --- | --- |
| 0.1.0 | 4 | 1 | 4 | 1 | 0 | 0 |
| 0.2.0 | 4 | 4 | 4 | 1 | 0 | 0 |
| 0.3.0 | 4 | 4 | 10 | 1 | 0 | 0 |
| 0.4.0 | 4 | 4 | 8 | 1 | 1 | 1 |

## Library integration

Libraries bundle this public JSON with their releases or generate an equivalent
module at build time. After INIT, they select an exact match using `board_model`
and `board_major.board_minor.board_patch`. Selection is independent of firmware
protocol and client package versions. The same format can be consumed by
JavaScript, Python or other languages.

The JavaScript SDK imports the bundle using:

```sh
python tools/update-board-profiles.py --profiles ../kinisi-motor-controller-firmware/include/board_profiles.json
```

Its automatically selected `controller.boardProfile` and
`controller.supportsSensor(kind, index)` are available in both transport clients.
The latter combines fitted hardware with firmware protocol support. The Sensors
UI uses the same profiles to show the sensors fitted to each board. A fitted
sensor with older firmware shows an update message.

An unknown identity has no selected profile. Clients must not borrow a different
board revision's profile. The current JavaScript SDK keeps compatible unknown
boards connected for existing commands but does not claim sensor availability;
its UI asks for a client update. Low-level sensor requests on unknown boards
still rely on firmware availability errors. Known absent sensors are rejected
locally before I/O. Disconnecting clears the selected identity and profile.

The Python SDK imports the same public JSON using
`python tools/update-board-profiles.py --profiles ../kinisi-motor-controller-firmware/include/board_profiles.json`
from its project root. It packages `pykinisi/board_profiles.json` in wheels and
source releases. `controller.board_profile` selects the profile automatically;
`controller.supports_sensor(kind, index)` combines hardware and protocol support.
Its `get_temperature(index)` and `get_imu(index)` commands use the same sensor IDs
and units as JavaScript. Both SDKs support explicit branch imports and `--check`.

Only bundled profiles and automatic lookup are supported. Custom profile loading
is reserved for future work.

Arduino uses the same explicit profile import script to generate
`kinisi_board_profiles.h/.cpp` and retain `tools/board_profiles.json`. The library
stores the tables in flash on AVR and selects an exact profile after `begin()`.
`boardProfile()` exposes the selected hardware metadata and
`supportsSensor(KinisiSensor::TEMPERATURE/IMU, index)` combines hardware,
protocol and Wire capacity. Temperature and IMU replies need 12 and 52 bytes
respectively; a default 32-byte AVR Wire buffer supports temperature only.
