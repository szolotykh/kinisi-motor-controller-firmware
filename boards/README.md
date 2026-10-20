# Board definitions

This directory contains the hardware configuration for each supported board
revision. Each JSON file is the source of truth for its board identity,
microcontroller, peripherals, and resource assignments.

Board-specific wiring and electrical settings belong in the individual JSON
files. Implementation notes belong in the [firmware documentation](../docs).
`schema_version` identifies the configuration format and is independent of
hardware and firmware versions.

## Generating firmware configuration

Run from the firmware repository root with Python 3.10 or newer:

```sh
python tools/board_config_generator/generator.py
```

The generator validates the definitions and produces
[hw_config.h](../include/hw_config.h) and
[board_revision.h](../include/board_revision.h). Edit the JSON sources and commit
the regenerated outputs alongside them. The same pass also produces a compact
[public board profile bundle](../include/board_profiles.json) for client libraries.
PlatformIO runs generation before
firmware builds and checks that the selected board matches the configured MCU.

Profiles contain board identity, resource counts and hardware features, without
pin or peripheral assignments. They describe physical hardware; firmware command
support is checked separately. See [client board profiles](../docs/board-profiles.md)
for the format and library integration.

To check that generated files are current without modifying them:

```sh
python tools/board_config_generator/generator.py --check
```

Generation is deterministic. The firmware uses the generated C configuration;
it does not parse JSON at runtime.

## Adding a board

1. Copy the closest existing definition and assign a unique board identity and
   build selector.
2. Update the hardware configuration to match the schematic. Keep resource IDs
   stable and ordered from zero, and explicitly declare any shared pins.
3. Add a matching environment in [platformio.ini](../platformio.ini), with the
   correct MCU and build selector.
4. Regenerate the headers, add test coverage for the board, and build its
   environment. Verify operation on the physical hardware.

Boards that use supported components reuse the existing firmware paths. New
MCUs, drivers, or peripheral mappings may require generator and firmware changes.

## Validation

The generator checks required fields, resource IDs, supported mappings, and
resource conflicts before writing output. Firmware-reserved pins cannot be
assigned to board resources, including through shared-pin declarations.
Resource counts are derived from the definitions. Hardware descriptions do not
by themselves add firmware support for a feature.

See the [test guide](../test/README.md) for configuration and firmware checks.
Builds and host tests complement physical board validation.
