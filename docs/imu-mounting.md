# Install the Kinisi Mini on a planar chassis

On model 1 v0.4.0 with protocol 2.24 or later and the `imu_mounting` capability, enter the signed displacement from the chassis tracking origin to **the geometric center of the PCB underside**. This point lies midway across the board outline on the face opposite the components. Exclude standoffs, connectors and enclosure thickness. IMU-only tracking and both fused observers share this installation.

## Measure the board position

The chassis origin is the point whose motion the platform reports, typically the midpoint of a differential axle or the center of the omni or mecanum wheel geometry. Chassis +X points forward, +Y left and +Z up. A PCB underside centered at the tracking origin has position `(0, 0, 0)` at every orientation. For a board centered 100 mm to the right and 20 mm below the origin, enter `(0, −0.100, −0.020)` metres.

Viewed from behind the robot looking forward, a right-side vertical installation with roll +90° looks like this:

```text
                           +Z up
                             ↑
chassis left  ← +Y           O  tracking origin
                             ·
                             ·                  | • U601
                             ·                  |
                             ···················B  → board +Z, component face
                                                |
                                      B = PCB underside center
```

On the Mini, board +X points away from the USB-C edge (J501), board +Y points toward the long edge nearest U601, and board +Z points out of the component face. At zero orientation these align with the corresponding chassis axes. Firmware applies the BMI088 sensing-axis rotation internally. [BMI088 top view and sensing axes, figures 6 and 12](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi088-ds001.pdf)

## Set and verify the installation

With motion stopped and controllers disarmed, write `SET_BOARD_MOUNT` (0x85) and read `GET_BOARD_MOUNT` (0x86). Roll, pitch and yaw are signed 16-bit centidegrees. X, Y and Z are signed 32-bit micrometres measured in chassis axes. Positive yaw turns board +X left. Roll +90° points board +Y up and the component face to the right. The rotation for column vectors is `Rz(yaw) Ry(pitch) Rx(roll)`. Roll and yaw accept ±180°, pitch ±90°, and each entered offset accepts ±10 m. Readback adds a one-byte configured flag. The complete write and readback frames are 22 and 23 bytes respectively.

For the flat centered board, write six zeros. For the right-side example above, write `(9000, 0, 0, 0, -100000, -20000)`. A changed installation requires fresh tracking initialization.

Firmware computes the chip position as `board_center + R_board_to_chassis × underside_center_to_imu`. In board coordinates the Mini vector is **(−15.373, +26.953, +2.075) mm**. Its horizontal components use the 81.025126 × 66.025126 mm PCB outline and U601 center. The nominal sensing height combines 1.6 mm of PCB thickness and an estimated 0.475 mm above the component face. Firmware rounds each rotated component to the nearest micrometre for reversible readback. The board configuration generates the firmware constants and the public profile's `imu_geometry.underside_center_to_imu_um`. [Mini board configuration](../boards/v0.4.json)

Use `SAVE_CONFIGURATION` to retain the entered reference, coordinates and orientation, including before platform commissioning. A reboot requires fresh stationary calibration. Pose, bias, targets and tracking episodes are volatile. Raw IMU reads remain in sensor axes.

## Existing chip-reference installations

The chip-reference commands are deprecated for new installations. Protocol 2.23's `SET_IMU_MOUNT` and `GET_IMU_MOUNT` (0x6E/0x6F) use chassis coordinates measured to **U601**. Previously saved chip-reference records and old writes preserve that physical chip position. On 2.24 either getter reports the installation in its named reference. A centered, level PCB returns `(−15373, 26953, 2075)` µm in the chip getter. When the installation is unconfigured, both getters return zero coordinates and angles with the configured flag clear.

An installer position at a ±10 m limit can put the other reference up to 31,098 µm beyond the input limit. Both getters report the derived position without clipping. Older SDKs that validate chip readback against the 2.23 input limit may reject such a derived chip position. Use a 2.24 SDK for cross-reference readback at these extremes.

## Read fused telemetry at its source time

With an explicit mount, passive fused telemetry combines the most recent valid
IMU pair with the preceding complete encoder interval. Its reported timestamp
is the latest acquisition used by those three inputs. The next encoder interval
is retained for the following update. The observer uses measured acceleration
and yaw rate to carry the interval-average wheel velocity from its midpoint to
the reported time before applying the wheel correction. Use the returned source
timestamp and age to compare a vector with another measurement. The raw encoder
endpoint and the USB read time can both be later.

## Upgrade a yaw-only installation

Old v0.4 FRAM records preserved the platform setup and, in revision 2, its feedback preference. They contained no IMU mounting yaw. After upgrading, `configured=false` preserves the yaw that each old start or setup call supplies. IMU-only takes the angle of **sensor +X**. Fusion takes the angle of **board +X**. A new installation uses board axes for both.

To reproduce a centered, level legacy installation explicitly, use zero roll and pitch. For an IMU at the chassis origin, use zero XYZ in the chip-reference command, or subtract the rotated intrinsic vector to obtain the board center. For legacy fusion, use its board yaw. For legacy IMU-only, add 180° to its sensor yaw and wrap into [−180°, +180°]. Thus sensor yaw 0° corresponds to board yaw 180°. If two old settings describe different physical orientations, check the board landmarks before choosing the one physical mount. An explicit configuration takes precedence over yaw arguments supplied by old clients. [IMU-only setup](imu-tracking.md) · [Fusion setup](fusion-experiment.md)
