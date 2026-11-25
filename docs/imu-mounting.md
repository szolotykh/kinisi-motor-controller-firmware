# Install the Kinisi Mini IMU on a planar chassis

On model 1 v0.4 with protocol 2.23 or later and the `imu_mounting` capability, enter the physical installation once for IMU-only tracking, passive fused telemetry and the fusion experiment. Measure to **U601**, the BMI088 package on the component face. The chassis tracking origin is the point whose planar pose and velocity the platform reports, typically the midpoint of the differential axle or the centre of the omni or mecanum wheel geometry.

## Identify the axes

Chassis +X points to the robot front, +Y to its left and +Z up. On the Mini component face, board +X points away from the edge with the USB-C receptacle J501. Board +Y points toward the long edge nearest U601. Board +Z points out of the component face. The chip's in-plane sensing axes point opposite the board axes. Firmware handles that fixed rotation. [BMI088 top view and sensing axes, figures 6 and 12](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi088-ds001.pdf)

Measure the vector **from the tracking origin to the chip**, resolved along the chassis axes, in metres. For a chip 100 mm to the right, enter (0, −0.100, 0), even with the board vertical. A chip ahead of the origin has positive X and one above it has positive Z.

## Set the orientation

At zero angles the board axes coincide with the chassis axes. Enter right-handed roll, pitch and yaw about X, Y and Z in degrees. The board-to-chassis matrix for column vectors is `Rz(yaw) Ry(pitch) Rx(roll)`. For example, +90° roll points board +Y up and the component face toward chassis right. With +180° roll the component face points down. Positive yaw turns board +X toward chassis left. Roll and yaw accept −180° through +180°, pitch −90° through +90°. Each offset accepts −10 through +10 metres.

With the robot stationary and controllers disarmed, set the installation and read it back. At wire level `SET_IMU_MOUNT` takes roll, pitch and yaw as three signed 16-bit centidegree values, then X, Y and Z as three signed 32-bit micrometre values. `GET_IMU_MOUNT` returns the six applied values and a configured flag. Both frames fit a 32-byte Wire buffer. Raw IMU measurements stay in sensor axes. Tracking rotates specific force and angular rate into chassis axes, then accounts for the chip's tangential and centripetal acceleration to report motion at the origin.

Use `SAVE_CONFIGURATION` for a durable installation. Mount geometry can be saved before platform commissioning. After boot, establish stationary calibration and a new tracking origin. Pose, bias, targets and the one-second IMU-only episode are volatile. The installation assumes a rigid sensor on a chassis moving in a horizontal plane.

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

To reproduce a centered, level legacy installation explicitly, use roll, pitch and offsets of zero. For legacy fusion, use its board yaw. For legacy IMU-only, add 180° to its sensor yaw and wrap into [−180°, +180°]. Thus sensor yaw 0° corresponds to board yaw 180°. If two old settings describe different physical orientations, check the board landmarks before choosing the one physical mount. An explicit configuration takes precedence over yaw arguments supplied by old clients. [IMU-only setup](imu-tracking.md) · [Fusion setup](fusion-experiment.md)
