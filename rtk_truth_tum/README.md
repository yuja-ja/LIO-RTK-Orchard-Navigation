# rtk_truth_tum

This package uses a second independent NMEA receiver only as an offline RTK
reference. Collection stores the original `/rtk_truth/*` observations in the
same bag as LiDAR, IMU, and the RTK used by the fusion filter. TUM files are
generated after collection so quality gates, time offset, ENU origin, and lever
arm can be changed without recollecting data.

## Build

```bash
cd ~/FAST-LIRO
catkin_make
source devel/setup.bash
```

## Start the second RTK

The second receiver should output `GGA + GST + RMC + THS`. Use a stable
`/dev/serial/by-id/...` device path when possible.

```bash
roslaunch rtk_truth_tum rtk_truth.launch port:=/dev/ttyUSB2 baud:=115200
```

It publishes only namespaced observations:

```text
/rtk_truth/epoch
/rtk_truth/heading
/rtk_truth/fix
/rtk_truth/velocity
/rtk_truth/raw_sentence
```

## Record one bag

```bash
rosbag record --lz4 --buffsize=1024 -O orchard.bag /velodyne_points /imu/data_raw /rtk/epoch /rtk/heading /rtk/fix /rtk/velocity /rtk/time_reference /rtk/raw_sentence /rtk_truth/epoch /rtk_truth/heading /rtk_truth/fix /rtk_truth/velocity /rtk_truth/time_reference /rtk_truth/raw_sentence
```

## Extract the independent antenna reference

Run this in a sourced ROS1 workspace. Only RTK-fixed epochs with GST covariance
pass by default.

```bash
rosrun rtk_truth_tum extract_rtk_truth_from_bag.py --bag /home/gsm/bag/orchard.bag --output /home/gsm/trajectory/rtk_truth_antenna.tum
```

The output point is the second receiver's main-antenna phase center. Its TUM
quaternion is identity because the primary reference is translation. A separate
yaw-only file can be requested with `--yaw-output`.

By default, valid NMEA UTC is used as the physical measurement timestamp when
it differs from the ROS arrival stamp by at most one second. This matches the
default FAST-LIO-RTK fusion timing. The extractor prints the observed
UTC-to-arrival correction range so a residual `--time-offset` can be calibrated.
Use `--use-arrival-time` only for a fusion run configured with
`rtk/use_utc_measurement_time: false`; change the safety bound with
`--max-utc-ros-offset`. THS heading reuses the latest valid UTC-to-arrival
correction and supports a separate `--heading-time-offset`.

## Calibrate lever arm and installation yaw

Use a dynamic data segment with turns and visible attitude excitation. A static
segment or a straight run cannot observe the lever arm reliably. First extract
the RTK position and yaw files:

```bash
rosrun rtk_truth_tum extract_rtk_truth_from_bag.py --bag orchard.bag --output rtk_truth_antenna.tum --yaw-output rtk_truth_yaw.tum
```

Then use the original FAST-LIO2 IMU-center trajectory (preferably before RTK
fusion) and the second RTK trajectory:

```bash
rosrun rtk_truth_tum calibrate_lever_arm_heading.py --estimate-tum FAST_LIO.txt --truth-tum rtk_truth_antenna.tum --truth-yaw-tum rtk_truth_yaw.tum --output-yaml rtk_extrinsic_calibration.yaml --max-dt 0.05
```

The script uses SciPy when available and otherwise falls back to a NumPy-only
robust Gauss-Newton solver. No ROS message input is needed by the calibration
step.

The result contains values that can be copied into the fusion configuration:

```yaml
antenna_lever_arm_i: [x, y, z]
heading_baseline_i: [cos(alpha), sin(alpha), 0.0]
heading_offset_deg: alpha
```

The lever arm is the IMU origin to the RTK main antenna phase center, expressed
in IMU coordinates. The heading offset is the counter-clockwise yaw from IMU
`+X` to the THS main-to-secondary baseline. Run the calibration separately for
each physical RTK antenna if the fusion receiver and the truth receiver are not
co-located.

## Apply the lever arm to each estimate

Measure the vector from the IMU origin to the second RTK main-antenna phase
center, expressed in the FAST-LIO IMU frame. Example `[0.30, -0.15, 0.80]`:

```bash
rosrun rtk_truth_tum transform_imu_tum_to_antenna.py --input FAST_LIO.txt --output FAST_LIO_truth_antenna.tum --lever-arm 0.30 -0.15 0.80
```

Run the same command for `FAST_LIO_RTK.txt`. Each algorithm's own full 3D
orientation moves its IMU-center estimate to the same physical antenna point;
the RTK reference remains independent.

```bash
evo_ape tum rtk_truth_antenna.tum FAST_LIO_truth_antenna.tum -a -r trans_part --t_max_diff 0.05 --plot
```

## Optional planar IMU-center reference

The provided NMEA `THS` sentence contains heading only, not roll or pitch. A
yaw-only lever-arm conversion can be generated, but it assumes a level vehicle
and is not strict 3D truth:

```bash
rosrun rtk_truth_tum extract_rtk_truth_from_bag.py --bag orchard.bag --output rtk_truth_antenna.tum --planar-imu-output rtk_truth_imu_planar.tum --lever-arm 0.30 -0.15 0.80
```
