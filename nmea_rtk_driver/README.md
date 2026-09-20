# nmea_rtk_driver

Read-only ROS1 Noetic serial driver for checksum-protected NMEA ASCII output
from an RTK receiver. The node never writes configuration commands to the
receiver.

## Recommended receiver output

Configure the receiver with its vendor tool or a serial terminal before
starting ROS. The recommended direct-observation set is:

- `GGA`: primary-antenna position, fix quality, satellite count, HDOP,
  ellipsoid-height components, differential age and station ID.
- `GST`: position uncertainty and horizontal error ellipse.
- `RMC`: UTC date, horizontal speed and true course.
- `GSA`: fix dimension, used satellites and PDOP/HDOP/VDOP for status/gating.
- `ZDA`: optional explicit UTC date/time for `sensor_msgs/TimeReference`.
- `THS`: optional dual-antenna true heading on receivers that support it.

For the document's `COM2` output at 10 Hz, enter these receiver commands with a
serial assistant (they are not sent by this ROS node):

```text
GPGGA COM2 0.1
GPGST COM2 0.1
GPRMC COM2 0.1
GPTHS COM2 0.1
GPGSA COM2 1
GPZDA COM2 1
```

`GPTHS` applies to the UM982 dual-antenna receiver. The command name uses the
`GP` prefix, while a multi-constellation receiver may emit the corresponding
sentence with a `GN` talker prefix. The parser accepts any talker prefix.

`GNS` can replace or back up `GGA`; `VTG` can replace `RMC`. Do not configure
both members of a pair as independent measurements. The driver assembles one
epoch and selects only one position source and one velocity source.

The documented `GNS` modes `F` (dynamic RTK) and `R` (static RTK) do not mean
RTK float and fixed. They are published as `FIX_RTK_UNSPECIFIED=9`; the original
mode string is retained. Use `GGA` quality 4/5 when fixed/float distinction is
required.

`GGAH`, `GNSH`, `GSTH`, `RMCH` and `VTGH` are secondary/from-antenna results.
They are checksum-checked and counted, but are deliberately rejected from all
primary position, velocity and epoch topics. They remain visible on the raw
topic when `publish_raw` is enabled.

`GSV`, `GBS`, `GRS`, `ROT` and other valid NMEA sentences are retained only on
the raw topic in this package. They are not treated as independent ESIKF
observations: `GSV` is signal diagnostics, `GBS/GRS` is integrity/residual
diagnostics, and `ROT` lacks a receiver-provided covariance and clear
independence from the navigation solution.

## Time semantics

Measurement headers use ROS arrival time, not NMEA UTC. For each serial read,
the driver timestamps the end of the read and back-projects each sentence's
first `$` byte using `serial_bits_per_byte / baud`. `timestamp_offset_sec` can
apply a measured constant transport correction. This is still software arrival
time and is not hardware time synchronization.

The raw UTC fields are preserved in `RtkEpoch`. When an RMC date is present,
`utc_valid` and `utc_stamp` provide the full UTC instant. `ZDA`, or RMC as a
fallback, is also published as `sensor_msgs/TimeReference`; it never replaces
the measurement header stamp.

## Topics

- `/rtk/fix` (`sensor_msgs/NavSatFix`): WGS-84 ellipsoid altitude and ENU
  covariance. GST error ellipses produce a full horizontal covariance including
  the east/north cross term.
- `/rtk/velocity` (`geometry_msgs/TwistWithCovarianceStamped`): ENU horizontal
  velocity. RMC/VTG has no vertical velocity, so the configured vertical
  variance is intentionally large.
- `/rtk/epoch` (`nmea_rtk_driver/RtkEpoch`): one de-duplicated UTC epoch with
  source, quality, arrival times, heights, DOP, covariance provenance and raw
  UTC/date fields.
- `/rtk/heading` (`nmea_rtk_driver/RtkHeading`): THS true heading, clockwise
  from true north. It is not an ENU yaw and must not be confused with RMC/VTG
  course over ground.
- `/rtk/status` (`nmea_rtk_driver/ReceiverStatus`): GSA status and DOP.
- `/rtk/time_reference` (`sensor_msgs/TimeReference`): receiver UTC mapping.
- `/rtk/raw_sentence` (`std_msgs/String`): framed NMEA including unsupported and
  rejected secondary-antenna sentences.
- `/diagnostics` (`diagnostic_msgs/DiagnosticArray`): rates, checksum/framing
  errors, reconnects, duplicate epochs and secondary-antenna rejection count.

## Build and run

Place this directory under the catkin workspace `src` directory, then run:

```bash
cd ~/FAST-LIRO
catkin_make
source devel/setup.bash
roslaunch nmea_rtk_driver nmea_rtk_driver.launch \
  port:=/dev/ttyUSB1 baud:=115200
```

The Linux user must have read/write permission for the serial device, normally
through membership in the `dialout` group.

Edit `config/nmea_rtk_driver.yaml` for source selection and noise values. The
default `GGA + GST + RMC` selection is the recommended input for later RTK
updates in the shared ESIKF. With `wait_for_gst` and `wait_for_velocity`
enabled, the driver keeps an epoch open until the matching covariance and
velocity arrive; it publishes a partial epoch only after
`epoch_stale_timeout_sec`.

## Tests

Catkin runs the protocol test when testing is enabled:

```bash
catkin_make run_tests_nmea_rtk_driver
catkin_test_results
```

The parser, epoch assembler and byte-stream framer have no ROS dependency and
can also be tested directly:

```bash
g++ -std=c++14 -Wall -Wextra -Wpedantic -Werror \
  -Inmea_rtk_driver/include \
  nmea_rtk_driver/src/nmea_parser.cpp \
  nmea_rtk_driver/src/epoch_assembler.cpp \
  nmea_rtk_driver/test/nmea_protocol_test.cpp \
  -o /tmp/nmea_protocol_test
/tmp/nmea_protocol_test
```

The tests cover checksums, arbitrary talkers, all supported sentence types,
GNS RTK semantics, secondary-antenna identification, UTC conversion, epoch
de-duplication, source preference, cross-read framing, multiple sentences per
read, leading garbage, malformed bytes, oversize recovery and bad checksums.
