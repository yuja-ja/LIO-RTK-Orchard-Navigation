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

