#!/usr/bin/env python3
"""Extract an independent RTK reference trajectory from a ROS1 bag."""

import argparse
import bisect
import io
import math
import os
import statistics
import sys

import rosbag


PI = math.pi
DEG_TO_RAD = PI / 180.0
RTK_FIXED = 4


def finite(value):
    return math.isfinite(float(value))


def wrap_angle(angle):
    while angle > PI:
        angle -= 2.0 * PI
    while angle <= -PI:
        angle += 2.0 * PI
    return angle


def geodetic_to_ecef(latitude_deg, longitude_deg, height_m):
    semi_major = 6378137.0
    eccentricity_squared = 6.69437999014e-3
    latitude = latitude_deg * DEG_TO_RAD
    longitude = longitude_deg * DEG_TO_RAD
    sin_latitude = math.sin(latitude)
    cos_latitude = math.cos(latitude)
    radius = semi_major / math.sqrt(
        1.0 - eccentricity_squared * sin_latitude * sin_latitude
    )
    return (
        (radius + height_m) * cos_latitude * math.cos(longitude),
        (radius + height_m) * cos_latitude * math.sin(longitude),
        (radius * (1.0 - eccentricity_squared) + height_m) * sin_latitude,
    )


def make_ecef_to_enu(latitude_deg, longitude_deg):
    latitude = latitude_deg * DEG_TO_RAD
    longitude = longitude_deg * DEG_TO_RAD
    sin_latitude = math.sin(latitude)
    cos_latitude = math.cos(latitude)
    sin_longitude = math.sin(longitude)
    cos_longitude = math.cos(longitude)
    return (
        (-sin_longitude, cos_longitude, 0.0),
        (
            -sin_latitude * cos_longitude,
            -sin_latitude * sin_longitude,
            cos_latitude,
        ),
        (
            cos_latitude * cos_longitude,
            cos_latitude * sin_longitude,
            sin_latitude,
        ),
    )


def matrix_vector(matrix, vector):
    return tuple(
        sum(matrix[row][column] * vector[column] for column in range(3))
        for row in range(3)
    )


def subtract(first, second):
    return tuple(first[index] - second[index] for index in range(3))


def quality_reason(message, args):
    if not (
        message.position_valid
        and message.coordinates_valid
        and message.ellipsoid_height_valid
    ):
        return "invalid_position"
    if message.fix_quality != RTK_FIXED:
        return "not_rtk_fixed"
    if not all(
        finite(value)
        for value in (
            message.latitude_deg,
            message.longitude_deg,
            message.ellipsoid_height_m,
        )
    ):
        return "nonfinite_position"
    if message.satellites_used < args.min_satellites:
        return "satellites"
    if message.hdop_valid and message.hdop > args.max_hdop:
        return "hdop"
    if message.pdop_valid and message.pdop > args.max_pdop:
        return "pdop"
    if (
        message.differential_age_valid
        and message.differential_age_sec > args.max_differential_age
    ):
        return "differential_age"
    if not message.position_covariance_valid:
        return "missing_covariance"
    if not args.allow_configured_covariance and message.position_covariance_source not in (
        "GST_ERROR_ELLIPSE",
        "GST_LAT_LON_ALT",
    ):
        return "covariance_not_gst"

    covariance = message.position_covariance_enu
    east_variance = covariance[0]
    east_north = 0.5 * (covariance[1] + covariance[3])
    north_variance = covariance[4]
    up_variance = covariance[8]
    if not all(
        finite(value)
        for value in (east_variance, east_north, north_variance, up_variance)
    ):
        return "nonfinite_covariance"
    if east_variance < 0.0 or north_variance < 0.0 or up_variance < 0.0:
        return "negative_covariance"
    discriminant = max(
        0.0,
        (east_variance - north_variance) ** 2 + 4.0 * east_north ** 2,
    )
    maximum_horizontal_variance = 0.5 * (
        east_variance + north_variance + math.sqrt(discriminant)
    )
    if math.sqrt(maximum_horizontal_variance) > args.max_horizontal_std:
        return "horizontal_std"
    if math.sqrt(up_variance) > args.max_vertical_std:
        return "vertical_std"
    return ""


def nearest_heading(headings, heading_times, stamp, maximum_age):
    if not headings:
        return None
    insertion = bisect.bisect_left(heading_times, stamp)
    candidates = []
    if insertion < len(headings):
        candidates.append(headings[insertion])
    if insertion > 0:
        candidates.append(headings[insertion - 1])
    if not candidates:
        return None
    best = min(candidates, key=lambda item: abs(item[0] - stamp))
    if abs(best[0] - stamp) > maximum_age:
        return None
    return best[1]


def epoch_measurement_stamp(message, args):
    arrival_stamp = message.header.stamp.to_sec()
    if not finite(arrival_stamp) or arrival_stamp <= 0.0:
        return None, None
    if not args.use_utc_measurement_time or not message.utc_valid:
        return arrival_stamp + args.time_offset, None

    utc_stamp = message.utc_stamp.to_sec()
    correction = utc_stamp - arrival_stamp
    if (
        finite(utc_stamp)
        and utc_stamp > 0.0
        and finite(correction)
        and abs(correction) <= args.max_utc_ros_offset
    ):
        return utc_stamp + args.time_offset, correction
    return arrival_stamp + args.time_offset, None


def yaw_quaternion(heading_true_rad, heading_offset_deg):
    body_heading = heading_true_rad + heading_offset_deg * DEG_TO_RAD
    yaw_enu = wrap_angle(0.5 * PI - body_heading)
    return yaw_enu, (0.0, 0.0, math.sin(0.5 * yaw_enu), math.cos(0.5 * yaw_enu))


def antenna_to_planar_imu(position, yaw_enu, lever_arm):
    cosine = math.cos(yaw_enu)
    sine = math.sin(yaw_enu)
    lever_enu = (
        cosine * lever_arm[0] - sine * lever_arm[1],
        sine * lever_arm[0] + cosine * lever_arm[1],
        lever_arm[2],
    )
    return subtract(position, lever_enu)


def make_parent(path):
    parent = os.path.dirname(os.path.abspath(path))
    if parent and not os.path.isdir(parent):
        os.makedirs(parent)


def write_tum(path, records, comments):
    make_parent(path)
    records.sort(key=lambda record: record[0])
    unique = []
    for record in records:
        serialized_stamp = "{:.9f}".format(record[0])
        if unique and serialized_stamp == unique[-1][0]:
            unique[-1] = (serialized_stamp, record)
        else:
            unique.append((serialized_stamp, record))
    with io.open(path, "w", encoding="ascii") as stream:
        stream.write("# timestamp tx ty tz qx qy qz qw\n")
        for comment in comments:
            stream.write("# {0}\n".format(comment))
        for _, record in unique:
            stream.write(
                "{:.9f} {:.9f} {:.9f} {:.9f} "
                "{:.9f} {:.9f} {:.9f} {:.9f}\n".format(*record)
            )
    return len(unique)


def parse_arguments():
    parser = argparse.ArgumentParser(
        description="Extract /rtk_truth RTK-fixed reference data from a ROS1 bag."
    )
    parser.add_argument("--bag", required=True)
    parser.add_argument("--output", required=True, help="antenna-center TUM output")
    parser.add_argument("--yaw-output", default="")
    parser.add_argument("--planar-imu-output", default="")
    parser.add_argument("--epoch-topic", default="/rtk_truth/epoch")
    parser.add_argument("--heading-topic", default="/rtk_truth/heading")
    parser.add_argument("--time-offset", type=float, default=0.0)
    parser.add_argument(
        "--heading-time-offset",
        type=float,
        default=None,
        help="residual heading offset; defaults to --time-offset",
    )
    parser.add_argument(
        "--use-arrival-time",
        dest="use_utc_measurement_time",
        action="store_false",
        help="timestamp positions at ROS arrival instead of valid NMEA UTC",
    )
    parser.set_defaults(use_utc_measurement_time=True)
    parser.add_argument(
        "--max-utc-ros-offset",
        type=float,
        default=1.0,
        help="maximum accepted absolute UTC-to-arrival offset in seconds",
    )
    parser.add_argument("--min-consecutive", type=int, default=5)
    parser.add_argument("--min-satellites", type=int, default=10)
    parser.add_argument("--max-hdop", type=float, default=2.0)
    parser.add_argument("--max-pdop", type=float, default=3.5)
    parser.add_argument("--max-differential-age", type=float, default=5.0)
    parser.add_argument("--max-horizontal-std", type=float, default=0.10)
    parser.add_argument("--max-vertical-std", type=float, default=0.20)
    parser.add_argument("--max-heading-age", type=float, default=0.08)
    parser.add_argument("--heading-offset-deg", type=float, default=0.0)
    parser.add_argument(
        "--lever-arm",
        nargs=3,
        type=float,
        metavar=("X", "Y", "Z"),
        default=(0.0, 0.0, 0.0),
        help="IMU origin to truth main antenna, expressed in IMU frame (m)",
    )
    parser.add_argument(
        "--manual-origin",
        nargs=3,
        type=float,
        metavar=("LAT_DEG", "LON_DEG", "ELLIPSOID_HEIGHT_M"),
    )
    parser.add_argument("--allow-configured-covariance", action="store_true")
    return parser.parse_args()


def main():
    args = parse_arguments()
    if not os.path.isfile(args.bag):
        raise IOError("bag file does not exist: {0}".format(args.bag))
    args.min_consecutive = max(1, args.min_consecutive)
    args.max_utc_ros_offset = max(0.0, args.max_utc_ros_offset)
    if args.heading_time_offset is None:
        args.heading_time_offset = args.time_offset

    epochs = []
    headings = []
    utc_corrections = []
    latest_measurement_time_correction = None
    with rosbag.Bag(args.bag, "r") as bag:
        topic_info = bag.get_type_and_topic_info().topics
        if args.epoch_topic not in topic_info:
            raise RuntimeError("epoch topic not found: {0}".format(args.epoch_topic))
        topics = [args.epoch_topic]
        if args.heading_topic in topic_info:
            topics.append(args.heading_topic)
        for topic, message, _ in bag.read_messages(topics=topics):
            if topic == args.epoch_topic:
                stamp, correction = epoch_measurement_stamp(message, args)
                if stamp is None:
                    continue
                epochs.append((stamp, message))
                if correction is not None:
                    latest_measurement_time_correction = correction
                    utc_corrections.append(correction)
            elif (
                message.valid
                and message.mode == "A"
                and finite(message.heading_true_rad)
            ):
                correction = (
                    latest_measurement_time_correction
                    if args.use_utc_measurement_time
                    and latest_measurement_time_correction is not None
                    else 0.0
                )
                headings.append(
                    (message.header.stamp.to_sec() + correction
                     + args.heading_time_offset,
                     message.heading_true_rad)
                )

    headings.sort(key=lambda item: item[0])
    heading_times = [item[0] for item in headings]
    epochs.sort(key=lambda item: item[0])

    if args.manual_origin:
        origin_latitude, origin_longitude, origin_height = args.manual_origin
        origin_set = True
    else:
        origin_latitude = origin_longitude = origin_height = 0.0
        origin_set = False
    origin_ecef = None
    ecef_to_enu = None

    antenna_records = []
    yaw_records = []
    planar_imu_records = []
    rejection_counts = {}
    good_streak = 0
    warmup = 0

    for stamp, message in epochs:
        reason = quality_reason(message, args)
        if reason:
            rejection_counts[reason] = rejection_counts.get(reason, 0) + 1
            good_streak = 0
            continue
        good_streak += 1
        if good_streak < args.min_consecutive:
            warmup += 1
            continue

        if not origin_set:
            origin_latitude = message.latitude_deg
            origin_longitude = message.longitude_deg
            origin_height = message.ellipsoid_height_m
            origin_set = True
        if origin_ecef is None:
            origin_ecef = geodetic_to_ecef(
                origin_latitude, origin_longitude, origin_height
            )
            ecef_to_enu = make_ecef_to_enu(origin_latitude, origin_longitude)

        ecef = geodetic_to_ecef(
            message.latitude_deg,
            message.longitude_deg,
            message.ellipsoid_height_m,
        )
        position_enu = matrix_vector(ecef_to_enu, subtract(ecef, origin_ecef))
        antenna_records.append(
            (stamp,) + position_enu + (0.0, 0.0, 0.0, 1.0)
        )

        heading = nearest_heading(
            headings, heading_times, stamp, args.max_heading_age
        )
        if heading is None:
            continue
        yaw_enu, quaternion = yaw_quaternion(heading, args.heading_offset_deg)
        yaw_records.append((stamp,) + position_enu + quaternion)
        if args.planar_imu_output:
            imu_position = antenna_to_planar_imu(
                position_enu, yaw_enu, args.lever_arm
            )
            planar_imu_records.append((stamp,) + imu_position + quaternion)

    if not antenna_records:
        raise RuntimeError("no RTK-fixed truth epochs passed the configured gates")

    origin_comment = "origin_wgs84_ellipsoid: {:.10f} {:.10f} {:.4f}".format(
        origin_latitude, origin_longitude, origin_height
    )
    antenna_count = write_tum(
        args.output,
        antenna_records,
        (origin_comment, "position is the truth main-antenna phase center"),
    )
    yaw_count = 0
    if args.yaw_output:
        yaw_count = write_tum(
            args.yaw_output,
            yaw_records,
            (origin_comment, "orientation contains yaw only; no roll or pitch"),
        )
    planar_count = 0
    if args.planar_imu_output:
        print(
            "WARNING: planar IMU truth assumes roll=pitch=0; it is not strict 3D truth.",
            file=sys.stderr,
        )
        planar_count = write_tum(
            args.planar_imu_output,
            planar_imu_records,
            (
                origin_comment,
                "IMU center uses yaw-only lever arm and assumes roll=pitch=0",
            ),
        )

    print("antenna poses written: {0}".format(antenna_count))
    print("yaw poses written: {0}".format(yaw_count))
    print("planar IMU poses written: {0}".format(planar_count))
    print("warmup epochs skipped: {0}".format(warmup))
    print("rejections: {0}".format(rejection_counts))
    if utc_corrections:
        print(
            "UTC-arrival correction used: count={0}, min={1:.6f} s, "
            "median={2:.6f} s, max={3:.6f} s".format(
                len(utc_corrections),
                min(utc_corrections),
                statistics.median(utc_corrections),
                max(utc_corrections),
            )
        )
    elif args.use_utc_measurement_time:
        print("UTC-arrival correction used: count=0 (arrival-time fallback)")


if __name__ == "__main__":
    try:
        main()
    except Exception as error:
        print("ERROR: {0}".format(error), file=sys.stderr)
        sys.exit(1)
