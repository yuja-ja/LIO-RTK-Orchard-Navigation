#!/usr/bin/env python3
"""Static-accuracy evaluation for the dual-receiver RTK reference setup.

Purpose
-------
Reviewer feedback asked how the reliability and uncertainty of the RTK
reference trajectory was established. This script answers that question for
the static case using data the vehicle already records:

  * per-receiver static scatter (2DRMS about the session mean),
  * between-receiver consistency (paired /rtk/epoch vs /rtk_truth/epoch
    differences) while both antennas are stationary,
  * comparison of the data-derived antenna separation with a tape-measured
    baseline (--tape-horizontal, --tape-dz).

Inputs
------
A ROS1 bag containing /rtk/epoch and /rtk_truth/epoch (both are
nmea_rtk_driver/RtkEpoch messages). A dedicated static session of >= 60 s
with open sky above both antennas is the intended use. Static segments are
detected automatically from the truth receiver; use --static-window START
DURATION to force a specific window (seconds relative to the first paired
epoch).

Only RTK-fixed epochs with a valid GST covariance pass by default, using
the same quality gates as extract_rtk_truth_from_bag.py.

Run (inside a sourced ROS1 workspace)
-------------------------------------
  rosrun rtk_truth_tum evaluate_rtk_static.py --bag orchard_static.bag \
      --tape-horizontal 0.62 --tape-dz 0.05

The tape values describe the vector FROM the primary main antenna TO the
truth main antenna: --tape-horizontal is the horizontal distance between
the two antenna mounting points, --tape-dz is the vertical offset
(positive if the truth antenna is higher). The data-derived baseline uses
the same convention (truth minus primary).

A self-test of the analysis pipeline (no bag required) is available:

  python3 evaluate_rtk_static.py --selftest
"""

import argparse
import bisect
import csv
import math
import os
import statistics
import sys

RTK_FIXED = 4


def finite(value):
    return math.isfinite(float(value))


# --------------------------------------------------------------------------
# Geodetic helpers (same formulas as extract_rtk_truth_from_bag.py)
# --------------------------------------------------------------------------

def geodetic_to_ecef(latitude_deg, longitude_deg, height_m):
    semi_major = 6378137.0
    eccentricity_squared = 6.69437999014e-3
    latitude = latitude_deg * math.pi / 180.0
    longitude = longitude_deg * math.pi / 180.0
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
    latitude = latitude_deg * math.pi / 180.0
    longitude = longitude_deg * math.pi / 180.0
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


def horizontal_std_from_covariance(message):
    """Largest horizontal standard deviation implied by the ENU covariance."""
    if not message.position_covariance_valid:
        return None
    covariance = message.position_covariance_enu
    if len(covariance) < 9:
        return None
    east_variance = covariance[0]
    east_north = 0.5 * (covariance[1] + covariance[3])
    north_variance = covariance[4]
    if not all(
        finite(value) for value in (east_variance, east_north, north_variance)
    ):
        return None
    if east_variance < 0.0 or north_variance < 0.0:
        return None
    discriminant = max(
        0.0,
        (east_variance - north_variance) ** 2 + 4.0 * east_north ** 2,
    )
    maximum_horizontal_variance = 0.5 * (
        east_variance + north_variance + math.sqrt(discriminant)
    )
    return math.sqrt(maximum_horizontal_variance)


# --------------------------------------------------------------------------
# Quality gating (mirrors extract_rtk_truth_from_bag.py)
# --------------------------------------------------------------------------

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


def measurement_stamp(message, args):
    arrival = message.position_arrival_stamp.to_sec()
    if not finite(arrival) or arrival <= 0.0:
        return None
    if args.use_utc and message.utc_valid:
        utc = message.utc_stamp.to_sec()
        if (
            finite(utc)
            and utc > 0.0
            and abs(utc - arrival) <= args.max_utc_ros_offset
        ):
            return utc
    return arrival


# --------------------------------------------------------------------------
# Bag reading (rosbag is imported lazily so --selftest runs without ROS)
# --------------------------------------------------------------------------

def read_bag_epochs(bag_path, topic, args):
    try:
        import rosbag
    except ImportError as error:
        raise RuntimeError(
            "rosbag is required; run inside a sourced ROS1 workspace"
        ) from error
    if not os.path.isfile(bag_path):
        raise IOError("bag file does not exist: {0}".format(bag_path))
    with rosbag.Bag(bag_path, "r") as bag:
        topic_info = bag.get_type_and_topic_info().topics
        if topic not in topic_info:
            raise RuntimeError("epoch topic not found in bag: {0}".format(topic))
        samples = []
        stamp_failures = 0
        for _, message, _ in bag.read_messages(topics=[topic]):
            stamp = measurement_stamp(message, args)
            if stamp is None:
                stamp_failures += 1
                continue
            samples.append((stamp, message))
    samples.sort(key=lambda item: item[0])
    return samples, stamp_failures


def filter_epochs(samples, args):
    kept = []
    counts = {}
    for stamp, message in samples:
        reason = quality_reason(message, args)
        if reason:
            counts[reason] = counts.get(reason, 0) + 1
        else:
            kept.append((stamp, message))
    return kept, counts


def resolve_origin(truth_samples, args):
    if args.origin:
        latitude, longitude, height = args.origin
    else:
        _, message = truth_samples[0]
        latitude = message.latitude_deg
        longitude = message.longitude_deg
        height = message.ellipsoid_height_m
    origin_ecef = geodetic_to_ecef(latitude, longitude, height)
    rotation = make_ecef_to_enu(latitude, longitude)
    return origin_ecef, rotation, (latitude, longitude, height)


def convert_to_enu(samples, origin_ecef, rotation):
    converted = []
    for stamp, message in samples:
        ecef = geodetic_to_ecef(
            message.latitude_deg,
            message.longitude_deg,
            message.ellipsoid_height_m,
        )
        east, north, up = matrix_vector(rotation, subtract(ecef, origin_ecef))
        hstd = horizontal_std_from_covariance(message)
        speed = message.speed_mps if message.velocity_valid else None
        converted.append((stamp, east, north, up, hstd, speed))
    return converted


# --------------------------------------------------------------------------
# Pairing, static-segment detection, statistics
# --------------------------------------------------------------------------

def pair_samples(primary_enu, truth_enu, max_dt):
    """Pair each truth sample with the nearest primary sample within max_dt."""
    primary_times = [sample[0] for sample in primary_enu]
    pairs = []
    dt_abs = []
    for stamp, te, tn, tu, th, tspeed in truth_enu:
        insertion = bisect.bisect_left(primary_times, stamp)
        best = None
        candidates = []
        if insertion < len(primary_times):
            candidates.append(insertion)
        if insertion > 0:
            candidates.append(insertion - 1)
        for candidate in candidates:
            dt = primary_times[candidate] - stamp
            if abs(dt) <= max_dt:
                if best is None or abs(dt) < abs(best[1]):
                    best = (candidate, dt)
        if best is None:
            continue
        index, dt = best
        pe, pn, pu, ph, pspeed = primary_enu[index][1:]
        pairs.append((stamp, pe, pn, pu, ph, te, tn, tu, th, tspeed, dt))
        dt_abs.append(abs(dt))
    return pairs, dt_abs


def detect_static_segments(pairs, args):
    """Return (segments, merged_chunks) over pairs using truth positions."""
    times = [pair[0] for pair in pairs]
    truth_east = [pair[5] for pair in pairs]
    truth_north = [pair[6] for pair in pairs]
    speeds = [pair[9] for pair in pairs]
    count = len(pairs)

    chunk_indices = []
    start = 0
    while start < count:
        chunk_start_time = times[start]
        stop = start
        while stop < count and times[stop] - chunk_start_time < args.chunk_sec:
            stop += 1
        if stop - start >= args.min_chunk_samples:
            east = truth_east[start:stop]
            north = truth_north[start:stop]
            mean_east = sum(east) / float(len(east))
            mean_north = sum(north) / float(len(north))
            radius = max(
                math.hypot(e - mean_east, n - mean_north)
                for e, n in zip(east, north)
            )
            chunk_speeds = [s for s in speeds[start:stop] if s is not None]
            speed_ok = (
                not chunk_speeds
                or max(chunk_speeds) <= args.max_static_speed
            )
            if radius <= args.max_static_radius and speed_ok:
                chunk_indices.append((start, stop))
        start = stop

    merged = []
    for chunk_start, chunk_stop in chunk_indices:
        if merged and chunk_start == merged[-1][1]:
            merged[-1] = (merged[-1][0], chunk_stop)
        else:
            merged.append((chunk_start, chunk_stop))

    segments = []
    for seg_start, seg_stop in merged:
        if times[seg_stop - 1] - times[seg_start] >= args.min_static_duration:
            segments.append((seg_start, seg_stop))
    return segments, merged


def mean(values):
    return sum(values) / float(len(values))


def sample_std(values, mu):
    if len(values) < 2:
        return 0.0
    return math.sqrt(
        sum((value - mu) ** 2 for value in values) / (len(values) - 1)
    )


def receiver_stats(east, north, up, reported_hstd):
    mean_east = mean(east)
    mean_north = mean(north)
    mean_up = mean(up)
    stats = {}
    stats["mean_e_m"] = mean_east
    stats["mean_n_m"] = mean_north
    stats["mean_u_m"] = mean_up
    stats["std_e_mm"] = 1000.0 * sample_std(east, mean_east)
    stats["std_n_mm"] = 1000.0 * sample_std(north, mean_north)
    stats["std_u_mm"] = 1000.0 * sample_std(up, mean_up)
    stats["2drms_h_mm"] = 1000.0 * 2.0 * math.hypot(
        sample_std(east, mean_east), sample_std(north, mean_north)
    )
    stats["max_dev_h_mm"] = 1000.0 * max(
        math.hypot(e - mean_east, n - mean_north)
        for e, n in zip(east, north)
    )
    stats["max_dev_u_mm"] = 1000.0 * max(
        abs(u - mean_up) for u in up
    )
    valid_hstd = [h for h in reported_hstd if h is not None]
    stats["mean_reported_hstd_mm"] = (
        1000.0 * mean(valid_hstd) if valid_hstd else None
    )
    stats["n_reported_hstd"] = len(valid_hstd)
    return stats


def segment_stats(pairs, start, stop, args, tape_horizontal=None, tape_dz=None):
    index = range(start, stop)
    primary_east = [pairs[k][1] for k in index]
    primary_north = [pairs[k][2] for k in index]
    primary_up = [pairs[k][3] for k in index]
    primary_hstd = [pairs[k][4] for k in index]
    truth_east = [pairs[k][5] for k in index]
    truth_north = [pairs[k][6] for k in index]
    truth_up = [pairs[k][7] for k in index]
    truth_hstd = [pairs[k][8] for k in index]

    de = [t - p for t, p in zip(truth_east, primary_east)]
    dn = [t - p for t, p in zip(truth_north, primary_north)]
    du = [t - p for t, p in zip(truth_up, primary_up)]

    result = {}
    result["segment"] = 0
    result["t_start"] = pairs[start][0]
    result["t_end"] = pairs[stop - 1][0]
    result["duration_s"] = result["t_end"] - result["t_start"]
    result["n_pairs"] = stop - start
    result["primary"] = receiver_stats(
        primary_east, primary_north, primary_up, primary_hstd
    )
    result["truth"] = receiver_stats(
        truth_east, truth_north, truth_up, truth_hstd
    )

    mean_de = mean(de)
    mean_dn = mean(dn)
    mean_du = mean(du)
    result["diff_mean_de_m"] = mean_de
    result["diff_mean_dn_m"] = mean_dn
    result["diff_mean_du_m"] = mean_du
    result["diff_std_de_mm"] = 1000.0 * sample_std(de, mean_de)
    result["diff_std_dn_mm"] = 1000.0 * sample_std(dn, mean_dn)
    result["diff_std_du_mm"] = 1000.0 * sample_std(du, mean_du)
    result["diff_rms_h_about_mean_mm"] = 1000.0 * math.sqrt(
        sum(
            (a - mean_de) ** 2 + (b - mean_dn) ** 2
            for a, b in zip(de, dn)
        )
        / float(len(de))
    )

    result["data_baseline_h_m"] = math.hypot(mean_de, mean_dn)
    result["data_baseline_dz_m"] = mean_du
    result["tape_baseline_h_m"] = tape_horizontal
    result["tape_baseline_dz_m"] = tape_dz
    result["baseline_h_diff_mm"] = (
        1000.0 * (result["data_baseline_h_m"] - tape_horizontal)
        if tape_horizontal is not None
        else None
    )
    result["baseline_dz_diff_mm"] = (
        1000.0 * (mean_du - tape_dz) if tape_dz is not None else None
    )
    return result


# --------------------------------------------------------------------------
# Reporting
# --------------------------------------------------------------------------

def format_mm(value):
    return "    n/a" if value is None else "{0:7.1f}".format(value)


def print_segment(result):
    primary = result["primary"]
    truth = result["truth"]
    print("")
    print(
        "segment {0}: t = {1:.2f} .. {2:.2f} s "
        "(duration {3:.1f} s, {4} paired epochs)".format(
            result["segment"],
            result["t_start"],
            result["t_end"],
            result["duration_s"],
            result["n_pairs"],
        )
    )
    print("  primary receiver (/rtk/epoch):")
    print(
        "    std E/N/U     : {0} / {1} / {2} mm".format(
            format_mm(primary["std_e_mm"]),
            format_mm(primary["std_n_mm"]),
            format_mm(primary["std_u_mm"]),
        )
    )
    print("    2DRMS_h       : {0} mm".format(format_mm(primary["2drms_h_mm"])))
    print(
        "    max dev h/U   : {0} / {1} mm".format(
            format_mm(primary["max_dev_h_mm"]),
            format_mm(primary["max_dev_u_mm"]),
        )
    )
    print(
        "    reported hstd : {0} mm (mean over {1} epochs)".format(
            format_mm(primary["mean_reported_hstd_mm"]),
            primary["n_reported_hstd"],
        )
    )
    print("  truth receiver (/rtk_truth/epoch):")
    print(
        "    std E/N/U     : {0} / {1} / {2} mm".format(
            format_mm(truth["std_e_mm"]),
            format_mm(truth["std_n_mm"]),
            format_mm(truth["std_u_mm"]),
        )
    )
    print("    2DRMS_h       : {0} mm".format(format_mm(truth["2drms_h_mm"])))
    print(
        "    max dev h/U   : {0} / {1} mm".format(
            format_mm(truth["max_dev_h_mm"]),
            format_mm(truth["max_dev_u_mm"]),
        )
    )
    print(
        "    reported hstd : {0} mm (mean over {1} epochs)".format(
            format_mm(truth["mean_reported_hstd_mm"]),
            truth["n_reported_hstd"],
        )
    )
    print("  difference (truth - primary):")
    print(
        "    mean dE/dN/dU : {0:+.4f} / {1:+.4f} / {2:+.4f} m".format(
            result["diff_mean_de_m"],
            result["diff_mean_dn_m"],
            result["diff_mean_du_m"],
        )
    )
    print(
        "    std dE/dN/dU  : {0} / {1} / {2} mm".format(
            format_mm(result["diff_std_de_mm"]),
            format_mm(result["diff_std_dn_mm"]),
            format_mm(result["diff_std_du_mm"]),
        )
    )
    print(
        "    RMS_h about mean: {0} mm".format(
            format_mm(result["diff_rms_h_about_mean_mm"])
        )
    )
    print("  antenna baseline:")
    print(
        "    data          : horizontal {0:.4f} m, dU {1:+.4f} m".format(
            result["data_baseline_h_m"], result["data_baseline_dz_m"]
        )
    )
    if result["tape_baseline_h_m"] is None:
        print("    tape          : not provided (use --tape-horizontal, --tape-dz)")
    else:
        tape_dz = (
            result["tape_baseline_dz_m"]
            if result["tape_baseline_dz_m"] is not None
            else 0.0
        )
        print(
            "    tape          : horizontal {0:.4f} m, dU {1:+.4f} m".format(
                result["tape_baseline_h_m"], tape_dz
            )
        )
        print(
            "    difference    : horizontal {0} mm, vertical {1} mm".format(
                format_mm(result["baseline_h_diff_mm"]),
                format_mm(result["baseline_dz_diff_mm"]),
            )
        )


SUMMARY_COLUMNS = [
    "segment", "t_start", "t_end", "duration_s", "n_pairs",
    "primary_std_e_mm", "primary_std_n_mm", "primary_std_u_mm",
    "primary_2drms_h_mm", "primary_max_dev_h_mm", "primary_max_dev_u_mm",
    "primary_mean_reported_hstd_mm", "primary_n_reported_hstd",
    "truth_std_e_mm", "truth_std_n_mm", "truth_std_u_mm",
    "truth_2drms_h_mm", "truth_max_dev_h_mm", "truth_max_dev_u_mm",
    "truth_mean_reported_hstd_mm", "truth_n_reported_hstd",
    "diff_mean_de_m", "diff_mean_dn_m", "diff_mean_du_m",
    "diff_std_de_mm", "diff_std_dn_mm", "diff_std_du_mm",
    "diff_rms_h_about_mean_mm",
    "data_baseline_h_m", "data_baseline_dz_m",
    "tape_baseline_h_m", "tape_baseline_dz_m",
    "baseline_h_diff_mm", "baseline_dz_diff_mm",
]


def flatten(result):
    primary = result["primary"]
    truth = result["truth"]
    return {
        "segment": result["segment"],
        "t_start": result["t_start"],
        "t_end": result["t_end"],
        "duration_s": result["duration_s"],
        "n_pairs": result["n_pairs"],
        "primary_std_e_mm": primary["std_e_mm"],
        "primary_std_n_mm": primary["std_n_mm"],
        "primary_std_u_mm": primary["std_u_mm"],
        "primary_2drms_h_mm": primary["2drms_h_mm"],
        "primary_max_dev_h_mm": primary["max_dev_h_mm"],
        "primary_max_dev_u_mm": primary["max_dev_u_mm"],
        "primary_mean_reported_hstd_mm": primary["mean_reported_hstd_mm"],
        "primary_n_reported_hstd": primary["n_reported_hstd"],
        "truth_std_e_mm": truth["std_e_mm"],
        "truth_std_n_mm": truth["std_n_mm"],
        "truth_std_u_mm": truth["std_u_mm"],
        "truth_2drms_h_mm": truth["2drms_h_mm"],
        "truth_max_dev_h_mm": truth["max_dev_h_mm"],
        "truth_max_dev_u_mm": truth["max_dev_u_mm"],
        "truth_mean_reported_hstd_mm": truth["mean_reported_hstd_mm"],
        "truth_n_reported_hstd": truth["n_reported_hstd"],
        "diff_mean_de_m": result["diff_mean_de_m"],
        "diff_mean_dn_m": result["diff_mean_dn_m"],
        "diff_mean_du_m": result["diff_mean_du_m"],
        "diff_std_de_mm": result["diff_std_de_mm"],
        "diff_std_dn_mm": result["diff_std_dn_mm"],
        "diff_std_du_mm": result["diff_std_du_mm"],
        "diff_rms_h_about_mean_mm": result["diff_rms_h_about_mean_mm"],
        "data_baseline_h_m": result["data_baseline_h_m"],
        "data_baseline_dz_m": result["data_baseline_dz_m"],
        "tape_baseline_h_m": result["tape_baseline_h_m"],
        "tape_baseline_dz_m": result["tape_baseline_dz_m"],
        "baseline_h_diff_mm": result["baseline_h_diff_mm"],
        "baseline_dz_diff_mm": result["baseline_dz_diff_mm"],
    }


def write_pair_csv(path, pairs, segments):
    index_to_segment = {}
    for segment_id, (start, stop) in enumerate(segments):
        for k in range(start, stop):
            index_to_segment[k] = segment_id
    with open(path, "w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow([
            "segment", "t",
            "e_primary", "n_primary", "u_primary",
            "e_truth", "n_truth", "u_truth",
            "de", "dn", "du", "hstd_primary", "hstd_truth",
        ])
        for k, pair in enumerate(pairs):
            _, pe, pn, pu, ph, te, tn, tu, th, _, _ = pair
            writer.writerow([
                index_to_segment.get(k, -1),
                "{0:.6f}".format(pair[0]),
                "{0:.6f}".format(pe), "{0:.6f}".format(pn),
                "{0:.6f}".format(pu),
                "{0:.6f}".format(te), "{0:.6f}".format(tn),
                "{0:.6f}".format(tu),
                "{0:.6f}".format(te - pe), "{0:.6f}".format(tn - pn),
                "{0:.6f}".format(tu - pu),
                "" if ph is None else "{0:.6f}".format(ph),
                "" if th is None else "{0:.6f}".format(th),
            ])


def write_summary_csv(path, results):
    with open(path, "w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(SUMMARY_COLUMNS)
        for result in results:
            flat = flatten(result)
            writer.writerow([
                "" if flat[column] is None else flat[column]
                for column in SUMMARY_COLUMNS
            ])


def parse_arguments():
    parser = argparse.ArgumentParser(
        description=(
            "Evaluate static accuracy of the dual-receiver RTK setup from "
            "paired /rtk/epoch and /rtk_truth/epoch data."
        )
    )
    parser.add_argument("--bag", default="", help="input ROS1 bag")
    parser.add_argument("--primary-topic", default="/rtk/epoch")
    parser.add_argument("--truth-topic", default="/rtk_truth/epoch")
    parser.add_argument("--out-pairs", default="", help="optional CSV of paired epochs")
    parser.add_argument("--out-summary", default="", help="optional CSV of segment statistics")
    parser.add_argument(
        "--tape-horizontal",
        type=float,
        default=None,
        help="tape-measured horizontal distance between the two main antennas (m)",
    )
    parser.add_argument(
        "--tape-dz",
        type=float,
        default=None,
        help="tape-measured vertical offset, truth antenna minus primary antenna (m)",
    )
    parser.add_argument(
        "--origin",
        nargs=3,
        type=float,
        metavar=("LAT_DEG", "LON_DEG", "ELLIPSOID_HEIGHT_M"),
        help="fixed ENU origin; default is the first passing truth epoch",
    )
    parser.add_argument(
        "--use-utc",
        action="store_true",
        help="use NMEA UTC stamps instead of position arrival stamps",
    )
    parser.add_argument("--max-utc-ros-offset", type=float, default=1.0)
    parser.add_argument("--min-satellites", type=int, default=10)
    parser.add_argument("--max-hdop", type=float, default=2.0)
    parser.add_argument("--max-pdop", type=float, default=3.5)
    parser.add_argument("--max-differential-age", type=float, default=5.0)
    parser.add_argument("--max-horizontal-std", type=float, default=0.10)
    parser.add_argument("--max-vertical-std", type=float, default=0.20)
    parser.add_argument("--allow-configured-covariance", action="store_true")
    parser.add_argument(
        "--max-dt",
        type=float,
        default=0.05,
        help="maximum primary-truth pairing time difference (s)",
    )
    parser.add_argument("--chunk-sec", type=float, default=2.0)
    parser.add_argument("--min-chunk-samples", type=int, default=3)
    parser.add_argument(
        "--max-static-radius",
        type=float,
        default=0.03,
        help="horizontal radius about the chunk mean to call a chunk static (m)",
    )
    parser.add_argument(
        "--max-static-speed",
        type=float,
        default=0.5,
        help="largest reported speed allowed inside a static chunk (m/s)",
    )
    parser.add_argument(
        "--min-static-duration",
        type=float,
        default=60.0,
        help="minimum merged static segment duration (s)",
    )
    parser.add_argument(
        "--static-window",
        nargs=2,
        type=float,
        metavar=("START", "DURATION"),
        help=(
            "force one analysis window, seconds relative to the first "
            "paired epoch (bypasses automatic static detection)"
        ),
    )
    parser.add_argument(
        "--selftest",
        action="store_true",
        help="run the pipeline on synthetic data with known statistics",
    )
    return parser.parse_args()


# --------------------------------------------------------------------------
# Self-test (no ROS required)
# --------------------------------------------------------------------------

def selftest(args):
    import random

    rng = random.Random(1234)
    sample_count = 1800  # 180 s at 10 Hz
    rate = 10.0
    base_east, base_north, base_up = 0.50, 0.30, 0.12
    truth_samples = []
    primary_samples = []
    for k in range(sample_count):
        stamp = 1000.0 + k / rate + rng.gauss(0.0, 0.0005)
        relative = stamp - 1000.0
        bump = 0.0
        if 70.0 < relative < 75.0:
            bump = (relative - 70.0) * 0.2
        truth_samples.append((
            stamp,
            bump + rng.gauss(0.0, 0.008),
            rng.gauss(0.0, 0.008),
            base_up + rng.gauss(0.0, 0.015),
            0.010,
            0.0,
        ))
        primary_samples.append((
            stamp + 0.010 + rng.gauss(0.0, 0.0005),
            bump + base_east + rng.gauss(0.0, 0.010),
            base_north + rng.gauss(0.0, 0.010),
            rng.gauss(0.0, 0.020),
            0.012,
            0.0,
        ))

    pairs, dt_abs = pair_samples(primary_samples, truth_samples, args.max_dt)
    assert len(pairs) == sample_count, "expected all epochs to pair"
    median_dt = statistics.median(dt_abs)
    assert 0.005 < median_dt < 0.015, "unexpected pairing offset"

    segments, _ = detect_static_segments(pairs, args)
    # The disturbance at t=70..75 s splits the 180 s session into two static
    # segments, both longer than the 60 s minimum duration. Pick the longest
    # one for the statistics below.
    assert len(segments) == 2, "expected two static segments, got {0}".format(len(segments))
    segments.sort(key=lambda bounds: bounds[1] - bounds[0], reverse=True)
    start, stop = segments[0]
    duration = pairs[stop - 1][0] - pairs[start][0]
    assert 60.0 < duration < 115.0, "unexpected segment duration {0}".format(duration)

    result = segment_stats(pairs, start, stop, args)
    assert 20.0 < result["truth"]["2drms_h_mm"] < 26.0, result["truth"]["2drms_h_mm"]
    assert 23.0 < result["primary"]["2drms_h_mm"] < 34.0, result["primary"]["2drms_h_mm"]
    assert (
        11.0 < result["diff_rms_h_about_mean_mm"] < 20.0
    ), result["diff_rms_h_about_mean_mm"]
    assert (
        abs(result["truth"]["mean_reported_hstd_mm"] - 10.0) < 1e-6
    ), result["truth"]["mean_reported_hstd_mm"]

    tape_h = math.hypot(base_east, base_north)
    result = segment_stats(pairs, start, stop, args, tape_h, base_up)
    assert result["baseline_h_diff_mm"] is not None
    assert abs(result["baseline_h_diff_mm"]) < 5.0, result["baseline_h_diff_mm"]
    assert result["baseline_dz_diff_mm"] is not None
    assert abs(result["baseline_dz_diff_mm"]) < 5.0, result["baseline_dz_diff_mm"]

    print("SELFTEST PASSED")


# --------------------------------------------------------------------------
# Main
# --------------------------------------------------------------------------

def main():
    args = parse_arguments()
    if args.selftest:
        selftest(args)
        return
    if not args.bag or not os.path.isfile(args.bag):
        raise IOError("bag file does not exist: {0}".format(args.bag))

    primary_raw, primary_stamp_failures = read_bag_epochs(
        args.bag, args.primary_topic, args
    )
    truth_raw, truth_stamp_failures = read_bag_epochs(
        args.bag, args.truth_topic, args
    )
    primary, primary_rejections = filter_epochs(primary_raw, args)
    truth, truth_rejections = filter_epochs(truth_raw, args)
    if not truth:
        raise RuntimeError("no truth epochs passed the quality gates")
    if not primary:
        raise RuntimeError("no primary epochs passed the quality gates")

    origin_ecef, rotation, origin = resolve_origin(truth, args)
    primary_enu = convert_to_enu(primary, origin_ecef, rotation)
    truth_enu = convert_to_enu(truth, origin_ecef, rotation)

    pairs, dt_abs = pair_samples(primary_enu, truth_enu, args.max_dt)
    if len(pairs) < 10:
        raise RuntimeError(
            "only {0} paired epochs; check the two receivers' timing or "
            "widen --max-dt".format(len(pairs))
        )

    if args.static_window:
        window_start, window_duration = args.static_window
        first_time = pairs[0][0]
        lower = first_time + window_start
        upper = lower + window_duration
        indices = [
            k for k, pair in enumerate(pairs) if lower <= pair[0] <= upper
        ]
        if len(indices) < 10:
            raise RuntimeError(
                "--static-window contains only {0} paired epochs".format(
                    len(indices)
                )
            )
        segments = [(indices[0], indices[-1] + 1)]
        merged = segments
    else:
        segments, merged = detect_static_segments(pairs, args)

    if not segments:
        longest = 0.0
        for start, stop in merged:
            longest = max(longest, pairs[stop - 1][0] - pairs[start][0])
        raise RuntimeError(
            "no static segment >= {0} s found (longest candidate "
            "{1:.1f} s); relax --max-static-radius/--max-static-speed/"
            "--min-static-duration or force a window with "
            "--static-window START DURATION".format(
                args.min_static_duration, longest
            )
        )

    print("=== RTK reference static evaluation ===")
    print("bag            : {0}".format(args.bag))
    print(
        "primary topic  : {0} (read {1}, passed {2}, rejections {3}, "
        "bad stamps {4})".format(
            args.primary_topic,
            len(primary_raw),
            len(primary),
            primary_rejections,
            primary_stamp_failures,
        )
    )
    print(
        "truth topic    : {0} (read {1}, passed {2}, rejections {3}, "
        "bad stamps {4})".format(
            args.truth_topic,
            len(truth_raw),
            len(truth),
            truth_rejections,
            truth_stamp_failures,
        )
    )
    print(
        "ENU origin     : lat {0:.8f}, lon {1:.8f}, h {2:.3f} m".format(
            origin[0], origin[1], origin[2]
        )
    )
    if dt_abs:
        print(
            "paired epochs  : {0} of {1} truth epochs "
            "(median |dt| {2:.1f} ms, max {3:.1f} ms)".format(
                len(pairs),
                len(truth_enu),
                1000.0 * statistics.median(dt_abs),
                1000.0 * max(dt_abs),
            )
        )

    results = []
    for segment_id, (start, stop) in enumerate(segments):
        result = segment_stats(
            pairs, start, stop, args, args.tape_horizontal, args.tape_dz
        )
        result["segment"] = segment_id
        results.append(result)
        print_segment(result)

    if args.out_pairs:
        write_pair_csv(args.out_pairs, pairs, segments)
        print("\npair CSV written : {0}".format(args.out_pairs))
    if args.out_summary:
        write_summary_csv(args.out_summary, results)
        print("summary CSV written: {0}".format(args.out_summary))

    print("")
    print("Interpretation notes:")
    print(" - 2DRMS_h is each receiver's horizontal scatter about its own")
    print("   session mean; it bounds the receiver noise level.")
    print(" - 'RMS_h about mean' of the difference is the between-receiver")
    print("   consistency under static conditions.")
    print(" - Baseline differences vs the tape measurement (both < 1-2 cm)")
    print("   support the absence of large differential biases between the")
    print("   two receivers.")
    print(" - A shared (common-mode) bias is invisible to these metrics; it")
    print("   can only be excluded with a higher-grade reference (total")
    print("   station or post-processed kinematic solution).")


if __name__ == "__main__":
    try:
        main()
    except Exception as error:
        print("ERROR: {0}".format(error), file=sys.stderr)
        sys.exit(1)
