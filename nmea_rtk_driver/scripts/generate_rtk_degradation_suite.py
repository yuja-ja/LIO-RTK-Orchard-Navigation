#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Generate RTK degradation bags while retaining a clean RTK truth stream.

For every input ``/rtk_truth/<suffix>`` topic, the output bag keeps the
original topic and also writes a duplicate as ``/rtk/<suffix>``.  Only the
new ``/rtk/*`` copy is degraded.  The clean ``/rtk_truth/*`` topics are never
modified and can therefore be used as the independent evaluation reference.

By default, degradation windows cover 30 percent of the complete bag.  The
number of windows is determined by dividing the target degradation time by
60 seconds (ceiling), with each window being 60 seconds except the last
which takes the remainder.  The old ``--inject-until`` option is accepted
for command-line compatibility but is intentionally ignored.
"""

import argparse
import copy
import json
import math
import os
import random
import sys


SCENARIOS = ("normal", "dropout", "bias", "jump", "false_fixed")
SCENARIO_EN = {
    "normal": "clean control; /rtk is an unchanged duplicate",
    "dropout": "RTK observations removed from /rtk inside the windows",
    "bias": "slowly varying horizontal multipath-like position bias",
    "jump": "instantaneous meter-level horizontal position jumps",
    "false_fixed": "wrong position while fix_quality remains fixed",
}

RTK_SUFFIXES = (
    "epoch",
    "fix",
    "heading",
    "raw_sentence",
    "status",
    "time_reference",
)

DEFAULT_TRUTH_PREFIX = "/rtk_truth/"
DEFAULT_FUSION_PREFIX = "/rtk/"


def normalize_prefix(prefix):
    """Return a ROS topic prefix with one leading and trailing slash."""
    prefix = prefix.strip()
    if not prefix.startswith("/"):
        prefix = "/" + prefix
    if not prefix.endswith("/"):
        prefix += "/"
    return prefix


def parse_windows(spec):
    """Parse ``start-duration,start-duration`` into ``(start, duration)``."""
    windows = []
    for entry in spec.split(","):
        entry = entry.strip()
        if not entry:
            continue
        parts = entry.split("-")
        if len(parts) != 2:
            raise ValueError(
                "window entry must be start-duration: {0}".format(entry))
        start, duration = float(parts[0]), float(parts[1])
        if start < 0.0 or duration <= 0.0:
            raise ValueError(
                "window start/duration must be non-negative/positive: {0}"
                .format(entry))
        windows.append((start, duration))
    if not windows:
        raise ValueError("--windows did not contain a valid window")
    return windows


def random_windows(bag_duration, gap, margin, fraction, rng):
    """Generate non-overlapping windows whose total duration is ``fraction``.

    Window count is determined by dividing the target duration by 60 s
    (ceiling).  Durations are as close to 60 s as possible; the last window
    takes the remainder.  Random slack is then distributed among the leading,
    inter-window and trailing gaps.
    """
    if bag_duration <= 0.0:
        raise ValueError("bag duration must be positive")
    if not (0.0 < fraction < 1.0):
        raise ValueError("degradation fraction must be between 0 and 1")
    if gap < 0.0 or margin < 0.0:
        raise ValueError("gap and margin must be non-negative")

    target = fraction * bag_duration
    if target <= 0.0:
        raise ValueError("target degradation time must be positive")

    # New logic: window count = ceil(target / 60)
    n_windows = math.ceil(target / 60.0)
    durations = []
    remaining = target
    for _ in range(n_windows - 1):
        durations.append(60.0)
        remaining -= 60.0
    durations.append(remaining)

    # Feasibility check
    required = sum(durations) + max(0, n_windows - 1) * gap + 2.0 * margin
    if required > bag_duration + 1e-9:
        raise ValueError(
            "cannot cover {0:.3f} s ({1:.1f}% of a {2:.3f} s bag) with "
            "{3} windows (~60 s each) and total duration {4:.3f} s, "
            "gap={5:.1f} s and margin={6:.1f} s; "
            "reduce --gap/--margin or use --windows"
            .format(target, 100.0 * fraction, bag_duration,
                    n_windows, sum(durations), gap, margin))

    # Distribute slack randomly among gaps (same mechanism as before)
    baseline = sum(durations) + max(0, n_windows - 1) * gap + 2.0 * margin
    slack = max(0.0, bag_duration - baseline)
    cuts = sorted(rng.uniform(0.0, slack) for _ in range(n_windows))
    free_gaps = []
    previous = 0.0
    for cut in cuts:
        free_gaps.append(cut - previous)
        previous = cut
    free_gaps.append(slack - previous)

    windows = []
    start = margin + free_gaps[0]
    for index, duration in enumerate(durations):
        windows.append((start, duration))
        if index + 1 < n_windows:
            start += duration + gap + free_gaps[index + 1]
    return windows, target


def build_windows(args, scenario, seed, bag_duration):
    """Build windows for one scenario/seed over the complete bag."""
    if scenario == "normal":
        return [], "none", 0.0

    rng = random.Random(seed)
    if args.windows is not None:
        specs = parse_windows(args.windows)
        previous_end = -1.0
        for start, duration in specs:
            if start + duration > bag_duration + 1e-9:
                raise ValueError(
                    "fixed window {0}-{1} exceeds complete bag duration "
                    "{2} s".format(start, start + duration, bag_duration))
            if start < previous_end - 1e-9:
                raise ValueError("fixed windows overlap")
            previous_end = start + duration
        target = sum(duration for _, duration in specs)
        placement = "fixed (--windows)"
    else:
        specs, target = random_windows(
            bag_duration, args.gap, args.margin,
            args.degradation_fraction, rng)
        placement = "random per seed; complete-bag 30% target, 60-s chunks"

    amplitudes = {
        "bias": args.bias_amplitude,
        "jump": args.jump_amplitude,
        "false_fixed": args.false_fixed_amplitude,
    }
    amplitude = amplitudes.get(scenario, 0.0)
    windows = []
    for start, duration in specs:
        azimuth_deg = (args.azimuth_deg if args.azimuth_deg is not None
                       else rng.uniform(0.0, 360.0))
        azimuth = azimuth_deg * math.pi / 180.0
        windows.append({
            "t0": start,
            "duration": duration,
            "amplitude": amplitude,
            "azimuth_deg": azimuth_deg,
            "dir_e": math.sin(azimuth),
            "dir_n": math.cos(azimuth),
            "phase": rng.uniform(0.0, 2.0 * math.pi),
        })
    return windows, placement, target


def window_offset(t_rel, window, scenario, args):
    """Return horizontal ENU offset in metres at bag-relative time."""
    t0 = window["t0"]
    duration = window["duration"]
    if t_rel < t0 or t_rel >= t0 + duration:
        return 0.0, 0.0
    elapsed = t_rel - t0
    if scenario == "bias":
        envelope = 0.5 * (1.0 - math.cos(2.0 * math.pi * elapsed /
                                           duration))
        magnitude = window["amplitude"] * envelope
    else:
        magnitude = window["amplitude"]
    if args.ripple_amp > 0.0:
        ripple = args.ripple_amp * math.sin(
            2.0 * math.pi * args.ripple_hz * elapsed + window["phase"])
        magnitude = max(0.0, magnitude + ripple)
    return window["dir_e"] * magnitude, window["dir_n"] * magnitude


def in_any_window(t_rel, windows):
    return any(w["t0"] <= t_rel < w["t0"] + w["duration"]
               for w in windows)


def meters_to_deg(de_m, dn_m, lat_deg, height_m):
    """Convert a local ENU horizontal offset to WGS84 degree deltas."""
    semi_major = 6378137.0
    eccentricity_squared = 6.69437999014e-3
    latitude = lat_deg * math.pi / 180.0
    sin_latitude = math.sin(latitude)
    cos_latitude = math.cos(latitude)
    meridian = semi_major * (1.0 - eccentricity_squared) / (
        (1.0 - eccentricity_squared * sin_latitude ** 2) ** 1.5)
    normal = semi_major / math.sqrt(
        1.0 - eccentricity_squared * sin_latitude ** 2)
    d_lat = dn_m / (meridian + height_m) * 180.0 / math.pi
    d_lon = de_m / ((normal + height_m) * cos_latitude) * 180.0 / math.pi
    return d_lat, d_lon


def _scale_covariance(message, covariance_attr, scale, std_attrs):
    if scale == 1.0:
        return
    covariance = getattr(message, covariance_attr, None)
    if covariance is not None:
        setattr(message, covariance_attr, [value * scale for value in covariance])
    factor = math.sqrt(scale)
    for attr in std_attrs:
        if hasattr(message, attr):
            value = getattr(message, attr)
            if isinstance(value, (int, float)) and math.isfinite(value):
                setattr(message, attr, value * factor)


def modify_epoch(message, de, dn, covariance_scale):
    lat = getattr(message, "latitude_deg", float("nan"))
    lon = getattr(message, "longitude_deg", float("nan"))
    height = getattr(message, "ellipsoid_height_m", float("nan"))
    if not math.isfinite(height):
        height = getattr(message, "msl_height_m", 0.0)
    if not (math.isfinite(lat) and math.isfinite(lon)):
        return False
    d_lat, d_lon = meters_to_deg(de, dn, lat, height if math.isfinite(height) else 0.0)
    message.latitude_deg += d_lat
    message.longitude_deg += d_lon
    _scale_covariance(
        message, "position_covariance_enu", covariance_scale,
        ("gst_rms_m", "gst_semi_major_std_m", "gst_semi_minor_std_m",
         "gst_latitude_std_m", "gst_longitude_std_m", "gst_altitude_std_m"))
    return True


def modify_fix(message, de, dn, covariance_scale):
    lat = getattr(message, "latitude", float("nan"))
    lon = getattr(message, "longitude", float("nan"))
    height = getattr(message, "altitude", 0.0)
    if not (math.isfinite(lat) and math.isfinite(lon)):
        return False
    d_lat, d_lon = meters_to_deg(de, dn, lat,
                                 height if math.isfinite(height) else 0.0)
    message.latitude += d_lat
    message.longitude += d_lon
    _scale_covariance(message, "position_covariance", covariance_scale, ())
    return True


def modify_message(message, de, dn, covariance_scale):
    """Modify a supported position message; return whether it was changed."""
    message_type = getattr(message, "_type", "")
    if message_type == "nmea_rtk_driver/RtkEpoch":
        return modify_epoch(message, de, dn, covariance_scale)
    if message_type == "sensor_msgs/NavSatFix":
        return modify_fix(message, de, dn, covariance_scale)
    return False


def topic_alias(topic, args):
    """Map one clean truth topic to its generated fusion topic."""
    for suffix in RTK_SUFFIXES:
        if topic == args.reference_prefix + suffix:
            return args.inject_prefix + suffix, suffix
    return None, None


def run_one(args, scenario, seed, windows, out_path, bag_duration,
            target_window_seconds):
    try:
        import rosbag
    except ImportError as error:
        raise RuntimeError(
            "rosbag is required; run inside a sourced ROS1 workspace") from error

    bag_path = os.path.abspath(args.bag)
    covariance_scale = (args.false_fixed_cov_scale
                        if scenario == "false_fixed" else 1.0)
    counts = {
        "written": 0,
        "truth_written": 0,
        "fusion_written": 0,
        "existing_fusion_skipped": 0,
        "modified": 0,
        "dropped": 0,
        "passthrough": 0,
        "raw_alias_skipped": 0,
        "truth_topics_missing": 0,
    }

    with rosbag.Bag(bag_path, "r") as inbag:
        bag_start = inbag.get_start_time()
        topic_info = inbag.get_type_and_topic_info().topics
        expected_topics = [args.reference_prefix + suffix
                           for suffix in RTK_SUFFIXES]
        counts["truth_topics_missing"] = sum(
            1 for topic in expected_topics if topic not in topic_info)
        compression = None if args.no_compression else "lz4"
        existing_fusion_topics = set(
            args.inject_prefix + suffix for suffix in RTK_SUFFIXES)
        with rosbag.Bag(out_path, "w", compression=compression) as outbag:
            for topic, message, record_time in inbag.read_messages():
                t_rel = record_time.to_sec() - bag_start

                # If the input already contains /rtk/*, do not mix that
                # receiver with the generated alias.  The clean
                # /rtk_truth/* stream is the sole source for /rtk/* in every
                # output bag.
                if topic in existing_fusion_topics:
                    counts["existing_fusion_skipped"] += 1
                    continue

                # Every original message remains in the output bag.
                outbag.write(topic, message, record_time)
                counts["written"] += 1
                if topic.startswith(args.reference_prefix):
                    counts["truth_written"] += 1

                alias, suffix = topic_alias(topic, args)
                if alias is None:
                    continue

                if suffix == "raw_sentence" and args.drop_raw:
                    counts["raw_alias_skipped"] += 1
                    continue

                active = scenario != "normal" and in_any_window(t_rel, windows)
                if scenario == "dropout" and active:
                    counts["dropped"] += 1
                    continue

                alias_message = message
                if active and scenario in ("bias", "jump", "false_fixed"):
                    de = dn = 0.0
                    for window in windows:
                        east, north = window_offset(
                            t_rel, window, scenario, args)
                        de += east
                        dn += north
                    if de != 0.0 or dn != 0.0:
                        alias_message = copy.deepcopy(message)
                        if modify_message(alias_message, de, dn,
                                          covariance_scale):
                            counts["modified"] += 1
                        else:
                            counts["passthrough"] += 1
                    else:
                        counts["passthrough"] += 1
                else:
                    counts["passthrough"] += 1

                # Same message timestamp and contents as the source unless
                # the generated fusion copy was deliberately degraded.
                outbag.write(alias, alias_message, record_time)
                counts["written"] += 1
                counts["fusion_written"] += 1

    coverage = sum(window["duration"] for window in windows)
    log = {
        "scenario": scenario,
        "seed": seed,
        "input_bag": bag_path,
        "output": os.path.abspath(out_path),
        "truth_prefix": args.reference_prefix,
        "fusion_prefix": args.inject_prefix,
        "copied_suffixes": list(RTK_SUFFIXES),
        "bag_duration_s": bag_duration,
        "degradation_window_seconds": coverage,
        "degradation_fraction": coverage / bag_duration if bag_duration else 0.0,
        "target_window_seconds": target_window_seconds,
        "windows": [
            {"t0_s": window["t0"],
             "duration_s": window["duration"],
             "amplitude_m": window["amplitude"],
             "azimuth_deg": window["azimuth_deg"]}
            for window in windows
        ],
        "covariance_scale": covariance_scale,
        "ripple_amp_m": args.ripple_amp,
        "ripple_hz": args.ripple_hz,
        "raw_alias_kept": not args.drop_raw,
        "counts": counts,
        "note": (
            "The original truth topics are always retained.  Only the "
            "generated fusion topics are degraded.  Windows use complete "
            "bag-relative time; there is no 400-second cutoff."),
    }
    log_path = os.path.splitext(out_path)[0] + ".json"
    with open(log_path, "w", encoding="utf-8") as stream:
        json.dump(log, stream, indent=2, sort_keys=True)

    return {
        "output": out_path,
        "json": log_path,
        "scenario": scenario,
        "seed": seed,
        "windows": windows,
        "target_window_seconds": target_window_seconds,
        "coverage_seconds": coverage,
        "coverage_fraction": coverage / bag_duration if bag_duration else 0.0,
        "covariance_scale": covariance_scale,
        "counts": counts,
        "bag_duration_s": bag_duration,
        "reference_prefix": args.reference_prefix,
        "fusion_prefix": args.inject_prefix,
    }


def parse_int_list(text):
    values = [int(item.strip()) for item in text.split(",") if item.strip()]
    if not values:
        raise ValueError("empty seed list")
    return values


def parse_scenario_list(text):
    if text.strip().lower() == "all":
        return list(SCENARIOS)
    scenarios = [item.strip().lower() for item in text.split(",")
                 if item.strip()]
    unknown = [item for item in scenarios if item not in SCENARIOS]
    if unknown:
        raise ValueError("unknown scenario(s): {0}".format(", ".join(unknown)))
    if not scenarios:
        raise ValueError("empty scenario list")
    return scenarios


def plan_jobs(scenarios, seeds):
    jobs = []
    for scenario in scenarios:
        if scenario == "normal":
            jobs.append((scenario, seeds[0]))
        else:
            jobs.extend((scenario, seed) for seed in seeds)
    return jobs


def output_name(args, scenario, seed):
    stem = os.path.splitext(os.path.basename(os.path.abspath(args.bag)))[0]
    suffix = "normal" if scenario == "normal" else "{0}_s{1}".format(
        scenario, seed)
    return os.path.join(args.out_dir, "{0}_{1}.bag".format(stem, suffix))


def format_entry(index, total, entry):
    lines = [
        "=" * 76,
        "[{0}/{1}] {2}".format(index, total,
                                os.path.basename(entry["output"])),
        "-" * 76,
        "scenario       : {0}".format(entry["scenario"]),
        "description    : {0}".format(SCENARIO_EN[entry["scenario"]]),
        "seed           : {0}".format(
            "(control)" if entry["scenario"] == "normal" else entry["seed"]),
        "bag duration   : {0:.3f} s".format(entry["bag_duration_s"]),
        "window coverage: {0:.3f} s ({1:.2f}%)".format(
            entry["coverage_seconds"], 100.0 * entry["coverage_fraction"]),
        "truth topics   : {0} (retained unchanged)".format(
            entry["reference_prefix"]),
        "fusion topics  : {0} (copy used by the algorithm)".format(
            entry["fusion_prefix"]),
    ]
    if entry["windows"]:
        lines.append("windows (bag-relative seconds):")
        for number, window in enumerate(entry["windows"], 1):
            lines.append(
                "  #{0} t0={1:8.3f} s duration={2:8.3f} s amplitude={3:6.2f} m "
                "azimuth={4:7.2f} deg".format(
                    number, window["t0"], window["duration"],
                    window["amplitude"], window["azimuth_deg"]))
    else:
        lines.append("windows        : none")
    lines.append("covariance scale: {0}".format(entry["covariance_scale"]))
    counts = entry["counts"]
    lines.append(
        "counts         : written {0}, truth {1}, fusion {2}, modified {3}, "
        "dropped {4}, raw aliases skipped {5}, existing fusion skipped {6}"
        .format(
            counts["written"], counts["truth_written"],
            counts["fusion_written"], counts["modified"],
            counts["dropped"], counts["raw_alias_skipped"],
            counts["existing_fusion_skipped"]))
    lines.append("json log       : {0}".format(os.path.basename(entry["json"])))
    return "\n".join(lines)


def write_manifest(path, entries, args, bag_duration):
    with open(path, "w", encoding="utf-8") as stream:
        stream.write("RTK DEGRADATION EXPERIMENT - MANIFEST\n")
        stream.write("=====================================\n\n")
        stream.write("generated by : generate_rtk_degradation_suite.py\n")
        stream.write("command      : {0}\n\n".format(" ".join(sys.argv)))
        stream.write("input bag    : {0}\n".format(os.path.abspath(args.bag)))
        stream.write("bag duration : {0:.3f} s\n".format(bag_duration))
        stream.write("truth prefix : {0} (retained unchanged)\n".format(
            args.reference_prefix))
        stream.write("fusion prefix: {0} (generated copy; degradation target)\n".format(
            args.inject_prefix))
        stream.write("copied topics : {0}\n".format(
            ", ".join(args.reference_prefix + suffix
                      + " -> " + args.inject_prefix + suffix
                      for suffix in RTK_SUFFIXES)))
        stream.write("window policy : complete bag; ~60 s chunks; "
                     "target {0:.1f}%\n\n".format(
                         100.0 * args.degradation_fraction))
        stream.write("scenarios\n---------\n")
        for scenario in SCENARIOS:
            stream.write("  {0:<12} {1}\n".format(
                scenario, SCENARIO_EN[scenario]))
        stream.write("\n")
        for index, entry in enumerate(entries, 1):
            stream.write(format_entry(index, len(entries), entry))
            stream.write("\n\n")
        stream.write("=" * 76 + "\n")
        stream.write("The /rtk_truth/* stream is the clean evaluation reference; "
                     "only /rtk/* is degraded.\n")


def parse_arguments():
    parser = argparse.ArgumentParser(
        description=(
            "Generate RTK degradation bags. Each output retains clean "
            "/rtk_truth/* and adds a degraded/clean /rtk/* copy."))
    parser.add_argument("--bag", default="", help="input ROS1 bag")
    parser.add_argument("--out-dir", default=None,
                        help="output directory (default: input bag directory)")
    parser.add_argument("--scenarios", default="all",
                        help="normal,dropout,bias,jump,false_fixed or all")
    parser.add_argument("--seeds", default="1,2,3",
                        help="comma-separated seeds for degraded scenarios")
    parser.add_argument("--windows", default=None,
                        help="fixed windows 'start-duration,...'; overrides random windows")
    parser.add_argument("--n-windows", type=int, default=3,
                        help="preferred number of random windows (default: 3)")
    parser.add_argument("--min-window-duration", type=float, default=60.0,
                        help="minimum random window duration in seconds")
    parser.add_argument("--max-window-duration", type=float, default=200.0,
                        help="maximum random window duration in seconds")
    parser.add_argument("--degradation-fraction", type=float, default=0.3,
                        help="total random-window coverage fraction (default: 0.3)")
    parser.add_argument("--window-duration", type=float, default=None,
                        help="legacy option: use one fixed duration for random windows")
    parser.add_argument("--gap", type=float, default=20.0,
                        help="minimum gap between random windows in seconds")
    parser.add_argument("--margin", type=float, default=10.0,
                        help="minimum leading/trailing margin in seconds")
    parser.add_argument("--bias-amplitude", type=float, default=1.0,
                        help="bias amplitude in metres")
    parser.add_argument("--jump-amplitude", type=float, default=2.0,
                        help="jump amplitude in metres")
    parser.add_argument("--false-fixed-amplitude", type=float, default=1.5,
                        help="false-fixed position amplitude in metres")
    parser.add_argument("--false-fixed-cov-scale", type=float, default=0.25,
                        help="covariance scale for false_fixed")
    parser.add_argument("--azimuth-deg", type=float, default=None,
                        help="fixed error azimuth; random per window by default")
    parser.add_argument("--ripple-amp", type=float, default=0.0,
                        help="sinusoidal ripple amplitude in metres")
    parser.add_argument("--ripple-hz", type=float, default=0.05,
                        help="sinusoidal ripple frequency in Hz")
    parser.add_argument("--inject-prefix", default=DEFAULT_FUSION_PREFIX,
                        help="generated fusion topic prefix (default: /rtk/)")
    parser.add_argument("--reference-prefix", default=DEFAULT_TRUTH_PREFIX,
                        help="clean source/reference prefix (default: /rtk_truth/)")
    parser.add_argument("--raw-topic", default=None,
                        help="deprecated compatibility option; raw alias is derived from prefixes")
    parser.add_argument("--drop-raw", action="store_true",
                        help="omit only generated /rtk/raw_sentence; truth raw topic remains")
    parser.add_argument("--no-compression", action="store_true",
                        help="write output bags without lz4 compression")
    parser.add_argument("--inject-until", type=float, default=None,
                        help="deprecated and ignored; degradation always covers the complete bag")
    return parser.parse_args()


def main():
    args = parse_arguments()
    if not args.bag:
        raise IOError("--bag is required")
    args.reference_prefix = normalize_prefix(args.reference_prefix)
    args.inject_prefix = normalize_prefix(args.inject_prefix)
    if args.reference_prefix == args.inject_prefix:
        raise ValueError("truth and fusion prefixes must be different")
    if args.inject_until is not None:
        print("WARNING: --inject-until is ignored; using the complete bag.")
    if args.n_windows <= 0:
        raise ValueError("--n-windows must be positive")
    if args.false_fixed_cov_scale <= 0.0:
        raise ValueError("--false-fixed-cov-scale must be positive")

    bag_path = os.path.abspath(args.bag)
    if not os.path.isfile(bag_path):
        raise IOError("bag file does not exist: {0}".format(bag_path))
    if args.out_dir is None:
        args.out_dir = os.path.dirname(bag_path)
    args.out_dir = os.path.abspath(args.out_dir)
    os.makedirs(args.out_dir, exist_ok=True)

    seeds = parse_int_list(args.seeds)
    scenarios = parse_scenario_list(args.scenarios)
    jobs = plan_jobs(scenarios, seeds)

    try:
        import rosbag
    except ImportError:
        raise RuntimeError("rosbag is required; source a ROS1 workspace first")
    with rosbag.Bag(bag_path, "r") as probe:
        bag_duration = probe.get_end_time() - probe.get_start_time()
        available_topics = probe.get_type_and_topic_info().topics
    missing_topics = [args.reference_prefix + suffix
                      for suffix in RTK_SUFFIXES
                      if args.reference_prefix + suffix not in available_topics]
    if missing_topics:
        raise ValueError(
            "input bag is missing required clean RTK topics: {0}".format(
                ", ".join(missing_topics)))

    print("bag duration : {0:.3f} s".format(bag_duration))
    print("degradation  : complete bag; random windows target {0:.1f}%".format(
        100.0 * args.degradation_fraction))
    print("topic copies : {0}* -> {1}*".format(
        args.reference_prefix, args.inject_prefix))

    entries = []
    for index, (scenario, seed) in enumerate(jobs, 1):
        windows, placement, target = build_windows(
            args, scenario, seed, bag_duration)
        out_path = output_name(args, scenario, seed)
        print("[{0}/{1}] scenario={2:<12} seed={3:<3} -> {4}".format(
            index, len(jobs), scenario, seed, os.path.basename(out_path)))
        print("         windows: {0}".format(
            "none" if not windows else ", ".join(
                "{0:.3f}-{1:.3f}s".format(window["t0"], window["duration"])
                for window in windows)))
        entry = run_one(args, scenario, seed, windows, out_path,
                        bag_duration, target)
        entry["placement"] = placement
        entries.append(entry)
        print("         coverage: {0:.3f} s ({1:.2f}%), written={2}, "
              "modified={3}, dropped={4}".format(
                  entry["coverage_seconds"],
                  100.0 * entry["coverage_fraction"],
                  entry["counts"]["written"],
                  entry["counts"]["modified"],
                  entry["counts"]["dropped"]))

    stem = os.path.splitext(os.path.basename(bag_path))[0]
    manifest_path = os.path.join(
        args.out_dir, "{0}_degradation_experiment.txt".format(stem))
    write_manifest(manifest_path, entries, args, bag_duration)

    print("\n=== suite complete ===")
    print("input bag  : {0}".format(bag_path))
    print("truth      : {0} (unchanged)".format(args.reference_prefix))
    print("fusion     : {0} (generated/degraded copy)".format(args.inject_prefix))
    print("manifest   : {0}".format(manifest_path))
    print("bags       :")
    for entry in entries:
        print("  {0}".format(os.path.basename(entry["output"])))


if __name__ == "__main__":
    try:
        main()
    except Exception as error:
        print("ERROR: {0}".format(error), file=sys.stderr)
        sys.exit(1)