#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""generate_rtk_degradation_suite.py

One-command generator for the reviewer-requested RTK degradation experiment:
five conditions evaluated separately, each degraded condition repeated with
multiple seeds, plus a plain-text manifest describing every output bag.

Degradations are injected ONLY into the first --inject-until seconds of the
bag (default 400 s), because truth data for evaluation only exists there.
All later data passes through untouched.

Run once inside a sourced ROS1 workspace (rosbag is required):

  rosrun nmea_rtk_driver generate_rtk_degradation_suite.py --bag orchard.bag

With the defaults this produces, next to the input bag:

  orchard_normal.bag                 control (passthrough, no modification)
  orchard_dropout_s1..s3.bag         RTK dropout inside the windows
  orchard_bias_s1..s3.bag            slowly varying multipath-like bias
  orchard_jump_s1..s3.bag            meter-level instantaneous jumps
  orchard_false_fixed_s1..s3.bag     wrong position, fix_quality still 4
  orchard_degradation_experiment.txt manifest describing every bag
  one .json log per output bag       full parameter record (reproducibility)

Degradations are injected into the RTK stream consumed by the fusion
algorithm (--inject-prefix, default /rtk_truth/). The other receiver
(--reference-prefix, default /rtk/) is never modified and serves as the
independent evaluation reference. Topic names are preserved, so each
output bag replays through the original launch configuration.

Injection windows are bag-relative seconds inside [0, --inject-until].
They are either fixed for all bags (--windows "100-60,300-60", best for
cross-condition comparison) or drawn randomly per seed (--n-windows 3
--window-duration 60, best for the "multiple masking patterns / random
seeds" requirement).
"""

import argparse
import copy
import json
import math
import os
import random
import sys

SCENARIOS = ("normal", "dropout", "bias", "jump", "false_fixed")

SCENARIO_CN = {
    "normal": "正常对照（原样通过，不修改任何数据）",
    "dropout": "RTK丢星（窗口内的RTK观测整体移除，滤波器收不到更新）",
    "bias": "偏差测量（缓变多径式偏差，平滑进出包络，模拟多径偏置）",
    "jump": "异常跳变（窗口起点米级瞬时阶跃，窗口结束恢复）",
    "false_fixed": "假固定解（位置错误但fix_quality保持4，协方差偏紧）",
}

SCENARIO_EN = {
    "normal": "control run, passthrough",
    "dropout": "RTK dropout, observations removed inside the windows",
    "bias": "slowly varying multipath-like bias with smooth envelope",
    "jump": "instantaneous meter-level position jumps",
    "false_fixed": "wrong position while fix_quality stays 4 (fixed)",
}

DEFAULT_INJECT_PREFIX = "/rtk_truth/"
DEFAULT_REFERENCE_PREFIX = "/rtk/"


# --------------------------------------------------------------------------
# Window helpers
# --------------------------------------------------------------------------

def parse_windows(spec):
    """Parse 'start-dur,start-dur' into a list of (start, duration) floats."""
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
                "window start/duration must be positive: {0}".format(entry))
        windows.append((start, duration))
    return windows


def random_windows(usable, count, window_duration, gap, margin, rng):
    """Draw `count` non-overlapping windows with uniform random starts
    inside [margin, usable - margin]."""
    end_limit = usable - margin
    need = count * (window_duration + gap) - gap
    if need > end_limit - margin:
        raise ValueError(
            "{0} windows of {1} s with {2} s gap do not fit in {3:.1f} s "
            "of usable time (margins {4} s each); reduce "
            "--n-windows/--window-duration or enlarge --inject-until".format(
                count, window_duration, gap, end_limit - margin, margin))
    start = margin
    positions = []
    for _ in range(count):
        chosen = rng.uniform(start, end_limit - need)
        positions.append(chosen)
        start = chosen + window_duration + gap
        need -= window_duration + gap
    return [(position, window_duration) for position in positions]


def build_windows(args, scenario, seed, bag_duration):
    """Build the injection window list for one (scenario, seed) run.
    All windows lie inside [0, args.inject_until]."""
    if scenario == "normal":
        return [], "none"
    horizon = min(bag_duration, args.inject_until)
    rng = random.Random(seed)
    if args.windows is not None:
        specs = parse_windows(args.windows)
        for t0, duration in specs:
            if t0 + duration > horizon + 1e-9:
                raise ValueError(
                    "fixed window {0}-{1} exceeds --inject-until {2} s; "
                    "windows must lie inside [0, {2}]".format(
                        t0, t0 + duration, horizon))
        placement = "fixed (--windows)"
    else:
        specs = random_windows(
            horizon, args.n_windows, args.window_duration,
            args.gap, args.margin, rng)
        placement = "random per seed"
    amplitudes = {
        "bias": args.bias_amplitude,
        "jump": args.jump_amplitude,
        "false_fixed": args.false_fixed_amplitude,
    }
    amplitude = amplitudes.get(scenario, 0.0)  # dropout has no amplitude
    windows = []
    for t0, duration in specs:
        azimuth_deg = (args.azimuth_deg if args.azimuth_deg is not None
                       else rng.uniform(0.0, 360.0))
        azimuth = azimuth_deg * math.pi / 180.0
        windows.append({
            "t0": t0,
            "duration": duration,
            "amplitude": amplitude,
            "azimuth_deg": azimuth_deg,
            "dir_e": math.sin(azimuth),
            "dir_n": math.cos(azimuth),
            "phase": rng.uniform(0.0, 2.0 * math.pi),
        })
    return windows, placement


def window_offset(t_rel, win, scenario, args):
    """Horizontal (de, dn) offset in meters at bag-relative time t_rel."""
    t0, duration = win["t0"], win["duration"]
    if t_rel < t0 or t_rel >= t0 + duration:
        return 0.0, 0.0
    s = t_rel - t0
    if scenario == "bias":
        envelope = 0.5 * (1.0 - math.cos(2.0 * math.pi * s / duration))
        magnitude = win["amplitude"] * envelope
    else:  # jump / false_fixed: constant offset over the window
        magnitude = win["amplitude"]
    if args.ripple_amp > 0.0:
        ripple = args.ripple_amp * math.sin(
            2.0 * math.pi * args.ripple_hz * s + win["phase"])
        magnitude = max(0.0, magnitude + ripple)
    return win["dir_e"] * magnitude, win["dir_n"] * magnitude


def in_any_window(t_rel, windows):
    return any(w["t0"] <= t_rel < w["t0"] + w["duration"] for w in windows)


# --------------------------------------------------------------------------
# Geodetic and message helpers
# --------------------------------------------------------------------------

def meters_to_deg(de_m, dn_m, lat_deg, height_m):
    """Convert an ENU horizontal offset (m) to geodetic degree deltas."""
    semi_major = 6378137.0
    eccentricity_squared = 6.69437999014e-3
    latitude = lat_deg * math.pi / 180.0
    sin_latitude, cos_latitude = math.sin(latitude), math.cos(latitude)
    meridian = semi_major * (1.0 - eccentricity_squared) / (
        (1.0 - eccentricity_squared * sin_latitude * sin_latitude) ** 1.5)
    normal = semi_major / math.sqrt(
        1.0 - eccentricity_squared * sin_latitude * sin_latitude)
    d_lat = dn_m / (meridian + height_m) * (180.0 / math.pi)
    d_lon = de_m / ((normal + height_m) * cos_latitude) * (180.0 / math.pi)
    return d_lat, d_lon


def modify_epoch(message, de, dn, cov_scale):
    d_lat, d_lon = meters_to_deg(
        de, dn, message.latitude_deg, message.ellipsoid_height_m)
    message.latitude_deg += d_lat
    message.longitude_deg += d_lon
    _scale_covariance(
        message, "position_covariance_enu", cov_scale,
        ("gst_rms_m", "gst_semi_major_std_m", "gst_semi_minor_std_m",
         "gst_latitude_std_m", "gst_longitude_std_m", "gst_altitude_std_m"))


def modify_fix(message, de, dn, cov_scale):
    d_lat, d_lon = meters_to_deg(de, dn, message.latitude, message.altitude)
    message.latitude += d_lat
    message.longitude += d_lon
    _scale_covariance(message, "position_covariance", cov_scale, ())


def _scale_covariance(message, covariance_attr, cov_scale, std_attrs):
    if cov_scale == 1.0:
        return
    covariance = getattr(message, covariance_attr, None)
    if covariance:
        message.__setattr__(
            covariance_attr, [c * cov_scale for c in covariance])
    if std_attrs:
        factor = math.sqrt(cov_scale)
        for attr in std_attrs:
            if hasattr(message, attr):
                message.__setattr__(attr, getattr(message, attr) * factor)


def modify_message(message, de, dn, cov_scale):
    """Apply a horizontal offset to any supported message type."""
    message_type = getattr(message, "_type", "")
    if message_type == "nmea_rtk_driver/RtkEpoch":
        modify_epoch(message, de, dn, cov_scale)
    elif message_type == "sensor_msgs/NavSatFix":
        modify_fix(message, de, dn, cov_scale)
    # other injected topics (velocity, heading, status, time_reference)
    # pass through unchanged: the injected error is positional.


def classify_topic(topic, args):
    """Return 'inject', 'reference', 'raw' or 'other' for a topic name."""
    if args.raw_topic and topic == args.raw_topic:
        return "raw"
    if topic.startswith(args.inject_prefix):
        return "inject"
    if topic.startswith(args.reference_prefix):
        return "reference"
    return "other"


# --------------------------------------------------------------------------
# Single-bag injection
# --------------------------------------------------------------------------

def run_one(args, scenario, seed, windows, out_path):
    try:
        import rosbag
    except ImportError as error:
        raise RuntimeError(
            "rosbag is required; run inside a sourced ROS1 workspace"
        ) from error
    bag_path = os.path.abspath(args.bag)
    if not os.path.isfile(bag_path):
        raise IOError("bag file does not exist: {0}".format(bag_path))
    cov_scale = (
        args.false_fixed_cov_scale if scenario == "false_fixed" else 1.0)

    counts = {"written": 0, "modified": 0, "dropped": 0,
              "raw_skipped": 0, "raw_kept": 0, "passthrough": 0}
    with rosbag.Bag(bag_path, "r") as inbag:
        bag_start = inbag.get_start_time()
        bag_end = inbag.get_end_time()
        bag_duration = bag_end - bag_start
        topics = inbag.get_type_and_topic_info().topics
        raw_present = args.raw_topic in topics
        compression = None if args.no_compression else "lz4"
        with rosbag.Bag(out_path, "w", compression=compression) as outbag:
            for topic, message, record_time in inbag.read_messages():
                t_rel = record_time.to_sec() - bag_start
                kind = classify_topic(topic, args)
                if kind == "raw":
                    if args.drop_raw:
                        counts["raw_skipped"] += 1
                        continue
                    counts["raw_kept"] += 1
                if kind == "inject" and t_rel < args.inject_until:
                    if scenario == "dropout" and in_any_window(t_rel, windows):
                        counts["dropped"] += 1
                        continue
                    if scenario in ("bias", "jump", "false_fixed"):
                        de, dn = 0.0, 0.0
                        for win in windows:
                            d_e, d_n = window_offset(t_rel, win, scenario, args)
                            de += d_e
                            dn += d_n
                        if de != 0.0 or dn != 0.0:
                            message = copy.deepcopy(message)
                            modify_message(message, de, dn, cov_scale)
                            counts["modified"] += 1
                        else:
                            counts["passthrough"] += 1
                    else:
                        counts["passthrough"] += 1
                outbag.write(topic, message, record_time)
                counts["written"] += 1

    log = {
        "scenario": scenario,
        "seed": seed,
        "input_bag": bag_path,
        "output": out_path,
        "inject_prefix": args.inject_prefix,
        "reference_prefix": args.reference_prefix,
        "inject_until_s": args.inject_until,
        "bag_duration_s": bag_duration,
        "windows": [
            {"t0_s": w["t0"], "duration_s": w["duration"],
             "amplitude_m": w["amplitude"], "azimuth_deg": w["azimuth_deg"]}
            for w in windows],
        "cov_scale": cov_scale,
        "ripple_amp_m": args.ripple_amp,
        "ripple_hz": args.ripple_hz,
        "raw_topic_present": raw_present,
        "raw_topic_kept": not args.drop_raw,
        "counts": counts,
        "note": (
            "degradations are injected only into bag time < inject_until_s "
            "(truth for evaluation exists only there); later data passes "
            "through untouched. The reference prefix receiver is never "
            "modified and serves as the independent evaluation reference. "
            "Windows are relative to bag start."),
    }
    log_path = os.path.splitext(out_path)[0] + ".json"
    with open(log_path, "w") as stream:
        json.dump(log, stream, indent=2, sort_keys=True)

    return {
        "output": out_path,
        "json": log_path,
        "scenario": scenario,
        "seed": seed,
        "windows": windows,
        "cov_scale": cov_scale,
        "counts": counts,
        "bag_duration_s": bag_duration,
        "inject_until": args.inject_until,
    }


# --------------------------------------------------------------------------
# Job planning
# --------------------------------------------------------------------------

def parse_int_list(text):
    values = []
    for entry in text.split(","):
        entry = entry.strip()
        if entry:
            values.append(int(entry))
    if not values:
        raise ValueError("empty list: {0}".format(text))
    return values


def parse_scenario_list(text):
    if text.strip().lower() == "all":
        return list(SCENARIOS)
    scenarios = []
    for entry in text.split(","):
        entry = entry.strip().lower()
        if not entry:
            continue
        if entry not in SCENARIOS:
            raise ValueError("unknown scenario: {0}".format(entry))
        scenarios.append(entry)
    if not scenarios:
        raise ValueError("empty scenario list")
    return scenarios


def plan_jobs(scenarios, seeds):
    """(scenario, seed) jobs; the normal control is produced once."""
    jobs = []
    for scenario in scenarios:
        if scenario == "normal":
            jobs.append((scenario, seeds[0]))
        else:
            jobs.extend((scenario, seed) for seed in seeds)
    return jobs


def output_name(args, scenario, seed):
    stem = os.path.splitext(os.path.basename(os.path.abspath(args.bag)))[0]
    if scenario == "normal":
        suffix = "normal"
    else:
        suffix = "{0}_s{1}".format(scenario, seed)
    return os.path.join(args.out_dir, "{0}_{1}.bag".format(stem, suffix))


# --------------------------------------------------------------------------
# Manifest (txt description of every output bag)
# --------------------------------------------------------------------------

def format_entry(index, total, entry):
    lines = []
    lines.append("=" * 76)
    lines.append("[{0}/{1}] {2}".format(
        index, total, os.path.basename(entry["output"])))
    lines.append("-" * 76)
    scenario = entry["scenario"]
    lines.append("scenario   : {0}".format(scenario))
    lines.append("  english  : {0}".format(SCENARIO_EN[scenario]))
    lines.append("  中文说明 : {0}".format(SCENARIO_CN[scenario]))
    lines.append("seed       : {0}".format(
        "- (control)" if scenario == "normal" else entry["seed"]))
    lines.append("inject limit: bag time < {0:.1f} s (later data untouched, "
                 "no truth for evaluation)".format(entry["inject_until"]))
    if entry["windows"]:
        lines.append("windows (bag-relative seconds):")
        for k, w in enumerate(entry["windows"], 1):
            if scenario == "dropout":
                lines.append(
                    "  #{0} t0={1:8.1f} s  duration={2:6.1f} s  "
                    "azimuth={3:6.1f} deg".format(
                        k, w["t0"], w["duration"], w["azimuth_deg"]))
            else:
                lines.append(
                    "  #{0} t0={1:8.1f} s  duration={2:6.1f} s  "
                    "amplitude={3:5.2f} m  azimuth={4:6.1f} deg".format(
                        k, w["t0"], w["duration"], w["amplitude"],
                        w["azimuth_deg"]))
    else:
        lines.append("windows    : none")
    lines.append("cov scale  : {0}".format(entry["cov_scale"]))
    if scenario == "normal":
        lines.append("injection  : passthrough, every message copied unchanged")
    elif scenario == "dropout":
        lines.append("injection  : all RTK messages inside the windows are")
        lines.append("             removed; the filter receives no RTK update")
    elif scenario == "bias":
        lines.append("injection  : horizontal position offset with a smooth")
        lines.append("             raised-cosine envelope (0 -> amplitude -> 0),")
        lines.append("             simulating slowly varying multipath bias")
    elif scenario == "jump":
        lines.append("injection  : instantaneous horizontal step at window")
        lines.append("             start, removed at window end")
    elif scenario == "false_fixed":
        lines.append("injection  : horizontal position offset while")
        lines.append("             fix_quality remains 4 (RTK fixed); the")
        lines.append("             reported covariance is scaled by cov_scale")
        lines.append("             (simulates an overconfident false fix)")
    counts = entry["counts"]
    lines.append("counts     : written {0}, modified {1}, dropped {2}, "
                 "raw kept {3}, raw skipped {4}".format(
                     counts["written"], counts["modified"],
                     counts["dropped"], counts["raw_kept"],
                     counts["raw_skipped"]))
    lines.append("evaluation : run through the fusion pipeline; compare")
    lines.append("             against the untouched {0} receiver".format(
        entry["reference"]))
    lines.append("json log   : {0}".format(os.path.basename(entry["json"])))
    return "\n".join(lines)


def write_manifest(path, entries, args):
    with open(path, "w", encoding="utf-8") as stream:
        stream.write("RTK DEGRADATION EXPERIMENT - MANIFEST\n")
        stream.write("=====================================\n\n")
        stream.write("generated by : generate_rtk_degradation_suite.py\n")
        stream.write("command      : {0}\n\n".format(" ".join(sys.argv)))
        stream.write("input bag    : {0}\n".format(os.path.abspath(args.bag)))
        stream.write("output dir   : {0}\n".format(args.out_dir))
        stream.write("inject prefix: {0}  (RTK stream fed to the fusion\n"
                     .format(args.inject_prefix))
        stream.write("               algorithm; degradations injected here)\n")
        stream.write("reference    : {0}  (untouched; independent evaluation\n"
                     .format(args.reference_prefix))
        stream.write("               reference)\n")
        stream.write("inject until : first {0:.1f} s of the bag; later data "
                     "passes through\n".format(args.inject_until))
        stream.write("               untouched (no truth available there for "
                     "evaluation)\n")
        stream.write("raw NMEA     : preserved unchanged (diagnostic only; "
                     "the fusion\n")
        stream.write("               filter does not consume it); use "
                     "--drop-raw to\n")
        stream.write("               remove it\n\n")
        stream.write("protocol\n")
        stream.write("--------\n")
        stream.write("Five RTK degradation conditions are evaluated "
                     "separately:\n\n")
        for scenario in SCENARIOS:
            stream.write("  {0:<12} {1}\n".format(
                scenario, SCENARIO_EN[scenario]))
        stream.write("\nEach degraded condition is repeated with every seed "
                     "in --seeds,\nwhich yields different window placements "
                     "(or, with fixed --windows,\ndifferent injection "
                     "azimuths), satisfying the multiple-masking-patterns /\n"
                     "random-seeds requirement. All windows lie inside "
                     "[0, inject until].\n\n")
        stream.write("reproducibility\n")
        stream.write("---------------\n")
        stream.write("All window times, amplitudes, azimuths and covariance "
                     "scales are\nrecorded below and in the per-bag .json "
                     "logs. Topic names in the\noutput bags are preserved, so "
                     "each bag replays through the original\nlaunch "
                     "configuration.\n\n")
        for index, entry in enumerate(entries, 1):
            stream.write(format_entry(index, len(entries), entry))
            stream.write("\n\n")
        stream.write("=" * 76 + "\n")
        stream.write("REVIEWER REQUIREMENT MAPPING\n")
        stream.write("=" * 76 + "\n")
        stream.write("- separate evaluation of normal input / dropout / biased\n"
                     "  measurements / abnormal jumps / false-fixed solutions\n"
                     "  -> one bag series per condition\n")
        stream.write("- multiple masking patterns or random seeds\n"
                     "  -> each degraded condition generated with seeds "
                     "{0}\n".format(parse_int_list(args.seeds)))
        stream.write("- full parameter disclosure (reproducibility)\n"
                     "  -> this manifest plus the per-bag .json logs\n")
        stream.write("- reference-trajectory reliability\n"
                     "  -> the untouched {0} receiver serves as the\n"
                     "     independent reference for every run\n".format(
                         args.reference_prefix))


# --------------------------------------------------------------------------
# Main
# --------------------------------------------------------------------------

def parse_arguments():
    parser = argparse.ArgumentParser(
        description=(
            "Generate the full RTK degradation experiment suite (five "
            "conditions x multiple seeds) plus a manifest txt from one "
            "input bag, in a single run. Injection is limited to the "
            "first --inject-until seconds of the bag."))
    parser.add_argument("--bag", default="", help="input ROS1 bag")
    parser.add_argument("--out-dir", default=None,
                        help="output directory (default: input bag directory)")
    parser.add_argument("--scenarios", default="all",
                        help="comma-separated scenarios or 'all' "
                             "(normal,dropout,bias,jump,false_fixed)")
    parser.add_argument("--seeds", default="1,2,3",
                        help="comma-separated seeds for the degraded scenarios")
    parser.add_argument("--inject-until", type=float, default=400.0,
                        help="only bag time < this value (s) is modified; "
                             "later data passes through untouched "
                             "(default: 400)")
    parser.add_argument("--windows", default=None,
                        help="fixed windows 'start-dur,...' in bag-relative "
                             "seconds inside [0, inject-until], identical "
                             "for every scenario and seed")
    parser.add_argument("--n-windows", type=int, default=3,
                        help="number of randomly placed windows per run")
    parser.add_argument("--window-duration", type=float, default=60.0,
                        help="window duration for random windows (s)")
    parser.add_argument("--gap", type=float, default=20.0,
                        help="minimum gap between random windows (s)")
    parser.add_argument("--margin", type=float, default=10.0,
                        help="margin at 0 and inject-until for random "
                             "windows (s)")
    parser.add_argument("--bias-amplitude", type=float, default=1.0,
                        help="bias scenario amplitude (m)")
    parser.add_argument("--jump-amplitude", type=float, default=2.0,
                        help="jump scenario amplitude (m)")
    parser.add_argument("--false-fixed-amplitude", type=float, default=1.5,
                        help="false_fixed scenario amplitude (m)")
    parser.add_argument("--false-fixed-cov-scale", type=float, default=0.25,
                        help="covariance scale for false_fixed "
                             "(0.25 = overconfident)")
    parser.add_argument("--azimuth-deg", type=float, default=None,
                        help="fixed injection azimuth in degrees; random per "
                             "window when omitted")
    parser.add_argument("--ripple-amp", type=float, default=0.0,
                        help="sinusoidal ripple amplitude on the bias (m)")
    parser.add_argument("--ripple-hz", type=float, default=0.05,
                        help="ripple frequency (Hz)")
    parser.add_argument("--inject-prefix", default=DEFAULT_INJECT_PREFIX,
                        help="topic prefix of the RTK stream fed to the "
                             "fusion algorithm (default: /rtk_truth/)")
    parser.add_argument("--reference-prefix", default=DEFAULT_REFERENCE_PREFIX,
                        help="topic prefix of the untouched reference "
                             "receiver (default: /rtk/)")
    parser.add_argument("--raw-topic", default="/rtk_truth/raw_sentence",
                        help="raw sentence topic of the injected receiver "
                             "(kept unchanged by default)")
    parser.add_argument("--drop-raw", action="store_true",
                        help="drop the injected receiver's raw sentence "
                             "topic from the output bags")
    parser.add_argument("--no-compression", action="store_true",
                        help="write output bags without lz4 compression")
    return parser.parse_args()


def main():
    args = parse_arguments()
    if not args.bag:
        raise IOError("--bag is required")
    bag_path = os.path.abspath(args.bag)
    if not os.path.isfile(bag_path):
        raise IOError("bag file does not exist: {0}".format(bag_path))
    if args.out_dir is None:
        args.out_dir = os.path.dirname(bag_path)
    os.makedirs(args.out_dir, exist_ok=True)

    seeds = parse_int_list(args.seeds)
    scenarios = parse_scenario_list(args.scenarios)
    jobs = plan_jobs(scenarios, seeds)

    try:
        import rosbag
    except ImportError:
        raise RuntimeError(
            "rosbag is required; run inside a sourced ROS1 workspace")
    with rosbag.Bag(bag_path, "r") as probe:
        bag_duration = probe.get_end_time() - probe.get_start_time()
    horizon = min(bag_duration, args.inject_until)
    print("bag duration : {0:.1f} s".format(bag_duration))
    print("inject until: {0:.1f} s (degradations only here)".format(horizon))

    entries = []
    for index, (scenario, seed) in enumerate(jobs, 1):
        out_path = output_name(args, scenario, seed)
        windows, placement = build_windows(args, scenario, seed, bag_duration)
        print("[{0}/{1}] scenario={2:<12} seed={3:<3} -> {4}".format(
            index, len(jobs), scenario, seed, os.path.basename(out_path)))
        print("         windows: {0}".format(
            "none" if not windows else
            ", ".join("{0:.1f}-{1:.1f}s".format(w["t0"], w["duration"])
                      for w in windows)))
        entry = run_one(args, scenario, seed, windows, out_path)
        entry["placement"] = placement
        entry["reference"] = args.reference_prefix
        entries.append(entry)
        print("         counts : written {0}, modified {1}, dropped {2}, "
              "raw kept {3}, raw skipped {4}".format(
                  entry["counts"]["written"], entry["counts"]["modified"],
                  entry["counts"]["dropped"], entry["counts"]["raw_kept"],
                  entry["counts"]["raw_skipped"]))

    stem = os.path.splitext(os.path.basename(bag_path))[0]
    manifest_path = os.path.join(
        args.out_dir, "{0}_degradation_experiment.txt".format(stem))
    write_manifest(manifest_path, entries, args)

    print("")
    print("=== suite complete ===")
    print("input bag  : {0} ({1:.1f} s)".format(bag_path, bag_duration))
    print("inject     : {0} (degraded, only first {1:.1f} s)".format(
        args.inject_prefix, horizon))
    print("reference  : {0} (untouched)".format(args.reference_prefix))
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
