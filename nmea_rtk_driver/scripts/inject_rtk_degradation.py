#!/usr/bin/env python3
"""Inject controlled RTK degradations into the RTK stream that is fused
into the algorithm.

Purpose
-------
Reviewer feedback requires the five RTK degradation conditions to be
evaluated separately (normal input, dropout, biased measurements, abnormal
jumps, false-fixed solutions) with multiple masking patterns or random
seeds. Real recordings cannot guarantee when and where each condition
occurs, so this tool injects each condition into clean field data: the
degradations are applied to the RTK topics consumed by the fusion filter
(by default everything under /rtk_truth/, the stream fed to the
algorithm), while the other receiver's topics (by default /rtk/) stay
untouched and serve as the independent evaluation reference.

Design
------
Both prefixes are configurable with --inject-prefix and
--reference-prefix, so the same tool works regardless of which receiver is
wired into the algorithm. For every injected epoch inside a degradation
window the position is offset by a scenario-specific error; the original
header stamps and record times are preserved, so the resulting bag
replays identically to the original except for the injected RTK data.

  normal       : passthrough (control run)
  dropout      : injected RTK epochs inside the window are removed entirely
  bias         : slowly varying horizontal bias (smooth in/out envelope,
                 optional sinusoidal ripple), covariance unchanged
  jump         : instantaneous meter-level step at window start, removed at
                 window end, covariance unchanged
  false_fixed  : meter-level position error while fix_quality remains 4
                 (RTK fixed) and the covariance stays normal (or is scaled
                 with --cov-scale, e.g. 0.25 for an overconfident receiver)

The raw sentence topic of the injected receiver is dropped from the output
by default (diagnostics only; the fusion filter must not consume it). Use
--keep-raw to retain it.

Windows are expressed in seconds relative to the bag start time. They can
be given explicitly (--windows 100-60,300-60) or drawn randomly
(--n-windows 3 --window-duration 60 --seed 1). Every run writes a JSON log
next to the output bag describing windows, amplitudes, azimuths and the
seed so the experiment is reproducible.

Run inside a sourced ROS1 workspace (rosbag is required):

  rosrun nmea_rtk_driver inject_rtk_degradation.py \
      --bag orchard.bag --scenario bias \
      --n-windows 3 --window-duration 60 --amplitude 1.0 --seed 1 \
      --output orchard_bias_s1.bag

Suggested protocol (one output bag per condition, repeated per seed):
  normal, dropout, bias, jump, false_fixed -- each with --seed 1,2,3.

A self-test of the pure (non-rosbag) logic is available:

  python3 inject_rtk_degradation.py --selftest
"""

import argparse
import copy
import json
import math
import os
import random
import sys

SCENARIOS = ("normal", "dropout", "bias", "jump", "false_fixed")
DEFAULT_INJECT_PREFIX = "/rtk_truth/"
DEFAULT_REFERENCE_PREFIX = "/rtk/"


def parse_windows(spec):
    """Parse 'start-dur,start-dur' into a list of (start, duration) floats."""
    windows = []
    for entry in spec.split(","):
        entry = entry.strip()
        if not entry:
            continue
        parts = entry.split("-")
        if len(parts) != 2:
            raise ValueError("window entry must be start-duration: {0}".format(entry))
        start = float(parts[0])
        duration = float(parts[1])
        if start < 0.0 or duration <= 0.0:
            raise ValueError("window start/duration must be positive: {0}".format(entry))
        windows.append((start, duration))
    return windows


def random_windows(bag_duration, count, window_duration, gap, margin, rng):
    """Draw `count` non-overlapping windows with uniform random starts."""
    end_limit = bag_duration - margin
    need = count * (window_duration + gap) - gap
    if need > end_limit - margin:
        raise ValueError(
            "{0} windows of {1} s with {2} s gap do not fit in "
            "{3:.1f} s of usable bag time (margins {4} s each); reduce "
            "--n-windows/--window-duration or the margins".format(
                count, window_duration, gap, end_limit - margin, margin
            )
        )
    start = margin
    positions = []
    for _ in range(count):
        upper = end_limit - need
        chosen = rng.uniform(start, upper)
        positions.append(chosen)
        start = chosen + window_duration + gap
        need -= window_duration + gap
    return [(position, window_duration) for position in positions]


def meters_to_deg(de_m, dn_m, lat_deg, height_m):
    """Convert an ENU horizontal offset (m) to geodetic degree deltas."""
    semi_major = 6378137.0
    eccentricity_squared = 6.69437999014e-3
    latitude = lat_deg * math.pi / 180.0
    sin_latitude = math.sin(latitude)
    cos_latitude = math.cos(latitude)
    meridian = semi_major * (1.0 - eccentricity_squared) / (
        (1.0 - eccentricity_squared * sin_latitude * sin_latitude) ** 1.5
    )
    normal = semi_major / math.sqrt(
        1.0 - eccentricity_squared * sin_latitude * sin_latitude
    )
    d_lat = dn_m / (meridian + height_m) * (180.0 / math.pi)
    d_lon = de_m / ((normal + height_m) * cos_latitude) * (180.0 / math.pi)
    return d_lat, d_lon


def window_offset(t_rel, win, scenario, args):
    """Horizontal (de, dn) offset in meters at bag-relative time t_rel.

    Returns (0.0, 0.0) outside the window.
    """
    t0, duration = win["t0"], win["duration"]
    if t_rel < t0 or t_rel >= t0 + duration:
        return 0.0, 0.0
    s = t_rel - t0
    if scenario == "bias":
        envelope = 0.5 * (1.0 - math.cos(2.0 * math.pi * s / duration))
        magnitude = win["amplitude"] * envelope
    else:  # jump and false_fixed are constant offsets over the window
        magnitude = win["amplitude"]
    if args.ripple_amp > 0.0:
        ripple = args.ripple_amp * math.sin(
            2.0 * math.pi * args.ripple_hz * s + win["phase"]
        )
        magnitude = max(0.0, magnitude + ripple)
    return win["dir_e"] * magnitude, win["dir_n"] * magnitude


def build_windows(windows_spec, args, bag_duration, rng):
    """Turn raw (start, duration) pairs into injection windows with random
    azimuth, phase and amplitude attributes."""
    windows = []
    for t0, duration in windows_spec:
        if args.azimuth_deg is None:
            azimuth_deg = rng.uniform(0.0, 360.0)
        else:
            azimuth_deg = args.azimuth_deg
        azimuth = azimuth_deg * math.pi / 180.0
        windows.append({
            "t0": t0,
            "duration": duration,
            "amplitude": args.amplitude,
            "azimuth_deg": azimuth_deg,
            "dir_e": math.sin(azimuth),
            "dir_n": math.cos(azimuth),
            "phase": rng.uniform(0.0, 2.0 * math.pi),
        })
    return windows


def classify_topic(topic, args):
    """Return 'inject', 'reference', 'raw' or 'other' for a topic name.

    The raw topic check comes first so that the injected receiver's raw
    sentence topic is not treated as an injectable observation.
    """
    if args.raw_topic and topic == args.raw_topic:
        return "raw"
    if topic.startswith(args.inject_prefix):
        return "inject"
    if topic.startswith(args.reference_prefix):
        return "reference"
    return "other"


def modify_epoch(message, de, dn, cov_scale):
    d_lat, d_lon = meters_to_deg(
        de, dn, message.latitude_deg, message.ellipsoid_height_m
    )
    message.latitude_deg += d_lat
    message.longitude_deg += d_lon
    _scale_covariance(
        message,
        "position_covariance_enu",
        cov_scale,
        ("gst_rms_m", "gst_semi_major_std_m", "gst_semi_minor_std_m",
         "gst_latitude_std_m", "gst_longitude_std_m", "gst_altitude_std_m"),
    )


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
            covariance_attr, [c * cov_scale for c in covariance]
        )
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
    # other primary messages (velocity, heading, status, time_reference)
    # pass through unchanged: the injected error is positional.


def in_any_window(t_rel, windows):
    return any(
        window["t0"] <= t_rel < window["t0"] + window["duration"]
        for window in windows
    )


def run(args):
    try:
        import rosbag
    except ImportError as error:
        raise RuntimeError(
            "rosbag is required; run inside a sourced ROS1 workspace"
        ) from error
    if not os.path.isfile(args.bag):
        raise IOError("bag file does not exist: {0}".format(args.bag))
    if args.scenario not in SCENARIOS:
        raise ValueError("unknown scenario: {0}".format(args.scenario))
    if args.output is None:
        stem = os.path.splitext(os.path.basename(args.bag))[0]
        args.output = "{0}_{1}.bag".format(stem, args.scenario)

    rng = random.Random(args.seed)
    with rosbag.Bag(args.bag, "r") as inbag:
        bag_start = inbag.get_start_time()
        bag_end = inbag.get_end_time()
        bag_duration = bag_end - bag_start
        topics = inbag.get_type_and_topic_info().topics
        raw_topic_present = args.raw_topic in topics
        inject_topics = sorted(
            t for t in topics if t.startswith(args.inject_prefix)
        )
        if not inject_topics:
            raise RuntimeError(
                "no RTK topics found under inject prefix: {0}".format(
                    args.inject_prefix
                )
            )

        if args.windows is not None:
            windows_spec = parse_windows(args.windows)
        elif args.n_windows is not None:
            windows_spec = random_windows(
                bag_duration, args.n_windows, args.window_duration,
                args.gap, args.margin, rng,
            )
        else:
            raise ValueError(
                "specify either --windows 'start-dur,...' or "
                "--n-windows/--window-duration (with --seed)"
            )
        windows = build_windows(windows_spec, args, bag_duration, rng)

        counts = {"written": 0, "modified": 0, "dropped": 0,
                  "raw_skipped": 0, "passthrough": 0}
        compression = None if args.no_compression else "lz4"
        with rosbag.Bag(args.output, "w", compression=compression) as outbag:
            for topic, message, record_time in inbag.read_messages():
                t_rel = record_time.to_sec() - bag_start
                kind = classify_topic(topic, args)
                if kind == "raw" and not args.keep_raw:
                    counts["raw_skipped"] += 1
                    continue
                if kind == "inject":
                    if args.scenario == "dropout" and in_any_window(t_rel, windows):
                        counts["dropped"] += 1
                        continue
                    if args.scenario not in ("normal", "dropout"):
                        de, dn = window_offset(t_rel, windows, args.scenario, args)
                        if de != 0.0 or dn != 0.0:
                            message = copy.deepcopy(message)
                            modify_message(message, de, dn, args.cov_scale)
                            counts["modified"] += 1
                        else:
                            counts["passthrough"] += 1
                    else:
                        counts["passthrough"] += 1
                outbag.write(topic, message, record_time)
                counts["written"] += 1

    log = {
        "scenario": args.scenario,
        "bag": args.bag,
        "output": args.output,
        "seed": args.seed,
        "bag_duration_s": bag_duration,
        "inject_prefix": args.inject_prefix,
        "reference_prefix": args.reference_prefix,
        "windows": [
            {"t0_s": w["t0"], "duration_s": w["duration"],
             "amplitude_m": w["amplitude"], "azimuth_deg": w["azimuth_deg"]}
            for w in windows
        ],
        "cov_scale": args.cov_scale,
        "ripple_amp_m": args.ripple_amp,
        "ripple_hz": args.ripple_hz,
        "raw_topic_present": raw_topic_present,
        "raw_topic_kept": args.keep_raw,
        "counts": counts,
        "note": (
            "degradations are injected into the topics under the inject "
            "prefix (the stream consumed by the fusion algorithm); the "
            "reference prefix receiver is untouched and serves as the "
            "independent evaluation reference. Windows are relative to "
            "bag start."
        ),
    }
    log_path = os.path.splitext(args.output)[0] + ".json"
    with open(log_path, "w") as stream:
        json.dump(log, stream, indent=2, sort_keys=True)

    print("scenario      : {0}".format(args.scenario))
    print("input bag     : {0} ({1:.1f} s)".format(args.bag, bag_duration))
    print("output bag    : {0}".format(args.output))
    print("inject prefix : {0}".format(args.inject_prefix))
    print("reference     : {0} (untouched)".format(args.reference_prefix))
    print("windows       : {0}".format(
        ", ".join("{0:.1f}-{1:.1f}s".format(w["t0"], w["duration"]) for w in windows)
    ))
    print("counts        : written {0}, modified {1}, dropped {2}, "
          "raw skipped {3}, passthrough {4}".format(
              counts["written"], counts["modified"], counts["dropped"],
              counts["raw_skipped"], counts["passthrough"]))
    print("log           : {0}".format(log_path))


class FakeCovarianceMessage:
    pass


def selftest(args):
    # window spec parsing
    assert parse_windows("100-60,300-60") == [(100.0, 60.0), (300.0, 60.0)]
    try:
        parse_windows("100-0")
        raise AssertionError("expected ValueError for zero duration")
    except ValueError:
        pass

    # random windows fit, are non-overlapping and inside the margins
    rng = random.Random(7)
    windows = random_windows(600.0, 3, 60.0, 20.0, 10.0, rng)
    assert len(windows) == 3
    for t0, duration in windows:
        assert 10.0 <= t0 and t0 + duration <= 590.0
    bounds = sorted((t0, t0 + duration) for t0, duration in windows)
    for first, second in zip(bounds, bounds[1:]):
        assert first[1] <= second[0] + 1e-9, "windows overlap"
    try:
        random_windows(100.0, 5, 60.0, 20.0, 10.0, rng)
        raise AssertionError("expected ValueError for infeasible windows")
    except ValueError:
        pass

    # meters to degrees at lat 23 deg, h 50 m
    # 1 m north changes latitude only
    d_lat, d_lon = meters_to_deg(0.0, 1.0, 23.0, 50.0)
    assert abs(d_lat - 9.0298e-6) < 1e-9, d_lat
    assert abs(d_lon) < 1e-12, d_lon
    # 1 m east changes longitude only
    d_lat2, d_lon2 = meters_to_deg(1.0, 0.0, 23.0, 50.0)
    assert abs(d_lat2) < 1e-12, d_lat2
    assert abs(d_lon2 - 9.7539e-6) < 1e-9, d_lon2

    # offset behavior
    window = {"t0": 100.0, "duration": 60.0, "amplitude": 1.0,
              "dir_e": 1.0, "dir_n": 0.0, "phase": 0.0}
    args.ripple_amp = 0.0
    assert window_offset(99.0, window, "bias", args) == (0.0, 0.0)
    de_peak, _ = window_offset(130.0, window, "bias", args)
    assert abs(de_peak - 1.0) < 1e-9, de_peak
    assert window_offset(160.0, window, "bias", args) == (0.0, 0.0)
    de_jump, _ = window_offset(101.0, window, "jump", args)
    assert abs(de_jump - 1.0) < 1e-9
    assert window_offset(170.0, window, "jump", args) == (0.0, 0.0)

    # window membership
    assert in_any_window(130.0, [window])
    assert not in_any_window(170.0, [window])

    # covariance scaling on fake messages
    scale_args = argparse.Namespace(cov_scale=0.25)
    fake = FakeCovarianceMessage()
    fake.position_covariance = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    fake._type = "sensor_msgs/NavSatFix"
    fake.latitude = 23.0
    fake.longitude = 113.0
    fake.altitude = 50.0
    modify_message(fake, 0.0, 1.0, scale_args.cov_scale)
    assert abs(fake.latitude - 23.0) > 0.0, "latitude should move north"
    assert abs(fake.position_covariance[0] - 0.25) < 1e-12

    fake_epoch = FakeCovarianceMessage()
    fake_epoch.position_covariance_enu = [1.0] * 9
    fake_epoch._type = "nmea_rtk_driver/RtkEpoch"
    fake_epoch.latitude_deg = 23.0
    fake_epoch.longitude_deg = 113.0
    fake_epoch.ellipsoid_height_m = 50.0
    fake_epoch.gst_rms_m = 2.0
    modify_message(fake_epoch, 0.0, 1.0, 0.25)
    assert abs(fake_epoch.position_covariance_enu[0] - 0.25) < 1e-12
    assert abs(fake_epoch.gst_rms_m - 1.0) < 1e-12

    print("SELFTEST PASSED")


def parse_arguments():
    parser = argparse.ArgumentParser(
        description=(
            "Inject controlled RTK degradations (dropout/bias/jump/"
            "false_fixed) into the RTK stream consumed by the fusion "
            "algorithm (--inject-prefix) while keeping the other receiver "
            "(--reference-prefix) untouched as the evaluation reference."
        )
    )
    parser.add_argument("--bag", default="", help="input ROS1 bag")
    parser.add_argument("--output", default=None, help="output bag path")
    parser.add_argument(
        "--scenario", choices=SCENARIOS, default="bias",
        help="degradation condition to inject (default: bias)",
    )
    parser.add_argument(
        "--windows", default=None,
        help="explicit windows as 'start-dur,start-dur' in bag-relative seconds",
    )
    parser.add_argument("--n-windows", type=int, default=None,
                        help="number of randomly placed windows")
    parser.add_argument("--window-duration", type=float, default=60.0,
                        help="window duration for randomly placed windows (s)")
    parser.add_argument("--gap", type=float, default=20.0,
                        help="minimum gap between random windows (s)")
    parser.add_argument("--margin", type=float, default=10.0,
                        help="margin at bag start/end for random windows (s)")
    parser.add_argument("--seed", type=int, default=1,
                        help="RNG seed for window placement, azimuths, phases")
    parser.add_argument("--amplitude", type=float, default=None,
                        help="error amplitude in meters; defaults per scenario "
                             "(bias 1.0, jump 2.0, false_fixed 1.5)")
    parser.add_argument("--azimuth-deg", type=float, default=None,
                        help="fixed injection azimuth in degrees; random per "
                             "window when omitted")
    parser.add_argument("--ripple-amp", type=float, default=0.0,
                        help="sinusoidal ripple amplitude on the bias (m)")
    parser.add_argument("--ripple-hz", type=float, default=0.05,
                        help="ripple frequency (Hz)")
    parser.add_argument("--cov-scale", type=float, default=1.0,
                        help="scale factor on the reported covariance "
                             "(0.25 = overconfident false-fixed)")
    parser.add_argument(
        "--inject-prefix",
        default=DEFAULT_INJECT_PREFIX,
        help="topic prefix of the RTK stream fed to the fusion algorithm; "
             "degradations are injected here (default: /rtk_truth/)",
    )
    parser.add_argument(
        "--reference-prefix",
        default=DEFAULT_REFERENCE_PREFIX,
        help="topic prefix of the untouched reference receiver used for "
             "evaluation (default: /rtk/)",
    )
    parser.add_argument(
        "--raw-topic",
        default="/rtk_truth/raw_sentence",
        help="raw sentence topic of the injected receiver; dropped from "
             "the output unless --keep-raw is set",
    )
    parser.add_argument("--keep-raw", action="store_true",
                        help="keep the injected receiver's raw sentence "
                             "topic in the output bag")
    parser.add_argument("--no-compression", action="store_true",
                        help="write the output bag without lz4 compression")
    parser.add_argument("--selftest", action="store_true",
                        help="run the pure-logic self-test (no rosbag needed)")
    return parser.parse_args()


def main():
    args = parse_arguments()
    if args.selftest:
        selftest(args)
        return
    if not args.bag:
        raise IOError("--bag is required")
    if args.amplitude is None:
        defaults = {"bias": 1.0, "jump": 2.0, "false_fixed": 1.5}
        args.amplitude = defaults.get(args.scenario, 0.0)
    run(args)


if __name__ == "__main__":
    try:
        main()
    except Exception as error:
        print("ERROR: {0}".format(error), file=sys.stderr)
        sys.exit(1)
