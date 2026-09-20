#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Generate Fig. 7 for a continuous ROS1 bag.

The bag is expected to contain one continuous recording with the following
sequence (the transfer interval is not used in the static statistics)::

    open-sky static -> robot transfer -> canopy-covered static

The exact two static intervals must be supplied by the user because a bag does
not contain a semantic label saying when the antenna is under a canopy.  The
script produces:

  fig7_rtk_quality.png       2 x 2 publication figure
  fig7_samples.csv           samples used for the plots
  fig7_summary.csv           numerical comparison of the two intervals

Example (run after sourcing ROS1):

  python3 plot_rtk_quality_fig7.py \
      --bag orchard1.bag \
      --topic /rtk_truth/epoch \
      --open-range 0-120 \
      --canopy-range 300-420 \
      --out-dir fig7

Times are seconds relative to the first valid message on the selected RTK
topic by default.  Use --time-origin bag if your intervals are defined from
the beginning of the whole bag.
"""

from __future__ import print_function

import argparse
import csv
import math
import os
import sys


FIX_LABELS = {
    0: "none",
    1: "single",
    2: "differential",
    3: "PPS",
    4: "RTK fixed",
    5: "RTK float",
    6: "estimated",
    7: "manual",
    8: "simulation",
    9: "RTK unspecified",
}


def finite(value):
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def value_or_nan(message, *names):
    for name in names:
        if hasattr(message, name):
            value = getattr(message, name)
            if finite(value):
                return float(value)
    return float("nan")


def parse_range(spec):
    """Parse START-END, using a half-open interval [START, END)."""
    parts = spec.strip().split("-", 1)
    if len(parts) != 2:
        raise ValueError("range must be written as START-END: {0}".format(spec))
    start = float(parts[0])
    end = float(parts[1])
    if start < 0.0 or end <= start:
        raise ValueError("range must satisfy 0 <= START < END: {0}".format(spec))
    return start, end


def import_plot_dependencies():
    try:
        import numpy as np
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        return np, plt
    except ImportError as error:
        raise RuntimeError(
            "numpy and matplotlib are required for plotting"
        ) from error


def choose_topic(bag, requested):
    info = bag.get_type_and_topic_info().topics
    if requested:
        if requested not in info:
            available = ", ".join(sorted(info.keys()))
            raise RuntimeError(
                "topic not found: {0}\nAvailable topics: {1}".format(
                    requested, available
                )
            )
        return requested

    # Prefer the fusion stream used in the manuscript, then the other common
    # stream name.  A user can always override this with --topic.
    for candidate in ("/rtk_truth/epoch", "/rtk/epoch"):
        if candidate in info:
            return candidate
    epoch_topics = [name for name in info if name.endswith("/epoch")]
    if len(epoch_topics) == 1:
        return epoch_topics[0]
    raise RuntimeError(
        "Could not select an RTK epoch topic. Please specify --topic.\n"
        "Available topics: {0}".format(", ".join(sorted(info.keys())))
    )


def read_rtk_records(bag_path, requested_topic, time_origin):
    try:
        import rosbag
    except ImportError as error:
        raise RuntimeError(
            "rosbag is required. Run this script inside a sourced ROS1 environment."
        ) from error

    records = []
    with rosbag.Bag(bag_path, "r") as bag:
        topic = choose_topic(bag, requested_topic)
        bag_start = bag.get_start_time()
        for topic_name, message, bag_time in bag.read_messages(topics=[topic]):
            # RtkEpoch uses latitude_deg/longitude_deg/ellipsoid_height_m.  The
            # fallback names also support sensor_msgs/NavSatFix-like messages.
            lat = value_or_nan(message, "latitude_deg", "latitude")
            lon = value_or_nan(message, "longitude_deg", "longitude")
            height = value_or_nan(
                message, "ellipsoid_height_m", "altitude", "height_m"
            )
            if not (finite(lat) and finite(lon)):
                continue

            covariance = list(getattr(message, "position_covariance_enu", []))
            sigma_h = float("nan")
            sigma_u = float("nan")
            if len(covariance) >= 9:
                if finite(covariance[0]) and finite(covariance[4]):
                    sigma_h = math.sqrt(
                        max(0.0, float(covariance[0]) + float(covariance[4]))
                    )
                if finite(covariance[8]):
                    sigma_u = math.sqrt(max(0.0, float(covariance[8])))

            # Fallback if the message does not contain an ENU covariance.
            if not finite(sigma_h):
                gst_lat = value_or_nan(message, "gst_latitude_std_m")
                gst_lon = value_or_nan(message, "gst_longitude_std_m")
                if finite(gst_lat) and finite(gst_lon):
                    sigma_h = math.hypot(gst_lat, gst_lon)
            if not finite(sigma_u):
                sigma_u = value_or_nan(message, "gst_altitude_std_m")

            fix_quality = getattr(message, "fix_quality", float("nan"))
            if not finite(fix_quality):
                fix_quality = float("nan")
            else:
                fix_quality = int(fix_quality)

            satellites = value_or_nan(message, "satellites_used")
            differential_age = value_or_nan(message, "differential_age_sec")
            records.append(
                {
                    "abs_time": bag_time.to_sec(),
                    "lat": lat,
                    "lon": lon,
                    "height": height,
                    "sigma_h": sigma_h,
                    "sigma_u": sigma_u,
                    "fix_quality": fix_quality,
                    "satellites": satellites,
                    "differential_age": differential_age,
                }
            )

    if not records:
        raise RuntimeError("No valid RTK position messages were found on {0}".format(topic))

    first_time = records[0]["abs_time"]
    origin = bag_start if time_origin == "bag" else first_time
    for record in records:
        record["time"] = record["abs_time"] - origin
    return topic, records


def geodetic_to_enu(records, np):
    """Convert WGS84 latitude/longitude/height to local ENU coordinates."""
    valid_height = [r["height"] for r in records if finite(r["height"])]
    lat0 = float(np.median([r["lat"] for r in records]))
    lon0 = float(np.median([r["lon"] for r in records]))
    h0 = float(np.median(valid_height)) if valid_height else 0.0

    a = 6378137.0
    e2 = 6.6943799901413165e-3

    def ecef(lat_deg, lon_deg, h):
        lat = math.radians(lat_deg)
        lon = math.radians(lon_deg)
        sin_lat = math.sin(lat)
        cos_lat = math.cos(lat)
        sin_lon = math.sin(lon)
        cos_lon = math.cos(lon)
        n = a / math.sqrt(1.0 - e2 * sin_lat * sin_lat)
        return (
            (n + h) * cos_lat * cos_lon,
            (n + h) * cos_lat * sin_lon,
            (n * (1.0 - e2) + h) * sin_lat,
        )

    x0, y0, z0 = ecef(lat0, lon0, h0)
    lat0_rad = math.radians(lat0)
    lon0_rad = math.radians(lon0)
    sin_lat = math.sin(lat0_rad)
    cos_lat = math.cos(lat0_rad)
    sin_lon = math.sin(lon0_rad)
    cos_lon = math.cos(lon0_rad)

    for record in records:
        h = record["height"] if finite(record["height"]) else h0
        x, y, z = ecef(record["lat"], record["lon"], h)
        dx, dy, dz = x - x0, y - y0, z - z0
        record["east"] = -sin_lon * dx + cos_lon * dy
        record["north"] = (
            -sin_lat * cos_lon * dx
            - sin_lat * sin_lon * dy
            + cos_lat * dz
        )
        record["up"] = (
            cos_lat * cos_lon * dx
            + cos_lat * sin_lon * dy
            + sin_lat * dz
        )
    return lat0, lon0, h0


def select_segment(records, interval, label, np):
    start, end = interval
    selected = [r.copy() for r in records if start <= r["time"] < end]
    if len(selected) < 2:
        raise RuntimeError(
            "The {0} interval [{1:.3f}, {2:.3f}) contains fewer than two "
            "valid RTK samples.".format(label, start, end)
        )

    e0 = float(np.median([r["east"] for r in selected]))
    n0 = float(np.median([r["north"] for r in selected]))
    u0 = float(np.median([r["up"] for r in selected]))
    t0 = selected[0]["time"]
    for record in selected:
        record["condition"] = label
        record["relative_time"] = record["time"] - t0
        record["east_centered"] = record["east"] - e0
        record["north_centered"] = record["north"] - n0
        record["up_centered"] = record["up"] - u0
    return selected


def percentile(values, np, q):
    values = np.asarray([v for v in values if finite(v)], dtype=float)
    return float(np.percentile(values, q)) if values.size else float("nan")


def segment_statistics(segment, np):
    east = np.asarray([r["east_centered"] for r in segment], dtype=float)
    north = np.asarray([r["north_centered"] for r in segment], dtype=float)
    up = np.asarray([r["up_centered"] for r in segment], dtype=float)
    sigma_h = np.asarray([r["sigma_h"] for r in segment], dtype=float)
    sigma_u = np.asarray([r["sigma_u"] for r in segment], dtype=float)
    fix = np.asarray([r["fix_quality"] for r in segment], dtype=float)
    sats = np.asarray([r["satellites"] for r in segment], dtype=float)
    age = np.asarray([r["differential_age"] for r in segment], dtype=float)

    std_e = float(np.std(east, ddof=0))
    std_n = float(np.std(north, ddof=0))
    std_u = float(np.std(up, ddof=0))
    valid_fix = fix[np.isfinite(fix)]
    return {
        "condition": segment[0]["condition"],
        "samples": len(segment),
        "duration_s": segment[-1]["time"] - segment[0]["time"],
        "std_E_mm": std_e * 1000.0,
        "std_N_mm": std_n * 1000.0,
        "std_U_mm": std_u * 1000.0,
        "2DRMS_horizontal_mm": 2000.0 * math.hypot(std_e, std_n),
        "mean_sigma_h_m": float(np.nanmean(sigma_h)) if np.isfinite(sigma_h).any() else float("nan"),
        "p95_sigma_h_m": percentile(sigma_h, np, 95.0),
        "mean_sigma_U_m": float(np.nanmean(sigma_u)) if np.isfinite(sigma_u).any() else float("nan"),
        "p95_sigma_U_m": percentile(sigma_u, np, 95.0),
        "fixed_ratio": float(np.mean(valid_fix == 4)) if valid_fix.size else float("nan"),
        "float_ratio": float(np.mean(valid_fix == 5)) if valid_fix.size else float("nan"),
        "mean_satellites": float(np.nanmean(sats)) if np.isfinite(sats).any() else float("nan"),
        "min_satellites": float(np.nanmin(sats)) if np.isfinite(sats).any() else float("nan"),
        "mean_differential_age_s": float(np.nanmean(age)) if np.isfinite(age).any() else float("nan"),
        "p95_differential_age_s": percentile(age, np, 95.0),
    }


def write_samples(path, segments):
    fields = [
        "condition", "time", "relative_time", "lat", "lon", "height",
        "east", "north", "up", "east_centered", "north_centered",
        "up_centered", "sigma_h", "sigma_u", "fix_quality", "satellites",
        "differential_age",
    ]
    with open(path, "w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for segment in segments:
            for record in segment:
                writer.writerow({field: record.get(field, "") for field in fields})


def write_summary(path, statistics):
    fields = list(statistics[0].keys())
    with open(path, "w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(statistics)


def plot_fig7(path, segments, statistics, np, plt):
    colors = {"open-sky static": "#1769aa", "canopy static": "#c23b22"}
    line_styles = {"open-sky static": "-", "canopy static": "--"}

    fig, axes = plt.subplots(2, 2, figsize=(10.5, 7.6), dpi=300)
    ax_a, ax_b, ax_c, ax_d = axes.ravel()

    # (a) Centered horizontal position samples and 2DRMS circles.
    for segment, stat in zip(segments, statistics):
        label = segment[0]["condition"]
        color = colors[label]
        e = np.asarray([r["east_centered"] for r in segment]) * 1000.0
        n = np.asarray([r["north_centered"] for r in segment]) * 1000.0
        ax_a.scatter(e, n, s=5, alpha=0.30, color=color, label=label)
        radius = stat["2DRMS_horizontal_mm"]
        theta = np.linspace(0.0, 2.0 * math.pi, 240)
        ax_a.plot(
            radius * np.cos(theta), radius * np.sin(theta),
            color=color, lw=1.2, ls="--",
            label="{0} 2DRMS = {1:.1f} mm".format(label, radius),
        )
    ax_a.axhline(0.0, color="0.75", lw=0.6)
    ax_a.axvline(0.0, color="0.75", lw=0.6)
    ax_a.set_aspect("equal", adjustable="box")
    ax_a.set_xlabel("East relative to segment median (mm)")
    ax_a.set_ylabel("North relative to segment median (mm)")
    ax_a.set_title("(a) Horizontal distribution of RTK position samples")
    ax_a.grid(True, color="0.90", lw=0.5)
    ax_a.legend(fontsize=7, loc="best")

    # (b) Reported horizontal and vertical uncertainty.
    for segment in segments:
        label = segment[0]["condition"]
        color = colors[label]
        t = np.asarray([r["relative_time"] for r in segment])
        sh = np.asarray([r["sigma_h"] for r in segment])
        su = np.asarray([r["sigma_u"] for r in segment])
        if np.isfinite(sh).any():
            ax_b.plot(t, sh, color=color, lw=0.9, ls=line_styles[label],
                      label="{0}: horizontal".format(label))
        if np.isfinite(su).any():
            ax_b.plot(t, su, color=color, lw=0.9, ls=":",
                      label="{0}: vertical".format(label))
    ax_b.set_xlabel("Time from beginning of static segment (s)")
    ax_b.set_ylabel("Reported uncertainty (m)")
    ax_b.set_title("(b) Reported RTK uncertainty during static observation")
    ax_b.grid(True, color="0.90", lw=0.5)
    ax_b.legend(fontsize=7, loc="best")

    # (c) Solution status and satellite availability.
    ax_c2 = ax_c.twinx()
    status_lines = []
    satellite_lines = []
    for segment in segments:
        label = segment[0]["condition"]
        color = colors[label]
        t = np.asarray([r["relative_time"] for r in segment])
        fix = np.asarray([r["fix_quality"] for r in segment])
        sats = np.asarray([r["satellites"] for r in segment])
        if np.isfinite(fix).any():
            line = ax_c.step(t, fix, where="post", color=color,
                             lw=0.9, ls=line_styles[label],
                             label="{0}: solution status".format(label))[0]
            status_lines.append(line)
        if np.isfinite(sats).any():
            line = ax_c2.plot(t, sats, color=color, lw=0.9,
                              alpha=0.65, label="{0}: satellites".format(label))[0]
            satellite_lines.append(line)
    ax_c.set_ylim(-0.5, 5.5)
    ax_c.set_yticks([0, 1, 2, 4, 5])
    ax_c.set_yticklabels(["none", "single", "diff.", "fixed", "float"], fontsize=8)
    ax_c.set_xlabel("Time from beginning of static segment (s)")
    ax_c.set_ylabel("Solution status")
    ax_c2.set_ylabel("Satellites used")
    ax_c.set_title("(c) RTK solution status and satellite availability")
    ax_c.grid(True, color="0.90", lw=0.5)
    handles = status_lines + satellite_lines
    labels = [h.get_label() for h in handles]
    if handles:
        ax_c.legend(handles, labels, fontsize=7, loc="best")

    # (d) Differential age; this panel remains useful even when status stays
    # fixed, because correction latency can change before a status transition.
    has_age = False
    for segment in segments:
        label = segment[0]["condition"]
        color = colors[label]
        t = np.asarray([r["relative_time"] for r in segment])
        age = np.asarray([r["differential_age"] for r in segment])
        if np.isfinite(age).any():
            has_age = True
            ax_d.plot(t, age, color=color, lw=0.9, ls=line_styles[label],
                      label=label)
    ax_d.set_xlabel("Time from beginning of static segment (s)")
    ax_d.set_ylabel("Differential age (s)")
    ax_d.set_title("(d) Differential age during static observation")
    ax_d.grid(True, color="0.90", lw=0.5)
    if has_age:
        ax_d.legend(fontsize=7, loc="best")
    else:
        ax_d.text(0.5, 0.5, "No valid differential-age data",
                  ha="center", va="center", transform=ax_d.transAxes)

    fig.tight_layout()
    fig.savefig(path, dpi=300, bbox_inches="tight")
    plt.close(fig)


def parse_arguments():
    parser = argparse.ArgumentParser(
        description="Generate Fig. 7 from open-sky and canopy static RTK segments"
    )
    parser.add_argument("--bag", required=True, help="input ROS1 bag")
    parser.add_argument(
        "--topic", default=None,
        help="RTK epoch topic, e.g. /rtk_truth/epoch; auto-selected if omitted",
    )
    parser.add_argument(
        "--open-range", required=True, metavar="START-END",
        help="open-sky static interval in seconds",
    )
    parser.add_argument(
        "--canopy-range", required=True, metavar="START-END",
        help="canopy-covered static interval in seconds",
    )
    parser.add_argument(
        "--time-origin", choices=("topic", "bag"), default="topic",
        help="time reference for the two intervals (default: first valid RTK epoch)",
    )
    parser.add_argument(
        "--out-dir", default="fig7_rtk_quality",
        help="output directory (default: fig7_rtk_quality)",
    )
    return parser.parse_args()


def main():
    args = parse_arguments()
    np, plt = import_plot_dependencies()
    open_interval = parse_range(args.open_range)
    canopy_interval = parse_range(args.canopy_range)
    topic, records = read_rtk_records(args.bag, args.topic, args.time_origin)
    geodetic_origin = geodetic_to_enu(records, np)
    open_segment = select_segment(records, open_interval, "open-sky static", np)
    canopy_segment = select_segment(records, canopy_interval, "canopy static", np)
    segments = [open_segment, canopy_segment]
    statistics = [segment_statistics(segment, np) for segment in segments]

    if not os.path.isdir(args.out_dir):
        os.makedirs(args.out_dir)
    figure_path = os.path.join(args.out_dir, "fig7_rtk_quality.png")
    samples_path = os.path.join(args.out_dir, "fig7_samples.csv")
    summary_path = os.path.join(args.out_dir, "fig7_summary.csv")
    plot_fig7(figure_path, segments, statistics, np, plt)
    write_samples(samples_path, segments)
    write_summary(summary_path, statistics)

    print("RTK topic       : {0}".format(topic))
    print("ENU origin      : lat {0:.8f}, lon {1:.8f}, h {2:.3f} m".format(*geodetic_origin))
    print("Open-sky range  : [{0:.3f}, {1:.3f}) s, {2} samples".format(
        open_interval[0], open_interval[1], len(open_segment)))
    print("Canopy range    : [{0:.3f}, {1:.3f}) s, {2} samples".format(
        canopy_interval[0], canopy_interval[1], len(canopy_segment)))
    print("Figure written   : {0}".format(figure_path))
    print("Samples written  : {0}".format(samples_path))
    print("Summary written  : {0}".format(summary_path))
    print("\nSummary:")
    for stat in statistics:
        print(
            "  {condition}: 2DRMS_h={2DRMS_horizontal_mm:.1f} mm, "
            "mean sigma_h={mean_sigma_h_m:.4f} m, fixed ratio={fixed_ratio}".format(
                **stat
            )
        )


if __name__ == "__main__":
    try:
        main()
    except Exception as error:
        print("ERROR: {0}".format(error), file=sys.stderr)
        sys.exit(1)

