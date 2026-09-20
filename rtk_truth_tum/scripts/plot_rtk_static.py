#!/usr/bin/env python3
"""Plot static-accuracy evaluation results from pairs.csv.

Reads the pair CSV produced by evaluate_rtk_static.py and writes three
publication-grade figures (300 dpi PNG):

  fig1  static_position_scatter.png   : per-receiver EN scatter + 2DRMS circle
  fig2  static_series.png             : E/N time series per receiver
  fig3  static_difference.png         : truth-primary difference time series

Run:

  python3 plot_rtk_static.py pairs.csv --out-dir figures \
      --title "RTK static evaluation, orchard site, 2026-08-21"

Requires matplotlib and numpy.
"""

import argparse
import csv
import math
import os
import sys


def read_pairs(path):
    rows = []
    with open(path, "r", newline="") as stream:
        reader = csv.DictReader(stream)
        for row in reader:
            rows.append({key: float(value) for key, value in row.items()})
    if not rows:
        raise RuntimeError("pairs CSV is empty")
    return rows


def load_helpers():
    try:
        import numpy as np
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        return np, plt
    except ImportError as error:
        raise RuntimeError("numpy and matplotlib are required") from error


def receiver_2drms(rows, np, name):
    east = np.array([row["e_" + name] for row in rows])
    north = np.array([row["n_" + name] for row in rows])
    return 2000.0 * math.hypot(float(east.std()), float(north.std()))


def plot_scatter(rows, np, plt, out_path, title):
    fig, ax = plt.subplots(figsize=(4.4, 4.4), dpi=300)
    primary_e = [row["e_primary"] for row in rows]
    primary_n = [row["n_primary"] for row in rows]
    truth_e = [row["e_truth"] for row in rows]
    truth_n = [row["n_truth"] for row in rows]

    pe, pn = np.array(primary_e), np.array(primary_n)
    te, tn = np.array(truth_e), np.array(truth_n)
    pe_m, pn_m = float(pe.mean()), float(pn.mean())
    te_m, tn_m = float(te.mean()), float(tn.mean())
    primary_2drms = receiver_2drms(rows, np, "primary")
    truth_2drms = receiver_2drms(rows, np, "truth")

    # 2DRMS circles about each receiver's own mean
    for mean_e, mean_n, array_e, array_n, color in (
        (pe_m, pn_m, pe, pn, "#1f77b4"),
        (te_m, tn_m, te, tn, "#d62728"),
    ):
        sigma_h = math.hypot(float(array_e.std()), float(array_n.std()))
        theta = np.linspace(0.0, 2.0 * math.pi, 180)
        ax.plot(
            mean_e + 2.0 * sigma_h * np.cos(theta),
            mean_n + 2.0 * sigma_h * np.sin(theta),
            color=color,
            lw=1.0,
            ls="--",
            alpha=0.9,
        )

    ax.scatter(
        (pe - pe_m) * 1000.0, (pn - pn_m) * 1000.0, s=2.5, c="#1f77b4",
        alpha=0.25, linewidths=0,
        label="primary (2DRMS {0:.1f} mm)".format(primary_2drms),
    )
    ax.scatter(
        (te - te_m) * 1000.0, (tn - tn_m) * 1000.0, s=2.5, c="#d62728",
        alpha=0.25, linewidths=0,
        label="truth (2DRMS {0:.1f} mm)".format(truth_2drms),
    )

    limit = 45.0
    ax.set_xlim(-limit, limit)
    ax.set_ylim(-limit, limit)
    ax.set_aspect("equal", adjustable="box")
    ax.axhline(0.0, color="0.7", lw=0.6)
    ax.axvline(0.0, color="0.7", lw=0.6)
    ax.set_xlabel("East about session mean (mm)")
    ax.set_ylabel("North about session mean (mm)")
    ax.set_title(title, fontsize=10)
    ax.legend(fontsize=8, loc="upper right")
    ax.grid(True, color="0.9", lw=0.4)
    fig.tight_layout()
    fig.savefig(out_path)
    plt.close(fig)


def plot_series(rows, np, plt, out_path, title):
    fig, axes = plt.subplots(2, 1, figsize=(7.2, 4.6), dpi=300, sharex=True)
    t = np.array([row["t"] for row in rows]) - rows[0]["t"]
    colors = ("#1f77b4", "#d62728")
    names = ("primary", "truth")
    labels = {
        "primary": "primary (2DRMS {0:.1f} mm)".format(
            receiver_2drms(rows, np, "primary")
        ),
        "truth": "truth (2DRMS {0:.1f} mm)".format(
            receiver_2drms(rows, np, "truth")
        ),
    }
    for axis, component in zip(axes, ("e_", "n_")):
        axis.axhline(0.0, color="0.7", lw=0.6)
        for color, name in zip(colors, names):
            series = np.array([row[component + name] for row in rows])
            series = series - series.mean()
            axis.plot(
                t, series * 1000.0, color=color, lw=0.8,
                label=labels[name],
            )
        axis.set_ylabel("{0} about mean (mm)".format(component[0].upper()))
        axis.legend(fontsize=8, loc="upper right")
        axis.grid(True, color="0.9", lw=0.4)
    axes[0].set_title(title, fontsize=10)
    axes[1].set_xlabel("Time (s)")
    fig.tight_layout()
    fig.savefig(out_path)
    plt.close(fig)


def plot_difference(rows, np, plt, out_path, title):
    fig, axes = plt.subplots(3, 1, figsize=(7.2, 5.8), dpi=300, sharex=True)
    t = np.array([row["t"] for row in rows]) - rows[0]["t"]
    for axis, component in zip(axes, ("de", "dn", "du")):
        series = np.array([row[component] for row in rows])
        mean_value = float(series.mean())
        axis.axhline(mean_value * 1000.0, color="0.6", lw=0.8, ls="--")
        axis.plot(t, series * 1000.0, color="#2ca02c", lw=0.8)
        axis.set_ylabel("d{0} (mm)".format(component[1].upper()))
        axis.grid(True, color="0.9", lw=0.4)
    axes[0].set_title(title, fontsize=10)
    axes[2].set_xlabel("Time (s)")
    fig.tight_layout()
    fig.savefig(out_path)
    plt.close(fig)


def parse_arguments():
    parser = argparse.ArgumentParser(
        description="Plot static-accuracy evaluation results from pairs.csv"
    )
    parser.add_argument("pairs", help="pairs.csv written by evaluate_rtk_static.py")
    parser.add_argument("--out-dir", default=".", help="output directory for PNG files")
    parser.add_argument(
        "--title", default="RTK static evaluation", help="figure title text"
    )
    return parser.parse_args()


def main():
    args = parse_arguments()
    rows = read_pairs(args.pairs)
    np, plt = load_helpers()
    os.makedirs(args.out_dir, exist_ok=True)
    plot_scatter(
        rows, np, plt,
        os.path.join(args.out_dir, "static_position_scatter.png"), args.title,
    )
    plot_series(
        rows, np, plt,
        os.path.join(args.out_dir, "static_series.png"), args.title,
    )
    plot_difference(
        rows, np, plt,
        os.path.join(args.out_dir, "static_difference.png"), args.title,
    )
    print("figures written to {0}".format(args.out_dir))


if __name__ == "__main__":
    try:
        main()
    except Exception as error:
        print("ERROR: {0}".format(error), file=sys.stderr)
        sys.exit(1)
