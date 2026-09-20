#!/usr/bin/env python3
"""Batch driver: generate all five RTK degradation scenario bags in one run.

Uses inject_rtk_degradation.py (same directory) to produce, for one input
bag, the full five-condition suite:

  normal, dropout, bias, jump, false_fixed

The degraded scenarios are repeated for every seed in --seeds (default
1,2,3), so one command produces e.g. 13 bags: one normal control plus
4 scenarios x 3 seeds. Each run writes its own JSON experiment log next
to the output bag; a manifest summary is printed at the end.

Run inside a sourced ROS1 workspace:

  rosrun nmea_rtk_driver run_all_injections.py \
      --bag orchard.bag --seeds 1,2,3 \
      --n-windows 3 --window-duration 60 \
      --bias-amplitude 1.0 --jump-amplitude 2.0 \
      --false-fixed-amplitude 1.5 --false-fixed-cov-scale 0.25

After the run, evaluate each output bag with your fusion pipeline and
compare against the untouched reference receiver (default /rtk/).

A self-test of the argument/job logic (no bag, no rosbag needed):

  python3 run_all_injections.py --selftest
"""

import argparse
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)

import inject_rtk_degradation as inject

ALL_SCENARIOS = ("normal", "dropout", "bias", "jump", "false_fixed")


def parse_int_list(text):
    values = []
    for entry in text.split(","):
        entry = entry.strip()
        if not entry:
            continue
        values.append(int(entry))
    if not values:
        raise ValueError("empty list: {0}".format(text))
    return values


def parse_scenario_list(text):
    if text.strip().lower() == "all":
        return list(ALL_SCENARIOS)
    scenarios = []
    for entry in text.split(","):
        entry = entry.strip().lower()
        if not entry:
            continue
        if entry not in inject.SCENARIOS:
            raise ValueError("unknown scenario: {0}".format(entry))
        scenarios.append(entry)
    if not scenarios:
        raise ValueError("empty scenario list")
    return scenarios


def build_namespace(args, scenario, seed):
    """Build the Namespace inject.run() expects for one (scenario, seed)."""
    bag_path = os.path.abspath(args.bag)
    stem = os.path.splitext(os.path.basename(bag_path))[0]
    if scenario == "normal":
        suffix = "normal"
    else:
        suffix = "{0}_s{1}".format(scenario, seed)
    output = os.path.join(args.out_dir, "{0}_{1}.bag".format(stem, suffix))

    amplitudes = {
        "bias": args.bias_amplitude,
        "jump": args.jump_amplitude,
        "false_fixed": args.false_fixed_amplitude,
    }
    cov_scales = {"false_fixed": args.false_fixed_cov_scale}

    return argparse.Namespace(
        bag=bag_path,
        output=output,
        scenario=scenario,
        windows=args.windows,
        n_windows=args.n_windows,
        window_duration=args.window_duration,
        gap=args.gap,
        margin=args.margin,
        seed=seed,
        amplitude=amplitudes.get(scenario, 0.0),
        azimuth_deg=args.azimuth_deg,
        ripple_amp=args.ripple_amp,
        ripple_hz=args.ripple_hz,
        cov_scale=cov_scales.get(scenario, 1.0),
        raw_topic=args.raw_topic,
        inject_prefix=args.inject_prefix,
        reference_prefix=args.reference_prefix,
        keep_raw=args.keep_raw,
        no_compression=args.no_compression,
    )


def plan_jobs(scenarios, seeds):
    """Return the ordered list of (scenario, seed) jobs.

    The normal control is a plain passthrough of the bag, so it is only
    produced once per input bag regardless of the seed count.
    """
    jobs = []
    for scenario in scenarios:
        if scenario == "normal":
            jobs.append((scenario, seeds[0]))
        else:
            jobs.extend((scenario, seed) for seed in seeds)
    return jobs


def run_all(args):
    seeds = parse_int_list(args.seeds)
    scenarios = parse_scenario_list(args.scenarios)
    os.makedirs(args.out_dir, exist_ok=True)
    jobs = plan_jobs(scenarios, seeds)

    results = []
    for index, (scenario, seed) in enumerate(jobs, 1):
        print("")
        print("[{0}/{1}] scenario={2} seed={3}".format(
            index, len(jobs), scenario, seed))
        namespace = build_namespace(args, scenario, seed)
        inject.run(namespace)
        results.append(namespace.output)

    print("")
    print("=== injection suite complete ===")
    print("input bag    : {0}".format(os.path.abspath(args.bag)))
    print("inject prefix: {0} (degraded)".format(args.inject_prefix))
    print("reference    : {0} (untouched)".format(args.reference_prefix))
    print("output bags  :")
    for path in results:
        print("  {0}  (+ {1})".format(path, os.path.splitext(path)[0] + ".json"))
    print("")
    print("Next steps: run each output bag through the fusion pipeline and")
    print("evaluate against the untouched {0} receiver.".format(args.reference_prefix))


def selftest():
    # integer list parsing
    assert parse_int_list("1,2,3") == [1, 2, 3]
    assert parse_int_list(" 1 , 2 ") == [1, 2]
    try:
        parse_int_list("")
        raise AssertionError("expected ValueError for empty seed list")
    except ValueError:
        pass

    # scenario parsing
    assert parse_scenario_list("all") == list(ALL_SCENARIOS)
    assert parse_scenario_list("bias,jump") == ["bias", "jump"]
    assert parse_scenario_list(" NORMAL , false_fixed ") == ["normal", "false_fixed"]
    try:
        parse_scenario_list("explosion")
        raise AssertionError("expected ValueError for unknown scenario")
    except ValueError:
        pass

    # job planning: normal once, others per seed
    jobs = plan_jobs(list(ALL_SCENARIOS), [1, 2, 3])
    assert jobs[0] == ("normal", 1)
    assert jobs.count(("normal", 1)) == 1
    for scenario in ("dropout", "bias", "jump", "false_fixed"):
        assert [job for job in jobs if job[0] == scenario] == [
            (scenario, seed) for seed in (1, 2, 3)
        ]

    # namespace construction
    args = argparse.Namespace(
        bag="/data/orchard.bag",
        out_dir="/data/out",
        windows=None,
        n_windows=3,
        window_duration=60.0,
        gap=20.0,
        margin=10.0,
        azimuth_deg=None,
        ripple_amp=0.0,
        ripple_hz=0.05,
        bias_amplitude=1.0,
        jump_amplitude=2.0,
        false_fixed_amplitude=1.5,
        false_fixed_cov_scale=0.25,
        raw_topic="/rtk_truth/raw_sentence",
        inject_prefix="/rtk_truth/",
        reference_prefix="/rtk/",
        keep_raw=False,
        no_compression=False,
    )
    bias_ns = build_namespace(args, "bias", 2)
    assert bias_ns.output == os.path.join("/data/out", "orchard_bias_s2.bag")
    assert bias_ns.scenario == "bias" and bias_ns.seed == 2
    assert bias_ns.amplitude == 1.0 and bias_ns.cov_scale == 1.0
    ff_ns = build_namespace(args, "false_fixed", 1)
    assert ff_ns.output.endswith("orchard_false_fixed_s1.bag")
    assert ff_ns.amplitude == 1.5 and ff_ns.cov_scale == 0.25
    normal_ns = build_namespace(args, "normal", 1)
    assert normal_ns.output.endswith("orchard_normal.bag")
    assert normal_ns.amplitude == 0.0

    # the injection module exposes everything the driver relies on
    for attr in ("run", "SCENARIOS", "DEFAULT_INJECT_PREFIX",
                 "DEFAULT_REFERENCE_PREFIX"):
        assert hasattr(inject, attr), attr

    print("SELFTEST PASSED")


def parse_arguments():
    parser = argparse.ArgumentParser(
        description=(
            "Generate all five RTK degradation scenario bags from one "
            "input bag in a single run (driver for "
            "inject_rtk_degradation.py)."
        )
    )
    parser.add_argument("--bag", default="", help="input ROS1 bag")
    parser.add_argument(
        "--out-dir", default=None,
        help="output directory (default: directory of the input bag)",
    )
    parser.add_argument(
        "--scenarios", default="all",
        help="comma-separated scenarios or 'all' "
             "(normal,dropout,bias,jump,false_fixed)",
    )
    parser.add_argument("--seeds", default="1,2,3",
                        help="comma-separated seeds for the degraded scenarios")
    parser.add_argument("--windows", default=None,
                        help="explicit windows 'start-dur,...' in bag-relative "
                             "seconds (same windows for every scenario and seed)")
    parser.add_argument("--n-windows", type=int, default=3,
                        help="number of randomly placed windows per run")
    parser.add_argument("--window-duration", type=float, default=60.0,
                        help="window duration for random windows (s)")
    parser.add_argument("--gap", type=float, default=20.0,
                        help="minimum gap between random windows (s)")
    parser.add_argument("--margin", type=float, default=10.0,
                        help="margin at bag start/end for random windows (s)")
    parser.add_argument("--bias-amplitude", type=float, default=1.0,
                        help="bias scenario amplitude (m)")
    parser.add_argument("--jump-amplitude", type=float, default=2.0,
                        help="jump scenario amplitude (m)")
    parser.add_argument("--false-fixed-amplitude", type=float, default=1.5,
                        help="false_fixed scenario amplitude (m)")
    parser.add_argument("--false-fixed-cov-scale", type=float, default=0.25,
                        help="covariance scale for false_fixed (0.25 = "
                             "overconfident)")
    parser.add_argument("--azimuth-deg", type=float, default=None,
                        help="fixed injection azimuth in degrees; random per "
                             "window when omitted")
    parser.add_argument("--ripple-amp", type=float, default=0.0,
                        help="sinusoidal ripple amplitude on the bias (m)")
    parser.add_argument("--ripple-hz", type=float, default=0.05,
                        help="ripple frequency (Hz)")
    parser.add_argument(
        "--inject-prefix", default=inject.DEFAULT_INJECT_PREFIX,
        help="topic prefix of the RTK stream fed to the fusion algorithm "
             "(default: /rtk_truth/)",
    )
    parser.add_argument(
        "--reference-prefix", default=inject.DEFAULT_REFERENCE_PREFIX,
        help="topic prefix of the untouched reference receiver "
             "(default: /rtk/)",
    )
    parser.add_argument("--raw-topic", default="/rtk_truth/raw_sentence",
                        help="raw sentence topic of the injected receiver")
    parser.add_argument("--keep-raw", action="store_true",
                        help="keep the injected receiver's raw sentence "
                             "topic in the output bags")
    parser.add_argument("--no-compression", action="store_true",
                        help="write output bags without lz4 compression")
    parser.add_argument("--selftest", action="store_true",
                        help="run the job/argument logic self-test")
    return parser.parse_args()


def main():
    args = parse_arguments()
    if args.selftest:
        selftest()
        return
    if not args.bag:
        raise IOError("--bag is required")
    if args.out_dir is None:
        args.out_dir = os.path.dirname(os.path.abspath(args.bag))
    run_all(args)


if __name__ == "__main__":
    try:
        main()
    except Exception as error:
        print("ERROR: {0}".format(error), file=sys.stderr)
        sys.exit(1)
