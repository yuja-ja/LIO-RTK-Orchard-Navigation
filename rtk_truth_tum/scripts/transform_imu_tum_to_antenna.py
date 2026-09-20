#!/usr/bin/env python3
"""Move a FAST-LIO IMU-center TUM trajectory to an RTK antenna point."""

import argparse
import io
import math
import os
import sys


def normalize_quaternion(quaternion):
    norm = math.sqrt(sum(value * value for value in quaternion))
    if norm < 1.0e-12:
        raise ValueError("zero quaternion")
    return tuple(value / norm for value in quaternion)


def rotate_vector(quaternion, vector):
    qx, qy, qz, qw = quaternion
    vx, vy, vz = vector
    return (
        (1.0 - 2.0 * (qy * qy + qz * qz)) * vx
        + 2.0 * (qx * qy - qz * qw) * vy
        + 2.0 * (qx * qz + qy * qw) * vz,
        2.0 * (qx * qy + qz * qw) * vx
        + (1.0 - 2.0 * (qx * qx + qz * qz)) * vy
        + 2.0 * (qy * qz - qx * qw) * vz,
        2.0 * (qx * qz - qy * qw) * vx
        + 2.0 * (qy * qz + qx * qw) * vy
        + (1.0 - 2.0 * (qx * qx + qy * qy)) * vz,
    )


def parse_tum(path):
    records = []
    with io.open(path, "r", encoding="ascii") as stream:
        for line_number, line in enumerate(stream, 1):
            stripped = line.strip()
            if not stripped or stripped.startswith("#"):
                continue
            fields = stripped.split()
            if len(fields) != 8:
                raise ValueError(
                    "{0}:{1}: expected 8 TUM fields".format(path, line_number)
                )
            values = tuple(float(field) for field in fields)
            if not all(math.isfinite(value) for value in values):
                continue
            quaternion = normalize_quaternion(values[4:8])
            records.append(values[:4] + quaternion)
    if not records:
        raise RuntimeError("no valid TUM poses found: {0}".format(path))
    records.sort(key=lambda record: record[0])
    unique = []
    for record in records:
        serialized_stamp = "{:.9f}".format(record[0])
        if unique and serialized_stamp == unique[-1][0]:
            unique[-1] = (serialized_stamp, record)
        else:
            unique.append((serialized_stamp, record))
    return [record for _, record in unique]


def transform(records, lever_arm):
    transformed = []
    for record in records:
        stamp, x, y, z, qx, qy, qz, qw = record
        quaternion = (qx, qy, qz, qw)
        lever_world = rotate_vector(quaternion, lever_arm)
        transformed.append(
            (
                stamp,
                x + lever_world[0],
                y + lever_world[1],
                z + lever_world[2],
                qx,
                qy,
                qz,
                qw,
            )
        )
    return transformed


def write_tum(path, records, source, lever_arm):
    parent = os.path.dirname(os.path.abspath(path))
    if parent and not os.path.isdir(parent):
        os.makedirs(parent)
    with io.open(path, "w", encoding="ascii") as stream:
        stream.write("# timestamp tx ty tz qx qy qz qw\n")
        stream.write("# source_imu_tum: {0}\n".format(source))
        stream.write(
            "# lever_arm_imu_to_truth_antenna_m: {:.6f} {:.6f} {:.6f}\n".format(
                *lever_arm
            )
        )
        for record in records:
            stream.write(
                "{:.9f} {:.9f} {:.9f} {:.9f} "
                "{:.9f} {:.9f} {:.9f} {:.9f}\n".format(*record)
            )


def parse_arguments():
    parser = argparse.ArgumentParser(
        description="Transform FAST-LIO IMU-center poses to the truth antenna."
    )
    parser.add_argument("--input", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument(
        "--lever-arm",
        nargs=3,
        required=True,
        type=float,
        metavar=("X", "Y", "Z"),
        help="IMU origin to truth main antenna, expressed in IMU frame (m)",
    )
    return parser.parse_args()


def main():
    args = parse_arguments()
    records = parse_tum(args.input)
    transformed = transform(records, tuple(args.lever_arm))
    write_tum(args.output, transformed, args.input, tuple(args.lever_arm))
    print("poses written: {0}".format(len(transformed)))


if __name__ == "__main__":
    try:
        main()
    except Exception as error:
        print("ERROR: {0}".format(error), file=sys.stderr)
        sys.exit(1)
