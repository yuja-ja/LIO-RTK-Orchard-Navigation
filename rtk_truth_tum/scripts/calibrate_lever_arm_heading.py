#!/usr/bin/env python3
"""Estimate an IMU-to-RTK lever arm and dual-antenna installation yaw."""

import argparse
import io
import math
import os
import sys
from types import SimpleNamespace

import numpy as np

try:
    from scipy.optimize import least_squares
except ImportError:
    least_squares = None


PI = math.pi


def wrap_angle(angle):
    while angle > PI:
        angle -= 2.0 * PI
    while angle <= -PI:
        angle += 2.0 * PI
    return angle


def normalize_quaternion(quaternion):
    norm = np.linalg.norm(quaternion)
    if norm < 1.0e-12:
        raise ValueError("zero quaternion")
    return np.asarray(quaternion, dtype=float) / norm


def quaternion_to_matrix(quaternion):
    qx, qy, qz, qw = normalize_quaternion(quaternion)
    return np.array(
        [
            [1.0 - 2.0 * (qy * qy + qz * qz),
             2.0 * (qx * qy - qz * qw),
             2.0 * (qx * qz + qy * qw)],
            [2.0 * (qx * qy + qz * qw),
             1.0 - 2.0 * (qx * qx + qz * qz),
             2.0 * (qy * qz - qx * qw)],
            [2.0 * (qx * qz - qy * qw),
             2.0 * (qy * qz + qx * qw),
             1.0 - 2.0 * (qx * qx + qy * qy)],
        ],
        dtype=float,
    )


def matrix_to_rotvec(matrix):
    cosine = max(-1.0, min(1.0, 0.5 * (np.trace(matrix) - 1.0)))
    angle = math.acos(cosine)
    if angle < 1.0e-10:
        return np.zeros(3, dtype=float)
    sine = math.sin(angle)
    axis = np.array(
        [matrix[2, 1] - matrix[1, 2],
         matrix[0, 2] - matrix[2, 0],
         matrix[1, 0] - matrix[0, 1]],
        dtype=float,
    ) / (2.0 * sine)
    return axis * angle


def rotvec_to_matrix(rotvec):
    angle = float(np.linalg.norm(rotvec))
    if angle < 1.0e-12:
        return np.eye(3, dtype=float)
    axis = np.asarray(rotvec, dtype=float) / angle
    x, y, z = axis
    skew = np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])
    return (
        np.eye(3) * math.cos(angle)
        + (1.0 - math.cos(angle)) * np.outer(axis, axis)
        + math.sin(angle) * skew
    )


def read_tum(path):
    timestamps = []
    positions = []
    rotations = []
    with io.open(path, "r", encoding="ascii") as stream:
        for line_number, line in enumerate(stream, 1):
            stripped = line.strip()
            if not stripped or stripped.startswith("#"):
                continue
            fields = stripped.split()
            if len(fields) != 8:
                raise ValueError(
                    "{0}:{1}: TUM line must have 8 fields".format(path, line_number)
                )
            values = np.asarray([float(field) for field in fields], dtype=float)
            if not np.all(np.isfinite(values)):
                continue
            timestamps.append(values[0])
            positions.append(values[1:4])
            rotations.append(quaternion_to_matrix(values[4:8]))
    if not timestamps:
        raise RuntimeError("no valid poses in {0}".format(path))
    order = np.argsort(np.asarray(timestamps))
    return (
        np.asarray(timestamps, dtype=float)[order],
        np.asarray(positions, dtype=float)[order],
        np.asarray(rotations, dtype=float)[order],
    )


def pair_by_time(reference_times, query_times, max_dt):
    pairs = []
    query_index = 0
    for reference_index, reference_time in enumerate(reference_times):
        while (
            query_index + 1 < len(query_times)
            and abs(query_times[query_index + 1] - reference_time)
            <= abs(query_times[query_index] - reference_time)
        ):
            query_index += 1
        if query_index >= len(query_times):
            break
        difference = abs(query_times[query_index] - reference_time)
        if difference <= max_dt:
            pairs.append((reference_index, query_index, difference))
    return pairs


def kabsch(source, target):
    source_center = np.mean(source, axis=0)
    target_center = np.mean(target, axis=0)
    source_zero = source - source_center
    target_zero = target - target_center
    covariance = source_zero.T.dot(target_zero)
    left, _, right_transposed = np.linalg.svd(covariance)
    rotation = right_transposed.T.dot(left.T)
    if np.linalg.det(rotation) < 0.0:
        right_transposed[-1, :] *= -1.0
        rotation = right_transposed.T.dot(left.T)
    translation = target_center - rotation.dot(source_center)
    return rotation, translation


def estimate_solution(estimate_positions, estimate_rotations, truth_positions,
                      initial_lever_arm, robust_fscale):
    initial_rotation, initial_translation = kabsch(
        estimate_positions, truth_positions
    )
    initial = np.concatenate(
        [
            matrix_to_rotvec(initial_rotation),
            initial_translation,
            np.asarray(initial_lever_arm, dtype=float),
        ]
    )

    def residual(parameters):
        world_rotation = rotvec_to_matrix(parameters[0:3])
        world_translation = parameters[3:6]
        lever_arm = parameters[6:9]
        body_offsets_estimate = np.einsum(
            "nij,j->ni", estimate_rotations, lever_arm
        )
        predicted = (estimate_positions + body_offsets_estimate).dot(
            world_rotation.T
        ) + world_translation
        return (predicted - truth_positions).reshape(-1)

    if least_squares is not None:
        result = least_squares(
            residual,
            initial,
            loss="soft_l1",
            f_scale=max(1.0e-3, robust_fscale),
            max_nfev=3000,
        )
    else:
        result = SimpleNamespace(
            x=gauss_newton(
                residual,
                initial,
                max(1.0e-3, robust_fscale),
            )
        )
        result.fun = residual(result.x)
    return result, residual(result.x), initial_rotation, initial_translation


def robust_cost(residual, fscale):
    vectors = residual.reshape(-1, 3)
    norm_squared = np.sum(vectors * vectors, axis=1)
    scaled = norm_squared / (fscale * fscale)
    return 0.5 * np.sum((fscale * fscale) * (np.sqrt(1.0 + scaled) - 1.0))


def gauss_newton(residual_function, initial, fscale):
    """Small NumPy-only robust optimizer for the nine calibration parameters."""
    parameters = np.asarray(initial, dtype=float).copy()
    damping = 1.0e-3
    finite_difference = 1.0e-5
    for _ in range(80):
        residual = residual_function(parameters)
        current_cost = robust_cost(residual, fscale)
        jacobian = np.empty((residual.size, parameters.size), dtype=float)
        for column in range(parameters.size):
            perturbed = parameters.copy()
            perturbed[column] += finite_difference
            jacobian[:, column] = (
                residual_function(perturbed) - residual
            ) / finite_difference

        vector_norm = np.linalg.norm(residual.reshape(-1, 3), axis=1)
        weights = 1.0 / np.sqrt(1.0 + (vector_norm / fscale) ** 2)
        row_weights = np.repeat(np.sqrt(weights), 3)
        weighted_jacobian = jacobian * row_weights[:, None]
        weighted_residual = residual * row_weights
        normal = weighted_jacobian.T.dot(weighted_jacobian)
        normal += damping * np.diag(np.maximum(np.diag(normal), 1.0e-9))
        gradient = weighted_jacobian.T.dot(weighted_residual)
        try:
            step = np.linalg.solve(normal, -gradient)
        except np.linalg.LinAlgError:
            step = np.linalg.lstsq(normal, -gradient, rcond=None)[0]
        if not np.all(np.isfinite(step)) or np.linalg.norm(step) < 1.0e-9:
            break

        candidate = parameters + step
        candidate_cost = robust_cost(residual_function(candidate), fscale)
        if candidate_cost < current_cost:
            parameters = candidate
            damping = max(1.0e-8, damping * 0.5)
            if abs(current_cost - candidate_cost) < 1.0e-10:
                break
        else:
            damping = min(1.0e8, damping * 10.0)
    return parameters


def yaw_from_matrix(matrix):
    return math.atan2(matrix[1, 0], matrix[0, 0])


def circular_mean(angles):
    values = np.asarray(angles, dtype=float)
    return math.atan2(float(np.mean(np.sin(values))),
                     float(np.mean(np.cos(values))))


def circular_std(angles):
    values = np.asarray(angles, dtype=float)
    resultant = math.hypot(float(np.mean(np.sin(values))),
                           float(np.mean(np.cos(values))))
    return math.sqrt(max(0.0, -2.0 * math.log(max(resultant, 1.0e-12))))


def calculate_heading_offset(estimate_times, estimate_rotations,
                             yaw_times, yaw_rotations, world_rotation,
                             max_dt):
    pairs = pair_by_time(yaw_times, estimate_times, max_dt)
    offsets = []
    for yaw_index, estimate_index, _ in pairs:
        baseline_yaw = yaw_from_matrix(yaw_rotations[yaw_index])
        imu_world_truth = world_rotation.dot(estimate_rotations[estimate_index])
        imu_yaw = yaw_from_matrix(imu_world_truth)
        offsets.append(wrap_angle(baseline_yaw - imu_yaw))
    if not offsets:
        return None, 0
    offset = circular_mean(offsets)
    residuals = np.asarray([wrap_angle(value - offset) for value in offsets])
    inliers = np.abs(residuals) < math.radians(20.0)
    if np.count_nonzero(inliers) >= 3:
        offset = circular_mean(np.asarray(offsets)[inliers])
        residuals = np.asarray(
            [wrap_angle(value - offset) for value in np.asarray(offsets)[inliers]]
        )
        count = int(np.count_nonzero(inliers))
    else:
        count = len(offsets)
    return (offset, circular_std(residuals)), count


def make_parent(path):
    parent = os.path.dirname(os.path.abspath(path))
    if parent and not os.path.isdir(parent):
        os.makedirs(parent)


def write_yaml(path, lever_arm, heading_offset, metrics):
    make_parent(path)
    alpha_deg = math.degrees(heading_offset) if heading_offset is not None else 0.0
    baseline = (math.cos(heading_offset), math.sin(heading_offset), 0.0)
    with io.open(path, "w", encoding="ascii") as stream:
        stream.write("# Generated by calibrate_lever_arm_heading.py\n")
        stream.write("# IMU origin to RTK main antenna, expressed in IMU frame.\n")
        stream.write(
            "antenna_lever_arm_i: [{:.9f}, {:.9f}, {:.9f}]\n".format(*lever_arm)
        )
        stream.write("# CCW yaw from IMU +X to the THS main-to-secondary baseline.\n")
        stream.write(
            "heading_baseline_i: [{:.9f}, {:.9f}, {:.9f}]\n".format(*baseline)
        )
        stream.write("heading_offset_deg: {:.9f}\n".format(alpha_deg))
        stream.write("# calibration_pairs: {}\n".format(metrics["pairs"]))
        stream.write("# position_rmse_m: {:.6f}\n".format(metrics["rmse_m"]))
        stream.write("# position_p95_m: {:.6f}\n".format(metrics["p95_m"]))
        stream.write("# orientation_excitation_deg: {:.3f}\n".format(
            metrics["excitation_deg"]
        ))


def parse_arguments():
    parser = argparse.ArgumentParser(
        description="Calibrate IMU-to-RTK lever arm and THS installation yaw."
    )
    parser.add_argument("--estimate-tum", required=True,
                        help="FAST-LIO body/IMU-center TUM trajectory")
    parser.add_argument("--truth-tum", required=True,
                        help="second RTK main-antenna ENU TUM trajectory")
    parser.add_argument("--truth-yaw-tum", default="",
                        help="optional second RTK yaw-only TUM trajectory")
    parser.add_argument("--output-yaml", required=True)
    parser.add_argument("--max-dt", type=float, default=0.05)
    parser.add_argument("--time-offset", type=float, default=0.0,
                        help="added to truth timestamps before association")
    parser.add_argument("--initial-lever-arm", nargs=3, type=float,
                        default=(0.0, 0.0, 0.0), metavar=("X", "Y", "Z"))
    parser.add_argument("--robust-fscale", type=float, default=0.10)
    return parser.parse_args()


def main():
    args = parse_arguments()
    estimate_times, estimate_positions, estimate_rotations = read_tum(
        args.estimate_tum
    )
    truth_times, truth_positions, _ = read_tum(args.truth_tum)
    truth_times = truth_times + args.time_offset

    pairs = pair_by_time(truth_times, estimate_times, args.max_dt)
    if len(pairs) < 20:
        raise RuntimeError(
            "only {} synchronized pairs; need at least 20".format(len(pairs))
        )
    truth_indices = np.asarray([pair[0] for pair in pairs], dtype=int)
    estimate_indices = np.asarray([pair[1] for pair in pairs], dtype=int)
    paired_truth_positions = truth_positions[truth_indices]
    paired_estimate_positions = estimate_positions[estimate_indices]
    paired_estimate_rotations = estimate_rotations[estimate_indices]

    reference_rotation = paired_estimate_rotations[0]
    excitation = np.max(
        [
            math.degrees(
                math.acos(
                    max(
                        -1.0,
                        min(
                            1.0,
                            0.5
                            * (
                                np.trace(reference_rotation.T.dot(rotation))
                                - 1.0
                            ),
                        ),
                    )
                )
            )
            for rotation in paired_estimate_rotations
        ]
    )
    if excitation < 5.0:
        print(
            "WARNING: orientation excitation is only {:.2f} deg; lever arm is "
            "poorly observable.".format(excitation),
            file=sys.stderr,
        )

    result, initial_residual, _, _ = estimate_solution(
        paired_estimate_positions,
        paired_estimate_rotations,
        paired_truth_positions,
        args.initial_lever_arm,
        args.robust_fscale,
    )
    optimized_rotation = rotvec_to_matrix(result.x[0:3])
    optimized_lever_arm = result.x[6:9]
    residual_vectors = result.fun.reshape(-1, 3)
    residual_norms = np.linalg.norm(residual_vectors, axis=1)
    metrics = {
        "pairs": len(pairs),
        "rmse_m": float(math.sqrt(np.mean(residual_norms ** 2))),
        "p95_m": float(np.percentile(residual_norms, 95.0)),
        "excitation_deg": float(excitation),
    }

    heading_result = None
    heading_count = 0
    if args.truth_yaw_tum:
        yaw_times, _, yaw_rotations = read_tum(args.truth_yaw_tum)
        yaw_times = yaw_times + args.time_offset
        heading_result, heading_count = calculate_heading_offset(
            estimate_times,
            estimate_rotations,
            yaw_times,
            yaw_rotations,
            optimized_rotation,
            args.max_dt,
        )
        if heading_result is None:
            print("WARNING: no synchronized yaw pairs; heading offset not estimated.",
                  file=sys.stderr)
    if heading_result is None:
        heading_offset = 0.0
        heading_std = float("nan")
    else:
        heading_offset, heading_std = heading_result

    write_yaml(args.output_yaml, optimized_lever_arm, heading_offset, metrics)

    print("synchronized pairs: {}".format(metrics["pairs"]))
    print("position RMSE: {:.4f} m".format(metrics["rmse_m"]))
    print("position P95: {:.4f} m".format(metrics["p95_m"]))
    print("orientation excitation: {:.2f} deg".format(metrics["excitation_deg"]))
    print("lever arm IMU->RTK antenna [m]: [{:.6f}, {:.6f}, {:.6f}]".format(
        *optimized_lever_arm
    ))
    if heading_result is not None:
        print("heading offset: {:.4f} deg ({} pairs, circular std {:.4f} deg)".format(
            math.degrees(heading_offset), heading_count, math.degrees(heading_std)
        ))
    print("YAML written: {}".format(args.output_yaml))


if __name__ == "__main__":
    try:
        main()
    except Exception as error:
        print("ERROR: {}".format(error), file=sys.stderr)
        sys.exit(1)
