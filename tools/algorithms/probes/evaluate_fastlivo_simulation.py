#!/usr/bin/env python3
# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Independent source-time evaluation, frozen first-pose yaw/translation only."""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import numpy as np


def multiply(a, b):
    return np.r_[a[3] * b[:3] + b[3] * a[:3] + np.cross(a[:3], b[:3]), a[3] * b[3] - a[:3] @ b[:3]]


def yaw(q):
    x, y, z, w = q
    return np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def evaluate(directory, fastlio=False):
    result = json.loads((directory / "result.json").read_text())
    limits = json.loads((directory.parent / "acceptance-limits.json").read_text())
    rows = [json.loads(l) for l in (directory / "vio.jsonl").read_text().splitlines()]
    poses = [p for p in rows if p["stream"] in ("aligned_pose", "local_body_pose")]
    if fastlio:
        poses = [
            json.loads(l)
            for l in (directory / "fastlio-comparison.jsonl").read_text().splitlines()
        ]
    with (directory / "truth.csv").open() as f:
        metadata = f.readline()
        truth = list(csv.DictReader(f))
    if result.get("scenario") == "clock-reset":
        # Score only source-time accuracy before the deliberately injected epoch reset.
        cutoff = result["injection"]["steady"]
        truth = [r for r in truth if int(r["mono_ns"]) / 1e9 <= cutoff]
        poses = [r for r in poses if r["receipt"] <= cutoff]

    ts = np.array([int(r["stamp_ns"]) for r in truth], dtype=np.int64)
    if (np.diff(ts) < 0).any():
        raise ValueError("truth rollback")
    keep = np.r_[True, np.diff(ts) > 0]
    ts = ts[keep]
    xyz = np.array([[float(r[k]) for k in ("x", "y", "z")] for r in truth])[keep]
    qs = np.array([[float(r[k]) for k in ("qx", "qy", "qz", "qw")] for r in truth])[keep]
    receipts = np.array([int(r["mono_ns"]) / 1e9 for r in truth])[keep]
    errors = []
    angles = []
    delays = []
    stamps = []
    positions = []
    rotation = shift = qalign = None
    missed = 0
    for p in poses:
        i = np.searchsorted(ts, p["stamp_ns"], side="right")
        if not 0 < i < len(ts) or ts[i] - ts[i - 1] > 0.15e9:
            missed += 1
            continue
        f = (p["stamp_ns"] - ts[i - 1]) / (ts[i] - ts[i - 1])
        x = xyz[i - 1] + f * (xyz[i] - xyz[i - 1])
        q0, q1 = qs[i - 1], qs[i]
        if q0 @ q1 < 0:
            q1 = -q1
        q = q0 + f * (q1 - q0)
        q /= np.linalg.norm(q)
        if rotation is None:
            angle = yaw(q) - yaw(p["quaternion"])
            c, s = np.cos(angle), np.sin(angle)
            rotation = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
            shift = x - rotation @ p["position"]
            qalign = np.array([0, 0, np.sin(angle / 2), np.cos(angle / 2)])
        measured = rotation @ p["position"] + shift
        errors.append(np.linalg.norm(measured - x))
        stamps.append(p["stamp_ns"])
        positions.append(x)
        angles.append(
            2 * np.arccos(min(1.0, abs(multiply(qalign, np.array(p["quaternion"])) @ q)))
        )
        delays.append(p["receipt"] - (receipts[i - 1] + f * (receipts[i] - receipts[i - 1])))
    if len(errors) < 2:
        raise ValueError("insufficient independent matches")
    tracks = [
        r["tracked_features"]
        for r in rows
        if r["stream"] == "visual_tracking" and r["initialized"]
    ]
    checks = dict(
        actual_runtime_passed=result["passed"],
        truth_no_discard="discarded=0" in metadata,
        sufficient_matches=len(errors) >= 100,
        coverage=len(errors) / len(poses) >= 0.95,
        position_rmse=np.sqrt(np.mean(np.square(errors))) <= limits["position_rmse_m"],
        position_max=max(errors) <= limits["position_max_m"],
        full_attitude_max=max(angles) <= limits["attitude_error_rad"],
        actual_visual_updates=sum(v >= 8 for v in tracks) >= 100,
        actual_motion=np.linalg.norm(np.ptp(positions, axis=0)) >= 0.5,
        source_span=(stamps[-1] - stamps[0]) / 1e9 >= 10,
    )
    obstacle = None
    if result.get("scenario") == "obstacle":
        config = result["obstacle"]
        center = np.asarray(config["center_world"][:2])
        route = xyz[(ts >= config["first_source_ns"]) & (xyz[:, 2] > 0.6)]
        direction = np.asarray(config["nominal_route_direction"], dtype=float)
        direction /= np.linalg.norm(direction)
        perpendicular = np.array([-direction[1], direction[0]])
        relative = route[:, :2] - center
        along = relative @ direction
        minimum = float(np.linalg.norm(relative, axis=1).min())
        obstacle = dict(
            minimum_horizontal_center_distance_m=minimum,
            maximum_lateral_detour_m=float(np.abs(relative @ perpendicular).max()),
            minimum_along_route_m=float(along.min()),
            maximum_along_route_m=float(along.max()),
            maximum_altitude_m=float(route[:, 2].max()),
        )
        checks.update(
            actual_obstacle_clearance=minimum
            >= config["required_horizontal_model_center_clearance_m"],
            actual_bypass_both_sides=along.min() < -0.8 and along.max() > 0.8,
            actual_horizontal_detour=obstacle["maximum_lateral_detour_m"] >= 0.7,
            stayed_below_obstacle_top=obstacle["maximum_altitude_m"] <= 2.2,
        )
    if fastlio:
        checks.pop("actual_visual_updates")
    return dict(
        backend="fast_lio" if fastlio else "fast_livo2",
        passed=all(checks.values()),
        checks={k: bool(v) for k, v in checks.items()},
        matched=len(errors),
        missed=missed,
        source_span_s=(stamps[-1] - stamps[0]) / 1e9,
        position_rmse_m=float(np.sqrt(np.mean(np.square(errors)))),
        position_max_m=float(max(errors)),
        attitude_max_rad=float(max(angles)),
        visual_updates_ge8=None if fastlio else sum(v >= 8 for v in tracks),
        visual_patch_count_quantiles=None
        if fastlio
        else np.quantile(tracks, [0, 0.5, 0.95, 1]).tolist(),
        receipt_delay_quantiles_s=(
            None
            if result.get("scope") == "offline_replay_not_flight"
            else np.quantile(delays, [0, 0.5, 0.95, 1]).tolist()
        ),
        alignment="first pose yaw + translation, fixed scale; no trajectory fitting",
        result_sha256=hashlib.sha256((directory / "result.json").read_bytes()).hexdigest(),
        evaluator_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        obstacle=obstacle,
        hardware=False,
    )


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("directory", type=Path)
    p.add_argument("--compare-fastlio", action="store_true")
    a = p.parse_args()
    d = evaluate(a.directory, a.compare_fastlio)
    (
        a.directory
        / ("fastlio-comparison-evaluation.json" if a.compare_fastlio else "evaluation.json")
    ).write_text(json.dumps(d, indent=2) + "\n")
    print(json.dumps(d, indent=2))
    return 0 if d["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
