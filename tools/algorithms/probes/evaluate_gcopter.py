#!/usr/bin/env python3
# Copyright 2026 EasyFly Project
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Audit recorded real GCOPTER commands, map provenance and native obstacle clearance."""
import argparse
import csv
import gzip
import hashlib
import json
import math
from pathlib import Path

import numpy as np


def rows(path):
    return [json.loads(line) for line in path.read_text().splitlines() if line]


def evaluate(plan, source_ns, derivative=0):
    time = max(0.0, min((source_ns - plan["stamp_ns"]) * 1e-9, sum(plan["durations"])))
    for index, duration in enumerate(plan["durations"]):
        if time <= duration or index == len(plan["durations"]) - 1:
            coefficients = np.asarray(plan["coefficients"][index * 18 : (index + 1) * 18]).reshape(
                3, 6
            )
            return np.array([np.polyval(np.polyder(c, derivative), time) for c in coefficients])
        time -= duration
    raise ValueError("empty polynomial")


def maximum(plan, derivative):
    result = 0.0
    for index, duration in enumerate(plan["durations"]):
        coefficients = np.asarray(plan["coefficients"][index * 18 : (index + 1) * 18]).reshape(
            3, 6
        )
        squared = np.array([0.0])
        for c in coefficients:
            d = np.polyder(c, derivative)
            squared = np.polyadd(squared, np.polymul(d, d))
        roots = np.roots(np.polyder(squared))
        candidates = [0.0, duration] + [
            r.real for r in roots if abs(r.imag) < 1e-7 and 0.0 < r.real < duration
        ]
        result = max(result, math.sqrt(max(0.0, max(np.polyval(squared, t) for t in candidates))))
    return result


def audit(directory):
    result = json.loads((directory / "result.json").read_text())
    plans = rows(directory / "polynomials.jsonl")
    commands = rows(directory / "commands.jsonl")
    states = rows(directory / "states.jsonl")
    limits = result["acceptance_limits"]
    failures = []
    if result.get("diagnostic_only"):
        failures.append("diagnostic run is not acceptance evidence")
    if result.get("strategy") != "gcopter":
        failures.append("legacy baseline requires its own evaluation")
    if not result["passed"]:
        failures.append("managed simulation did not pass")
    lookup = {(p["session"], p["trajectory_id"]): p for p in plans}
    with gzip.open(directory / "voxel-grid.jsonl.gz", "rt") as stream:
        maps = {(m["session"], m["generation"], m["revision"]): m for m in map(json.loads, stream)}
    dynamics = []
    for plan in plans:
        source = maps.get((plan["map_session"], plan["map_generation"], plan["map_revision"]))
        checked = maps.get(
            (plan["map_session"], plan["map_generation"], plan["validated_map_revision"])
        )
        if (
            not source
            or not checked
            or source["stamp_ns"] != plan["map_stamp_ns"]
            or checked["stamp_ns"] != plan["validated_map_stamp_ns"]
        ):
            failures.append("missing or mismatched source map receipt")
        maxima = [maximum(plan, k) for k in [1, 2, 3]]
        dynamics.append(
            dict(
                session=plan["session"],
                duration_s=sum(plan["durations"]),
                max_speed=maxima[0],
                max_acceleration=maxima[1],
                max_jerk=maxima[2],
            )
        )
        if any(
            v > bound + 1e-6
            for v, bound in zip(
                maxima,
                [
                    limits["planned_speed_mps"],
                    limits["max_acceleration_mps2"],
                    limits["max_jerk_mps3"],
                ],
            )
        ):
            failures.append("independent polynomial dynamics exceeded")
    command_error = 0.0
    for command in commands:
        plan = lookup[(command["session"], command["trajectory_id"])]
        for derivative, field in enumerate(["position", "velocity", "acceleration"]):
            command_error = max(
                command_error,
                float(
                    np.linalg.norm(
                        evaluate(plan, command["stamp_ns"], derivative) - command[field]
                    )
                ),
            )
    if command_error > 1e-6:
        failures.append("command differs from scheduled polynomial")
    tracking = []
    for plan in plans:
        session_commands = [c for c in commands if c["session"] == plan["session"]]
        if not session_commands:
            failures.append("published polynomial without observed commands")
            continue
        begin, end = min(c["stamp_ns"] for c in session_commands), max(
            c["stamp_ns"] for c in session_commands
        )
        tracking += [
            float(np.linalg.norm(evaluate(plan, state["source_ns"]) - state["position"]))
            for state in states
            if begin <= state["source_ns"] <= end
        ]
    if tracking and max(tracking) > limits["tracking_error_m"] + 1e-6:
        failures.append("recorded source-time tracking error exceeded")
    truth = []
    with (directory / "truth.csv").open() as stream:
        truth = list(csv.DictReader(line for line in stream if not line.startswith("#")))
    clearance = None
    native_altitude_min = None
    crossing = []
    if commands:
        begin, end = min(c["stamp_ns"] for c in commands), max(c["stamp_ns"] for c in commands)
        flight = [t for t in truth if begin <= int(t["stamp_ns"]) <= end]
        navigating = [s for s in states if begin <= s["source_ns"] <= end]
        if any(
            not limits["flight_z_min_m"] <= s["position"][2] <= limits["flight_z_max_m"]
            for s in navigating
        ):
            failures.append("measured navigation altitude outside frozen bounds")
        if not flight:
            failures.append("missing independent native flight truth")
        else:
            native_altitude_min = min(float(t["z"]) for t in flight)
            if native_altitude_min < limits["collision_body_bounding_sphere_m"]:
                failures.append("native vehicle bounding sphere intersects the physical floor")
        for obstacle in result.get("obstacles", []):
            center, size = np.array(obstacle["center"]), np.array(obstacle["size"])
            points = np.array([[float(t[k]) for k in ["x", "y", "z"]] for t in flight])
            distances = np.linalg.norm(
                np.maximum(np.abs(points - center) - size * 0.5, 0.0), axis=1
            )
            distance = float(min(distances))
            clearance = distance if clearance is None else min(clearance, distance)
            if distance < limits["independent_model_center_to_obstacle_aabb_m"]:
                failures.append("native physical obstacle clearance exceeded")
            if size[0] < 0.3:
                samples = points[np.abs(points[:, 0] - center[0]) <= 0.05]
                crossing.append(
                    dict(
                        obstacle=obstacle["name"],
                        count=len(samples),
                        z_min=float(min(samples[:, 2])) if len(samples) else None,
                        z_max=float(max(samples[:, 2])) if len(samples) else None,
                    )
                )
                if result["scenario"] in ["over", "under", "channel", "map-change"] and not len(
                    samples
                ):
                    failures.append("no actual obstacle-plane crossing")
                if len(samples) and result["scenario"] == "map-change":
                    if min(np.abs(samples[:, 1] - center[1])) < size[1] * 0.5 + 0.35:
                        failures.append("missing actual lateral bypass of the added panel")
                if len(samples) and result["scenario"] in [
                    "over",
                    "under",
                    "channel",
                ]:
                    if max(np.abs(samples[:, 1] - center[1])) > size[1] * 0.5 - 0.35:
                        failures.append("crossing bypassed the physical wall laterally")
                    lower = "lower" in obstacle["name"] or obstacle["name"].endswith("_low")
                    if lower and min(samples[:, 2]) < center[2] + size[2] * 0.5 + 0.35:
                        failures.append("missing independent over-wall clearance")
                    if not lower and max(samples[:, 2]) > center[2] - size[2] * 0.5 - 0.35:
                        failures.append("missing independent under-wall clearance")
        if result["scenario"] == "narrow":
            points = np.array([[float(t[k]) for k in ["x", "y", "z"]] for t in flight])
            corridor = points[(points[:, 0] >= 0.5) & (points[:, 0] <= 3.5)]
            if not len(corridor) or max(np.abs(corridor[:, 1])) > 1.1 - 0.35:
                failures.append("flight did not traverse the physical narrow corridor")
    output = dict(
        evaluator_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        case_result_sha256=hashlib.sha256((directory / "result.json").read_bytes()).hexdigest(),
        passed=not failures,
        failures=failures,
        polynomial_count=len(plans),
        independent_dynamics=dynamics,
        command_polynomial_error=command_error,
        source_time_tracking_max_m=max(tracking) if tracking else None,
        native_obstacle_clearance_min_m=clearance,
        native_navigation_altitude_min_m=native_altitude_min,
        actual_plane_crossings=crossing,
        native_truth_samples=len(truth),
        hardware=False,
    )
    (directory / "independent-evaluation.json").write_text(json.dumps(output, indent=2) + "\n")
    return output


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    arguments = parser.parse_args()
    report = audit(arguments.directory)
    print(json.dumps(report, indent=2))
    raise SystemExit(0 if report["passed"] else 1)
