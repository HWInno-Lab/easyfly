#!/usr/bin/env python3
# Copyright 2026 EasyFly Project
# SPDX-License-Identifier: Apache-2.0
"""Explicit ALG-9 real-perception follow/cancel SITL; never an automatic test.

Reuse the installed navigation probe's private PX4 staging, live state checks,
exact cancellation, result draining and process shutdown checks. Do not publish
targets, alter perception stamps, change safety parameters or connect hardware.
"""
import argparse
import hashlib
from importlib.machinery import SourceFileLoader
import importlib.util
import json
import math
from pathlib import Path
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--px4-dir", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--domain", type=int, required=True)
    parser.add_argument("--gazebo-port", type=int, required=True)
    args = parser.parse_args()
    if not 1 <= args.domain <= 232 or not 1024 <= args.gazebo_port <= 65535:
        parser.error("use an explicit nonzero DDS domain and valid local port")
    from ament_index_python.packages import get_package_prefix
    from easyfly_msgs.action import Land, Takeoff, TrackTarget
    from easyfly_msgs.msg import Target3DArray
    from rclpy.qos import qos_profile_sensor_data

    entry = (
        Path(get_package_prefix("easyfly_sim_launcher"))
        / "lib/easyfly_sim_launcher/navigation_sitl"
    )
    loader = SourceFileLoader("alg9_installed_navigation_probe", str(entry))
    spec = importlib.util.spec_from_loader(loader.name, loader)
    navigation = importlib.util.module_from_spec(spec)
    loader.exec_module(navigation)
    args.strategy = "astar"
    args.scenario = "real-perception-follow-cancel"
    args.semantic_capture_shutdown = False
    args.lifecycle_preparation = "isolated"

    class FollowProbe(navigation.Probe):
        def __init__(self):
            super().__init__(args)
            self.targets = None
            self.target_receipt = 0.0
            self.follow_feedback = []
            self.target_samples = []
            self.report["follow_probe_sha256"] = hashlib.sha256(
                Path(__file__).read_bytes()
            ).hexdigest()
            self.report[
                "target_source"
            ] = "unaltered live /target_positions from installed perception"
            self.node.create_subscription(
                Target3DArray,
                "/target_positions",
                self.receive_targets,
                qos_profile_sensor_data,
            )
            self.node.create_subscription(
                TrackTarget.Impl.FeedbackMessage,
                "/track_target/_action/feedback",
                self.receive_feedback,
                qos_profile_sensor_data,
            )

        def receive_targets(self, msg):
            self.targets, self.target_receipt = msg, time.monotonic()
            if len(self.target_samples) < 10000:
                self.target_samples.append(
                    {
                        "steady": self.target_receipt,
                        "source_ns": msg.header.stamp.sec * 1000000000
                        + msg.header.stamp.nanosec,
                        "frame": msg.header.frame_id,
                        "targets": [
                            {
                                "id": t.track_id,
                                "valid": t.position_valid,
                                "position": [
                                    t.position_x,
                                    t.position_y,
                                    t.position_z,
                                ],
                            }
                            for t in msg.targets
                        ],
                    }
                )

        def receive_feedback(self, msg):
            if len(self.follow_feedback) < 10000:
                self.follow_feedback.append(
                    {
                        "steady": time.monotonic(),
                        "goal_uuid": bytes(msg.goal_id.uuid).hex(),
                        "target_id": msg.feedback.current_target_id,
                        "visible": msg.feedback.target_visible,
                        "distance": msg.feedback.distance_to_target,
                    }
                )

        def candidates(self):
            msg = self.targets
            now = self.node.get_clock().now().nanoseconds
            if msg is None or msg.header.frame_id != "odom":
                return []
            stamp = (
                msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec
            )
            if (
                stamp <= 0
                or not -0.2 <= (now - stamp) * 1e-9 <= 1.0
                or not 0 <= time.monotonic() - self.target_receipt <= 0.3
            ):
                return []
            state = self.state()
            if state is None:
                return []
            return [
                t
                for t in msg.targets
                if t.position_valid
                and t.track_id >= 0
                and all(
                    math.isfinite(v)
                    for v in (t.position_x, t.position_y, t.position_z)
                )
                and 4.5
                <= math.hypot(
                    t.position_x - state["position"][0],
                    t.position_y - state["position"][1],
                )
                <= 30.0
            ]

        def run(self):
            self.start()
            takeoff = self.send(
                "/takeoff",
                Takeoff,
                Takeoff.Goal(target_altitude=1.5, climb_speed=0.3),
            )
            self.finish(takeoff, seconds=75)
            self.stable_hover(altitude=1.5)
            self.wait(lambda: bool(self.candidates()), 20)
            self.report["target_publishers"] = [
                {
                    "node": p.node_name,
                    "namespace": p.node_namespace,
                    "type": p.topic_type,
                }
                for p in self.node.get_publishers_info_by_topic(
                    "/target_positions"
                )
            ]
            target = min(
                self.candidates(),
                key=lambda t: math.hypot(
                    t.position_x - self.state()["position"][0],
                    t.position_y - self.state()["position"][1],
                ),
            )
            self.report["selected_target"] = {
                "id": target.track_id,
                "position": [
                    target.position_x,
                    target.position_y,
                    target.position_z,
                ],
            }
            initial = self.state()["position"][:2]
            before = set(self.report["action_status"]["/goto_waypoint"])
            follow = self.send(
                "/track_target",
                TrackTarget,
                TrackTarget.Goal(
                    target_id=target.track_id,
                    follow_distance=4.0,
                    follow_height=1.5,
                ),
            )

            def moved_with_feedback():
                if follow["result"].done():
                    raise RuntimeError(
                        "follow ended before motion and exact cancel"
                    )
                visible = any(
                    v["goal_uuid"] == follow["entry"]["goal_uuid"]
                    and v["visible"]
                    and 0 <= time.monotonic() - v["steady"] <= 0.5
                    for v in self.follow_feedback
                )
                statuses = self.report["action_status"]["/goto_waypoint"]
                children = set(statuses) - before
                return (
                    visible
                    and any(
                        statuses[identity]["status"] == 2
                        for identity in children
                    )
                    and self.hover_or_follow_motion(initial)
                )

            self.wait(moved_with_feedback, 25)
            self.report["follow_motion_state"] = self.state()
            self.cancel_exact(follow)
            self.report["follow_cancel"] = self.report.pop("mission_cancel")
            self.finish(follow, expected=5, seconds=40)
            children = (
                set(self.report["action_status"]["/goto_waypoint"]) - before
            )
            if not children:
                raise RuntimeError("real follow child UUID missing")
            for identity in children:
                self.child_result("/goto_waypoint", identity)
            self.stable_hover()
            self.observe_for(1.0)
            if (
                set(self.report["action_status"]["/goto_waypoint"]) - before
                != children
            ):
                raise RuntimeError(
                    "late follow child after parent cancellation"
                )
            land = self.send("/land", Land, Land.Goal())
            self.finish(land, seconds=90)
            self.wait(lambda: self.ready(False), 15)
            self.report["final_disarmed_state"] = self.state()

        def hover_or_follow_motion(self, initial):
            return (
                self.ready(True)
                and self.state()["mode"] == 3
                and math.dist(self.state()["position"][:2], initial) >= 0.15
            )

        def close(self):
            self.report["target_samples"] = self.target_samples
            self.report["follow_feedback"] = self.follow_feedback
            super().close()

    probe = FollowProbe()
    try:
        probe.run()
    except (Exception, KeyboardInterrupt) as error:
        probe.report["error"] = type(error).__name__ + ": " + str(error)
        print(probe.report["error"], flush=True)
    finally:
        probe.close()
    print(
        json.dumps(
            {
                "passed": probe.report["passed"],
                "result": str(probe.directory / "result.json"),
            }
        )
    )
    return 0 if probe.report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
