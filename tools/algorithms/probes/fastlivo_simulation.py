#!/usr/bin/env python3
# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Explicit real FAST-LIVO2 Gazebo acceptance, native truth is read-only."""
import argparse
import importlib.util
import json
import os
from pathlib import Path
import signal
import time
import subprocess
import hashlib
import math

SPEC = importlib.util.spec_from_file_location(
    "fastlivo_common", Path(__file__).with_name("openvins_simulation.py")
)
common = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(common)
navigation = common.navigation
from diagnostic_msgs.msg import DiagnosticArray
from easyfly_msgs.msg import LocalizationStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from rclpy.qos import DurabilityPolicy, QoSProfile


class Probe(common.Probe):
    def __init__(self, args):
        from easyfly_fastdds_compat.runtime import activate

        middleware = activate(openvins=True)
        super().__init__(args)
        self.environment["LD_LIBRARY_PATH"] = os.environ["LD_LIBRARY_PATH"]
        self.report["dds_compatibility"] = middleware
        self.report["backend"] = "fast_livo2"
        self.report["scene"] = args.scene
        limits = (
            Path(__file__).resolve().parents[3]
            / "artifacts/new-algorithms/new3-fastlivo2-20261021/acceptance-limits.json"
        )
        self.report["acceptance_limits"] = json.loads(limits.read_text())
        self.report["acceptance_limits_sha256"] = hashlib.sha256(limits.read_bytes()).hexdigest()
        root = Path(__file__).resolve().parents[3]
        snapshot = self.directory / "source-snapshot" / "fastlivo"
        files = [Path(__file__), Path(__file__).with_name("openvins_simulation.py")]
        for directory in [
            "src/perception/p3_tracking/easyfly_fast_livo2_integration",
            "src/third_party/algorithms/localization/fast_livo2",
        ]:
            files.extend(
                p
                for p in (root / directory).rglob("*")
                if p.is_file() and "__pycache__" not in p.parts
            )
        self.report["fastlivo_source_hashes"] = {}
        for source in files:
            relative = source.relative_to(root)
            target = snapshot / relative
            target.parent.mkdir(parents=True, exist_ok=True)
            data = source.read_bytes()
            target.write_bytes(data)
            self.report["fastlivo_source_hashes"][str(relative)] = hashlib.sha256(data).hexdigest()

        self.relays = []
        self.extra_processes = []
        self.fastlio_log = (self.directory / "fastlio-comparison.jsonl").open("x")
        from nav_msgs.msg import Odometry
        from easyfly_openvins_integration.geometry import rotation

        def fastlio_pose(m):
            p, q = m.pose.pose.position, m.pose.pose.orientation
            quat = [q.x, q.y, q.z, q.w]
            base = rotation(quat) @ [0.011, 0.02329, -0.12412] + [p.x, p.y, p.z]
            self.fastlio_log.write(
                json.dumps(
                    dict(
                        stream="aligned_pose",
                        stamp_ns=m.header.stamp.sec * 10 ** 9 + m.header.stamp.nanosec,
                        position=base.tolist(),
                        quaternion=quat,
                        receipt=time.monotonic(),
                        frame="fastlio_comparison",
                    )
                )
                + "\n"
            )

        self.node.create_subscription(Odometry, "/comparison/fastlio/odometry", fastlio_pose, 100)
        self.relay_counts = dict(camera=0, imu=0, lidar=0)
        self.original_counts = dict(camera=0, imu=0, lidar=0)
        from sensor_msgs.msg import Image, Imu
        from livox_ros_driver2.msg import CustomMsg

        if args.scenario == "localization-loss":
            from rclpy.qos import qos_profile_sensor_data

            self.node.create_subscription(
                Imu,
                "/livox/imu",
                lambda m: self.original_counts.__setitem__("imu", self.original_counts["imu"] + 1),
                qos_profile_sensor_data,
            )
        self.report.pop("openvins_provenance_sha256", None)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        for topic, cls in [
            ("status", LocalizationStatus),
            ("alignment_status", LocalizationStatus),
            ("navigation_status", LocalizationStatus),
            ("pose", PoseWithCovarianceStamped),
            ("aligned_pose", PoseStamped),
        ]:
            self.node.create_subscription(
                cls,
                "/localization/fast_livo2/" + topic,
                lambda m, k=topic: self.on_vio(k, m),
                qos if "status" in topic else 100,
            )
        self.node.create_subscription(
            DiagnosticArray, "/fastlivo/raw/visual_tracking", self.on_tracking, 32
        )
        for key in [
            "EASYFLY_GUARD_TAKE_TRACE",
            "EASYFLY_GUARD_CLOCK_TRACE",
            "EASYFLY_GUARD_FRESHNESS_TRACE",
        ]:
            self.environment.pop(key, None)

    def run(self):
        super().run()
        self.report["initialization_bootstrap"] = (
            "real on-ground LiDAR/IMU/visual initialization followed by explicit PX4 flight; "
            "no injected truth state or synthetic observations"
        )

    def scene_origin(self):
        if self.args.scene != "gcopter_3d_lane_v1":
            return super().scene_origin()
        from ament_index_python.packages import get_package_share_directory
        import yaml

        path = (
            Path(get_package_share_directory("easyfly_gazebo_worlds"))
            / "scenes/gcopter_3d_lane_v1.yaml"
        )
        data = yaml.safe_load(path.read_text())
        if data["spawn"]["yaw"] != 0.0:
            raise RuntimeError("controlled fixture requires the qualified zero-yaw lane spawn")
        self.report["injection_scene_manifest_sha256"] = hashlib.sha256(
            path.read_bytes()
        ).hexdigest()
        return data["spawn"]["position"]

    def launch_command(self):
        if self.args.record_inputs:
            log = (self.directory / "bag-recorder.log").open("x")
            command = [
                "ros2",
                "bag",
                "record",
                "-o",
                str(self.directory / "input-bag"),
                "/clock",
                "/livox/lidar",
                "/livox/imu",
                "/camera/color/image_raw",
                "/camera/color/camera_info",
                "/fastlivo/input/camera",
                "/fastlivo/input/imu",
                "/fastlivo/input/lidar",
            ]
            proc = subprocess.Popen(
                command, env=self.environment, stdout=log, stderr=subprocess.STDOUT
            )
            self.extra_processes.append(("bag", proc, log))
        if self.args.compare_fastlio:
            from ament_index_python.packages import get_package_prefix, get_package_share_directory

            exe = Path(get_package_prefix("fast_lio")) / "lib/fast_lio/fastlio_mapping"
            config = Path(get_package_share_directory("fast_lio")) / "config/mid360.yaml"
            extrinsic = (
                Path(get_package_share_directory("easyfly_fast_lio_integration"))
                / "config/gazebo_mid360_extrinsics.yaml"
            )
            command = [
                "nice",
                "-n",
                "10",
                str(exe),
                "--ros-args",
                "-r",
                "__node:=fastlio_comparison",
                "--params-file",
                str(config),
                "--params-file",
                str(extrinsic),
                "-p",
                "use_sim_time:=true",
                "-p",
                "publish.scan_publish_en:=false",
                "-p",
                "publish.map_en:=false",
                "-p",
                "publish.path_en:=false",
            ]
            for topic in [
                "/Odometry",
                "/cloud_registered",
                "/cloud_registered_body",
                "/cloud_effected",
                "/Laser_map",
                "/path",
                "/tf",
                "/tf_static",
            ]:
                command += [
                    "-r",
                    topic
                    + ":=/comparison/fastlio/"
                    + ("odometry" if topic == "/Odometry" else topic[1:]),
                ]
            log = (self.directory / "fastlio-comparison.log").open("x")
            proc = subprocess.Popen(
                command, env=self.environment, stdout=log, stderr=subprocess.STDOUT
            )
            self.extra_processes.append(("fastlio_comparison", proc, log))
            self.report["comparison"] = dict(
                same_raw_sensor_topics=True,
                control_input=False,
                pid=proc.pid,
                command=command,
                executable_sha256=hashlib.sha256(exe.read_bytes()).hexdigest(),
                original_fastlio_point_time_interpretation=True,
            )
        if self.args.managed_profile:
            return [
                "ros2",
                "launch",
                "easyfly_sim_launcher",
                "profile.launch.py",
                "profile:=navigation_dev",
                "fastlivo_navigation:=true",
                "algorithms_json:="
                + json.dumps(dict(slam="fast_livo2", strategy=self.args.strategy, provider="cpu")),
                "scene:=" + self.args.scene,
                "headless:=true",
                "rviz:=false",
            ]
        command = [
            "ros2",
            "launch",
            "easyfly_px4_sitl",
            "perception_sitl.launch.py",
            "localization_backend:=fast_livo2",
            "fastlivo_navigation:=true",
            "scene:=" + self.args.scene,
            "scene_profile:=navigation_dev",
            "navigation_strategy:=" + self.args.strategy,
            "headless:=true",
            "rviz:=false",
            "detection_use_gpu:=false",
        ]
        if self.args.scenario == "localization-loss":
            for kind in ["camera", "imu", "lidar"]:
                log = (self.directory / ("relay-" + kind + ".log")).open("x")
                proc = subprocess.Popen(
                    [
                        "python3",
                        str(Path(__file__).with_name("fastlivo_sensor_relay.py")),
                        kind,
                        str(self.directory / ("relay-" + kind + ".jsonl")),
                    ],
                    env=self.environment,
                    stdout=log,
                    stderr=subprocess.STDOUT,
                )
                self.relays.append((kind, proc, log))
                command.append("fastlivo_" + kind + "_topic:=/acceptance/" + kind)
        return command

    def start(self):
        navigation.Probe.start(self)
        roles = {"estimator": [], "guard": []}
        for pid, entry in self.owned.items():
            if entry["executable"].endswith("/fastlivo_mapping"):
                roles["estimator"].append(pid)
            try:
                command = Path("/proc", str(pid), "cmdline").read_bytes().decode(errors="replace")
            except FileNotFoundError:
                continue
            if "fastlivo_guard" in command:
                roles["guard"].append(pid)
        self.report["fastlivo_roles"] = roles
        self.report["fastlivo_runtime_executables_sha256"] = {
            str(pid): hashlib.sha256(Path(self.owned[pid]["executable"]).read_bytes()).hexdigest()
            for pid in roles["estimator"]
        }
        runtime_libraries = {}
        for pid in roles["estimator"]:
            for line in Path("/proc", str(pid), "maps").read_text().splitlines():
                if "libvikit_common" in line:
                    path = Path(line.split()[-1])
                    runtime_libraries[str(path)] = hashlib.sha256(path.read_bytes()).hexdigest()
        self.report["fastlivo_runtime_libraries_sha256"] = runtime_libraries
        self.report["global_tf_publishers"] = [
            dict(name=p.node_name, namespace=p.node_namespace)
            for p in self.node.get_publishers_info_by_topic("/tf")
        ]
        if any(
            p["name"] in ("laserMapping", "fastlio_comparison")
            for p in self.report["global_tf_publishers"]
        ):
            raise RuntimeError("raw comparison/estimator TF leaked onto the global control tree")
        if len(roles["estimator"]) != 1:
            raise RuntimeError("unique real FAST-LIVO2 executable required")

    def on_image(self, message):
        self.original_counts["camera"] += 1
        return super().on_image(message)

    def on_map_input(self, topic, message):
        if topic == "/livox/lidar":
            self.original_counts["lidar"] += 1
        return super().on_map_input(topic, message)

    def configure_simulated_battery(self):
        # The single-SITL reservation and exact owned binary identify native IPC.
        # Avoid a premature MAVROS parameter-catalogue request blocking other services.
        matches = [
            (pid, d)
            for pid, d in self.owned.items()
            if d["executable"] == self.report["staging"]["binary"]
        ]
        if (
            len(matches) != 1
            or self.state()["armed"]
            or navigation.children(self.process.pid).get(matches[0][0]) != matches[0][1]
        ):
            raise RuntimeError("owned disarmed PX4 required for native fixture configuration")

        def command(*args):
            binary = Path(self.report["staging"]["binary"])
            client = binary.parent / "px4-param"
            if client.resolve(strict=True) != binary.resolve(strict=True):
                raise RuntimeError("native parameter client differs from owned PX4")
            cmd = [str(client), "--instance", "0", *args]
            result = subprocess.run(
                cmd,
                cwd=self.report["staging"]["rootfs"],
                env=self.environment,
                capture_output=True,
                text=True,
                timeout=5,
            )
            if result.returncode:
                raise RuntimeError(
                    f"native PX4 fixture command failed rc={result.returncode}: "
                    + result.stdout
                    + result.stderr
                )
            return result.stdout

        before = command("show", "SIM_BAT_DRAIN")
        changed = command("set", "SIM_BAT_DRAIN", str(self.args.battery_drain_seconds))
        after = command("show", "SIM_BAT_DRAIN")
        import re

        if not re.search(r"SIM_BAT_DRAIN[^\n]*:\s*1800(?:\.0*)?(?:\s|$)", after):
            raise RuntimeError("native simulated battery readback mismatch: " + after)
        self.report["simulated_battery"] = dict(
            drain_seconds=self.args.battery_drain_seconds,
            before=before,
            set_output=changed,
            readback=after,
            owned_px4_pid=matches[0][0],
            battery_failsafe_thresholds_changed=False,
            transport="owned native PX4 command socket",
        )

    def run_fault_scenario(self):
        if self.args.scenario in ("cancel", "visual-loss"):
            return super().run_fault_scenario()
        active = self.navigation(1.5)
        origin = self.state()["position"]
        self.wait(lambda: __import__("math").dist(origin, self.state()["position"]) >= 0.25, 25)
        if active["result"].done():
            raise RuntimeError("navigation ended before moving fault")
        if self.args.scenario == "localization-loss":
            kind = self.args.fault_kind
            _, proc, _ = next(r for r in self.relays if r[0] == kind)
            if proc.poll() is not None:
                raise RuntimeError("sensor relay stopped before injection")
            event = dict(
                scenario="sensor-loss",
                kind=kind,
                steady=time.monotonic(),
                state=self.state(),
                goal_uuid=active["entry"]["goal_uuid"],
                input_counts_before=dict(self.original_counts),
            )
            self.report["injection"] = event
            proc.send_signal(signal.SIGINT)
            proc.wait(timeout=5)
            self.finish(active, expected=6, seconds=30)
            event["terminal_steady"] = time.monotonic()
            end = time.monotonic() + 1.0
            self.wait(lambda: time.monotonic() >= end, 3)
            event["input_counts_after"] = dict(self.original_counts)
            if any(
                self.original_counts[k] <= event["input_counts_before"][k]
                for k in self.original_counts
                if k != kind
            ):
                raise RuntimeError("other sensor streams did not continue after isolated loss")
            self.wait(
                lambda: navigation.takeover_streams_quiet(
                    [
                        r
                        for r in self.report["streams"]
                        if r["topic"] != "/uav_command"
                        or r.get("command_mode") == navigation.UAVCommand.TRAJECTORY
                    ],
                    event["terminal_steady"],
                    time.monotonic(),
                ),
                5,
            )
            event["actual_trajectory_stream_quiet"] = True
            return
        pid = self.report["fastlivo_roles"]["estimator"][0]
        if navigation.children(self.process.pid).get(pid) != self.owned[pid]:
            raise RuntimeError("owned backend identity changed")
        event = dict(
            scenario=self.args.scenario,
            pid=pid,
            steady=time.monotonic(),
            state=self.state(),
            goal_uuid=active["entry"]["goal_uuid"],
        )
        self.report["injection"] = event
        os.kill(pid, signal.SIGINT)
        self.finish(active, expected=6, seconds=30)
        event["terminal_steady"] = time.monotonic()
        self.wait(
            lambda: navigation.takeover_streams_quiet(
                [
                    r
                    for r in self.report["streams"]
                    if r["topic"] != "/uav_command"
                    or r.get("command_mode") == navigation.UAVCommand.TRAJECTORY
                ],
                event["terminal_steady"],
                time.monotonic(),
            ),
            5,
        )
        event["actual_trajectory_stream_quiet"] = True

    def close(self):
        try:
            super().close()
        finally:
            self.fastlio_log.close()
            extra_exits = {}
            for name, proc, log in self.extra_processes:
                if proc.poll() is None:
                    proc.send_signal(signal.SIGINT)
                    try:
                        proc.wait(timeout=15)
                    except subprocess.TimeoutExpired:
                        proc.kill()
                        proc.wait(timeout=5)
                        self.report["forced"] = True
                extra_exits[name] = proc.returncode
                log.close()
            self.report["extra_process_exits"] = extra_exits
            self.report["passed"] = self.report["passed"] and all(
                v == 0 for v in extra_exits.values()
            )
            relay_exits = {}
            for kind, proc, log in self.relays:
                if proc.poll() is None:
                    proc.send_signal(signal.SIGINT)
                    try:
                        proc.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        proc.kill()
                        proc.wait(timeout=5)
                        self.report["forced"] = True
                relay_exits[kind] = proc.returncode
                log.close()
            self.report["relay_exits"] = relay_exits
            self.report["passed"] = (
                self.report["passed"]
                and not self.report["forced"]
                and all(v == 0 for v in relay_exits.values())
            )
            (self.directory / "result.json").write_text(json.dumps(self.report, indent=2) + "\n")


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--output-dir", type=Path, required=True)
    p.add_argument("--px4-dir", type=Path, required=True)
    p.add_argument("--truth-executable", type=Path, required=True)
    p.add_argument("--domain", type=int, default=223)
    p.add_argument("--gazebo-port", type=int, default=12623)
    p.add_argument("--scene", default="primitives_sparse_seed42_v1")
    p.add_argument(
        "--scenario",
        choices=[
            "dynamic",
            "mission",
            "cancel",
            "backend-exit",
            "localization-loss",
            "visual-loss",
            "clock-reset",
            "obstacle",
            "long-run",
        ],
        default="dynamic",
    )
    p.add_argument("--feature-board-distance", type=float, default=4.0)
    p.add_argument("--obstacle-distance", type=float, default=3.0)
    p.add_argument("--heading-degrees", type=float, default=0.0)
    p.add_argument("--obstacle-lateral-offset", type=float, default=0.0)
    p.add_argument("--record-inputs", action="store_true")
    p.add_argument("--compare-fastlio", action="store_true")
    p.add_argument("--fault-kind", choices=["camera", "imu", "lidar"], default="camera")
    p.add_argument("--strategy", choices=["astar", "ego", "combined"], default="combined")
    p.add_argument("--hold-source-seconds", type=float, default=12.0)
    p.add_argument("--managed-profile", action="store_true")
    p.add_argument("--visual-feature-board", action="store_true")
    p.add_argument("--execute-simulation", action="store_true", required=True)
    args = p.parse_args()
    if (
        not 1 <= args.domain <= 232
        or not 1024 <= args.gazebo_port <= 65535
        or not 10 <= args.hold_source_seconds <= 600
    ):
        p.error("invalid isolated runtime or duration")
    if (
        not math.isfinite(args.heading_degrees)
        or abs(args.heading_degrees) > 90
        or not math.isfinite(args.obstacle_lateral_offset)
        or abs(args.obstacle_lateral_offset) > 2
    ):
        p.error("invalid physical fixture geometry or heading")
    if args.managed_profile and args.scenario == "localization-loss":
        p.error("transparent sensor relay faults use the explicit low-level simulation entry")
    if not 4 <= args.feature_board_distance <= 12 or not 3 <= args.obstacle_distance <= 6:
        p.error("controlled fixture distances outside reviewed simulation bounds")
    args.battery_drain_seconds = 1800.0
    args.cpu_workers = 0
    args.no_input_audit = True
    args.bootstrap_climb_speed = 0.3
    args.initialization_translation = 0.0
    args.output_dir.mkdir(parents=True, exist_ok=False)
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    probe = Probe(args)
    try:
        probe.run()
    except Exception as e:
        probe.report["error"] = repr(e)
    finally:
        probe.close()
    print(json.dumps({k: probe.report[k] for k in ["passed", "error", "forced"]}, indent=2))
    return 0 if probe.report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
