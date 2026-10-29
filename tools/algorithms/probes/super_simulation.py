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

"""Explicit managed SUPER flights; native truth is read-only and never a planner input."""
import argparse
import base64
from concurrent.futures import ThreadPoolExecutor
import gzip
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import threading
import time

from ament_index_python.packages import get_package_prefix
from easyfly_msgs.msg import NavigationSession, SepticTrajectory, VoxelGrid
from easyfly_msgs.srv import GetActionOwnership
from quadrotor_msgs.msg import PositionCommand
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from traj_utils.msg import Bspline
from process_cpu_trace import snapshot

ROOT = Path(__file__).resolve().parents[3]
spec = importlib.util.spec_from_file_location(
    "super_sim_common", Path(__file__).with_name("openvins_simulation.py")
)
common = importlib.util.module_from_spec(spec)
spec.loader.exec_module(common)
navigation = common.navigation


def stamp(value):
    return value.sec * 10 ** 9 + value.nanosec


class Probe(navigation.Probe):
    configure_simulated_battery = common.Probe.configure_simulated_battery
    model_witness = common.Probe.model_witness

    def __init__(self, args):
        self.cpu_samples = []; self.last_cpu_sample = 0.
        self.comparison_map = None
        from easyfly_fastdds_compat.runtime import activate

        activate()
        super().__init__(args)
        middleware = activate()
        self.environment.update(middleware["profile_environment"])
        self.environment["LD_LIBRARY_PATH"] = os.environ["LD_LIBRARY_PATH"]
        self.report["dds_compatibility"] = middleware
        self.report["scene"] = "gcopter_3d_lane_v1"
        self.report["probe_sha256"] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
        self.report["acceptance_limits"] = json.loads(
            (
                ROOT / "artifacts/new-algorithms/new4-super-20261022/acceptance-limits.json"
            ).read_text()
        )
        self.voxel = None
        self.voxel_receipt = 0.0
        self.map_count = self.command_count = self.discarded = 0
        self.polynomials = []
        self.bsplines = []
        self.bspline_log = (self.directory / "ego-bsplines.jsonl").open("x")
        self.node.create_subscription(Bspline, "/planning/bspline", self.on_bspline, 100)
        self.replan_notices = []
        self.command_events = []
        self.truth = self.truth_log = None
        self.paused = []
        self.voxel_log = gzip.open(self.directory / "voxel-grid.jsonl.gz", "xt")
        self.command_log = (self.directory / "commands.jsonl").open("x")
        self.polynomial_log = (self.directory / "polynomials.jsonl").open("x")
        self.node.create_subscription(
            VoxelGrid,
            "/planning/voxel_grid",
            self.on_voxel,
            QoSProfile(depth=20, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.node.create_subscription(
            SepticTrajectory,
            "/planning/super_trajectory",
            self.on_plan,
            QoSProfile(depth=20, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.node.create_subscription(
            PositionCommand, "/position_cmd", self.on_command, qos_profile_sensor_data
        )
        self.node.create_subscription(
            NavigationSession,
            "/planning/super_replan",
            lambda msg: self.replan_notices.append(
                dict(
                    receipt=time.monotonic(),
                    clock_ns=self.node.get_clock().now().nanoseconds,
                    session=msg.session_id,
                    active=msg.active,
                    state=self.state(),
                )
            ),
            QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        if self.args.strategy == "super":
            self.node.create_subscription(
                PointCloud2, "/livox/lidar/rays", self.on_rays, qos_profile_sensor_data
            )
        for topic, kind in [
            ("/uav_command", navigation.UAVCommand),
            ("/mavros/setpoint_raw/local", navigation.PositionTarget),
        ]:
            self.node.create_subscription(
                kind, topic, lambda m, t=topic: self.stream(t, m), qos_profile_sensor_data
            )
        sources = self.directory / "source-snapshot"
        sources.mkdir()
        files = [Path(__file__).resolve()]
        files += list(
            (ROOT / "src/execution/e2_planning/easyfly_super_planner").rglob("*.cpp")
        )
        files += list(
            (ROOT / "src/execution/e2_planning/easyfly_super_planner").rglob("*.hpp")
        )
        files += [
            ROOT
            / "src/execution/e2_planning/easyfly_global_planner/src/navigation_coordinator_node.cpp",
            ROOT / "src/simulation/easyfly_px4_sitl/launch/perception_sitl.launch.py",
            ROOT / "src/simulation/easyfly_gazebo_worlds/scenes/gcopter_3d_lane_v1.yaml",
            ROOT / "src/third_party/algorithms/planning/super.provenance.json",
            Path(common.__file__),
            Path(navigation.__file__),
            ROOT / "src/easyfly_common/easyfly_msgs/msg/VoxelGrid.msg",
            ROOT / "src/easyfly_common/easyfly_msgs/msg/SepticTrajectory.msg",
            ROOT / "src/application/a3_mission/easyfly_sim_launcher/launch/profile.launch.py",
            ROOT / "src/execution/e2_planning/easyfly_global_planner/launch/navigate.launch.py",
            ROOT
            / "src/simulation/easyfly_gazebo_plugins/livox_laser_plugin/src/livox_points_plugin.cpp",
            ROOT / "src/simulation/easyfly_gazebo_plugins/safe_transport/src/server_main.cpp",
            ROOT
            / "src/simulation/easyfly_gazebo_plugins/safe_transport/include/easyfly_transport/signal_relay.hpp",
            ROOT
            / "src/simulation/easyfly_gazebo_plugins/safe_transport/test/test_signal_relay.cpp",
        ]
        self.report["source_hashes"] = {}
        for source in files:
            relative = source.relative_to(ROOT)
            target = sources / relative
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(source.read_bytes())
            self.report["source_hashes"][str(relative)] = hashlib.sha256(
                source.read_bytes()
            ).hexdigest()

    def refresh_owned(self):
        if self.process is None or self.process.poll() is not None:
            return
        current = navigation.children(self.process.pid)
        for pid, identity in current.items():
            if pid in self.owned and self.owned[pid] != identity:
                raise RuntimeError('owned PID identity changed')
            self.owned[pid] = identity

    def wait(self, predicate, seconds):
        def observed():
            now = time.monotonic()
            if self.owned and now - self.last_cpu_sample >= 1.:
                self.last_cpu_sample = now
                self.refresh_owned()
                record = snapshot(list(self.owned.items()))
                record['clock_ns'] = self.node.get_clock().now().nanoseconds
                self.cpu_samples.append(record)
            return predicate()
        return super().wait(observed, seconds)

    def on_bspline(self, msg):
        row = dict(receipt=time.monotonic(), stamp_ns=stamp(msg.start_time),
                   session=msg.navigation_session_id, trajectory_id=msg.traj_id,
                   order=msg.order, knots=list(msg.knots),
                   position_points=[[p.x, p.y, p.z] for p in msg.pos_pts],
                   yaw_points=list(msg.yaw_pts), yaw_dt=msg.yaw_dt, drone_id=msg.drone_id)
        self.bsplines.append(row)
        self.bspline_log.write(json.dumps(row)+'\n')

    def start_comparison_map(self):
        # An observer-only copy of the exact shared ray-map producer, not an
        # extra planner or a second SITL. No trajectory/command publishers.
        executable = Path(get_package_prefix('easyfly_trajectory_optimizer')) / 'lib/easyfly_trajectory_optimizer/gcopter_voxel_map'
        self.comparison_map_log = (self.directory/'comparison-map.log').open('x')
        self.comparison_map = subprocess.Popen([str(executable), '--ros-args', '-p',
            'use_sim_time:=true', '-r', '__node:=super_comparison_map'], env=self.environment,
            stdout=self.comparison_map_log, stderr=subprocess.STDOUT)
        proc = Path('/proc')/str(self.comparison_map.pid)
        identity = dict(executable=str((proc/'exe').resolve(strict=True)),
                        start_ticks=(proc/'stat').read_text().rsplit(')',1)[1].split()[19])
        self.comparison_map_identity = identity
        self.owned[self.comparison_map.pid] = identity
        self.wait(lambda:self.voxel is not None and self.voxel.valid and self.map_count>=3, 20)
        self.report['comparison_map'] = dict(pid=self.comparison_map.pid, **identity,
            executable_sha256=hashlib.sha256(executable.read_bytes()).hexdigest(),
            observation_only=True, same_source_ray_map=True)
        client = self.node.create_client(common.GetParameters, '/ego_planner_node/get_parameters')
        self.wait(client.service_is_ready, 10)
        names = ['manager/max_acc', 'optimization/max_acc', 'manager/max_jerk']
        future = client.call_async(common.GetParameters.Request(names=names))
        self.wait(future.done, 10)
        values = {name: value.double_value for name, value in zip(names, future.result().values)}
        self.node.destroy_client(client)
        if values != {'manager/max_acc': 2.0, 'optimization/max_acc': 2.0, 'manager/max_jerk': 4.0}:
            raise RuntimeError('actual EGO comparison limits differ from declared bounds')
        self.report['actual_ego_comparison_parameters'] = values


    def on_voxel(self, msg):
        self.voxel, self.voxel_receipt = msg, time.monotonic()
        self.map_count += 1
        if self.map_count > 20000:
            self.discarded += 1
            return
        row = dict(
            receipt=self.voxel_receipt,
            clock_ns=self.node.get_clock().now().nanoseconds,
            stamp_ns=stamp(msg.header.stamp),
            frame=msg.header.frame_id,
            session=msg.session_id,
            generation=msg.generation,
            revision=msg.revision,
            valid=msg.valid,
            resolution=msg.resolution,
            origin=[msg.origin.x, msg.origin.y, msg.origin.z],
            size=[msg.size_x, msg.size_y, msg.size_z],
            inflation=msg.inflation_radius,
            data_b64=base64.b64encode(memoryview(msg.data)).decode(),
        )
        self.voxel_log.write(json.dumps(row) + "\n")

    def on_plan(self, msg):
        row = dict(
            receipt=time.monotonic(),
            stamp_ns=stamp(msg.header.stamp),
            session=msg.navigation_session_id,
            trajectory_id=msg.trajectory_id,
            map_session=msg.map_session_id,
            map_generation=msg.map_generation,
            map_revision=msg.map_revision,
            validated_map_revision=msg.validated_map_revision,
            map_stamp_ns=stamp(msg.map_stamp),
            validated_map_stamp_ns=stamp(msg.validated_map_stamp),
            state_stamp_ns=stamp(msg.state_stamp),
            durations=list(msg.durations),
            coefficients=list(msg.coefficients),
            max_speed=msg.max_speed,
            max_acceleration=msg.max_acceleration,
            max_jerk=msg.max_jerk,
        )
        self.polynomials.append(row)
        self.polynomial_log.write(json.dumps(row) + "\n")

    def on_command(self, msg):
        self.command_count += 1
        if self.command_count > 200000:
            self.discarded += 1
            return
        row = dict(
            receipt=time.monotonic(),
            stamp_ns=stamp(msg.header.stamp),
            session=msg.navigation_session_id,
            trajectory_id=msg.trajectory_id,
            position=[msg.position.x, msg.position.y, msg.position.z],
            velocity=[msg.velocity.x, msg.velocity.y, msg.velocity.z],
            acceleration=[msg.acceleration.x, msg.acceleration.y, msg.acceleration.z],
            yaw=msg.yaw,
        )
        self.command_events.append(row)
        self.command_log.write(json.dumps(row) + "\n")

    def on_rays(self, msg):
        row = self.report.setdefault("native_rays", dict(count=0, hit=0, miss=0, invalid=0))
        row["count"] += 1
        row["last_source_ns"] = stamp(msg.header.stamp)
        if msg.point_step != 24:
            raise RuntimeError("unexpected native ray format")
        data = memoryview(msg.data)
        types = bytes(data[12::24])
        row["hit"] += types.count(1)
        row["miss"] += types.count(0)
        row["invalid"] += types.count(2)
        if row["count"] <= 3:
            (self.directory / f'rays-{row["count"]}.bin').write_bytes(data)
            (self.directory / f'rays-{row["count"]}.json').write_text(
                json.dumps(
                    dict(
                        stamp_ns=stamp(msg.header.stamp),
                        frame=msg.header.frame_id,
                        width=msg.width,
                        point_step=msg.point_step,
                        data_sha256=hashlib.sha256(data).hexdigest(),
                    ),
                    indent=2,
                )
                + "\n"
            )

    def map_ready(self):
        if self.args.strategy != "super":
            p = self.latest.get("/occupancy_grid")
            return (
                p is not None
                and self.counts.get("/occupancy_grid", 0) >= 3
                and time.monotonic() - self.receipts["/occupancy_grid"] <= 1.0
                and -0.2e9
                <= self.node.get_clock().now().nanoseconds - stamp(p.header.stamp)
                <= 1e9
            )
        p = self.voxel
        return (
            p is not None
            and p.valid
            and p.header.frame_id == "odom"
            and self.map_count >= 3
            and time.monotonic() - self.voxel_receipt <= 1
            and -0.2e9 <= self.node.get_clock().now().nanoseconds - stamp(p.header.stamp) <= 1e9
        )

    def startup_ready(self):
        if self.truth is None and self.process is not None:
            path = self.directory / "console.log"
            if path.exists() and "Connected to gazebo master" in path.read_text():
                self.truth_log = (self.directory / "truth.log").open("x")
                self.truth = subprocess.Popen(
                    [
                        str(self.args.truth_executable.resolve(strict=True)),
                        str(self.directory / "truth.csv"),
                    ],
                    env=self.environment,
                    stdout=self.truth_log,
                    stderr=subprocess.STDOUT,
                )
                self.report["truth_pid"] = self.truth.pid
                self.report["truth_executable_sha256"] = hashlib.sha256(
                    self.args.truth_executable.read_bytes()
                ).hexdigest()
        if self.args.strategy != "super":
            return navigation.Probe.startup_ready(self)
        return (
            self.ready(False)
            and self.map_ready()
            and "super_navigation" in self.node.get_node_names()
        )

    def start(self):
        super().start()
        if self.args.baseline_strategy == "ego":
            self.start_comparison_map()
        self.report[
            "server_signal_handoff_verified"
        ] = "EasyFly Gazebo: async-safe SIGINT handoff installed" in (
            self.directory / "console.log"
        ).read_text(
            errors="replace"
        )
        if not self.report["server_signal_handoff_verified"]:
            raise RuntimeError("Gazebo did not install the async-safe shutdown handoff")
        masks = []
        for pid, identity in self.owned.items():
            if identity["executable"].endswith("/easyfly_gzserver"):
                for status in Path("/proc", str(pid), "task").glob("*/status"):
                    try:
                        value = re.search(r"^SigBlk:\s+([0-9a-f]+)$", status.read_text(), re.M)
                        masks.append(dict(tid=int(status.parent.name), blocked=value.group(1)))
                    except FileNotFoundError:
                        pass
        self.report["server_signal_masks"] = masks
        if not masks:
            raise RuntimeError("simulator threads could not be audited")
        self.report["runtime_executables_sha256"] = {
            p["executable"]: hashlib.sha256(Path(p["executable"]).read_bytes()).hexdigest()
            for p in self.owned.values()
            if p["executable"].endswith(
                (
                    "/super_navigation",
                    "/gcopter_voxel_map",
                    "/navigation_coordinator_node",
                    "/ego_planner_node",
                )
            )
        }
        libraries = {}
        for pid in self.owned:
            try:
                lines = Path("/proc", str(pid), "maps").read_text().splitlines()
                for line in lines:
                    path = line.split()[-1]
                    if any(
                        name in path
                        for name in [
                            "libsuper_validation", "libsuper_execution_core",
                            "libeasyfly_super_planner", "libeasyfly_rog_map",
                            "liblivox_laser_simulation",
                            "libeasyfly_global_planner",
                            "libeasyfly_native_imu",
                            "libfastrtps",
                        ]
                    ):
                        if Path(path).is_file():
                            libraries[path] = hashlib.sha256(Path(path).read_bytes()).hexdigest()
            except OSError:
                pass
        self.report["runtime_libraries_sha256"] = libraries
        worker = (
            Path(get_package_prefix("easyfly_super_planner"))
            / "lib/easyfly_super_planner/super_worker"
        )
        self.report["worker_sha256"] = hashlib.sha256(worker.read_bytes()).hexdigest()

    def launch_command(self):
        common_args = [
            "scene:=gcopter_3d_lane_v1",
            "headless:=true",
            "rviz:=false",
            "detection_use_gpu:=false",
            "super_evidence_directory:=" + str(self.directory),
        ]
        if self.args.baseline_strategy == "ego":
            common_args.append("local_planner_max_acceleration:=2.0")
        if self.args.managed_profile:
            command = [
                "ros2",
                "launch",
                "easyfly_sim_launcher",
                "profile.launch.py",
                "profile:=navigation_dev",
                "algorithms_json:="
                + json.dumps(
                    dict(slam="fast_lio", strategy=self.args.strategy, provider="cpu"),
                    separators=(",", ":"),
                ),
                *common_args,
            ]
        else:
            command = [
                "ros2",
                "launch",
                "easyfly_px4_sitl",
                "perception_sitl.launch.py",
                "scene_profile:=navigation_dev",
                "navigation_strategy:=" + self.args.strategy,
                *common_args,
            ]
        if self.args.shutdown_stack_diagnostic:
            prefix = self.directory / "allow_owned_server_ptrace.py"
            prefix.write_text(
                "import ctypes, os, sys\n"
                "libc=ctypes.CDLL(None, use_errno=True)\n"
                "assert libc.prctl(0x59616d61, ctypes.c_ulong(-1),0,0,0)==0\n"
                "os.execv(sys.argv[1],sys.argv[1:])\n"
            )
            command[2:2] = [
                "--launch-prefix",
                "python3 " + str(prefix),
                "--launch-prefix-filter",
                ".*easyfly_gzserver.*",
            ]
            self.report["diagnostic_only"] = True
        return command

    def observe(self, source_seconds):
        begin = self.node.get_clock().now().nanoseconds
        self.wait(
            lambda: self.node.get_clock().now().nanoseconds - begin >= source_seconds * 1e9,
            source_seconds * 8 + 15,
        )

    def spawn_box(self, name, center, size, inflight=False):
        if inflight:
            state = self.state()
            if (
                self.args.scenario != "map-change"
                or not self.ready(True)
                or not 0.25 <= state["position"][0] <= 0.6
                or abs(state["position"][1]) > 0.3
                or center != [2.0, 0.0, 1.3]
                or size != [0.2, 1.0, 2.6]
            ):
                raise RuntimeError(
                    "future physical obstacle is not safely separated from the vehicle"
                )
            self.report["physical_map_change_injection"] = dict(
                state=state, steady=time.monotonic()
            )
        elif self.state()["armed"]:
            raise RuntimeError("static fixture insertion requires confirmed disarm")
        path = self.directory / (name + ".sdf")
        geometry = "<geometry><box><size>" + " ".join(map(str, size)) + "</size></box></geometry>"
        path.write_text(
            '<sdf version="1.6"><model name="' + name + '"><static>true</static><link name="body">'
            '<collision name="collision">'
            + geometry
            + '</collision><visual name="visual">'
            + geometry
            + "<material><ambient>0.3 0.4 0.5 1</ambient><diffuse>0.3 0.4 0.5 1</diffuse></material>"
            "</visual></link></model></sdf>"
        )
        cmd = [
            "gz",
            "model",
            "--spawn-file=" + str(path),
            "--model-name=" + name,
            "-x",
            str(center[0]),
            "-y",
            str(center[1]),
            "-z",
            str(center[2]),
        ]

        def insert_and_witness():
            done = subprocess.run(
                cmd,
                env=self.environment,
                capture_output=True,
                text=True,
                timeout=3 if inflight else 10,
            )
            if done.returncode:
                raise RuntimeError("physical fixture spawn failed: " + done.stderr)
            return self.model_witness(name, center, require_collision=True)

        if inflight:
            # Continue receiving actual evidence during native CLI operations.
            with ThreadPoolExecutor(max_workers=1) as pool:
                future = pool.submit(insert_and_witness)
                self.wait(future.done, 25)
                witness = future.result()
        else:
            witness = insert_and_witness()
        self.report.setdefault("obstacles", []).append(
            dict(
                name=name,
                center=center,
                size=size,
                sdf_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                native_witness=witness,
                command=cmd,
            )
        )

    def fixtures(self):
        scenario = self.args.scenario
        if scenario == "over":
            self.spawn_box("super_lower_wall", [2.0, 0.0, 0.6], [0.2, 10.0, 1.2])
        if scenario == "under":
            self.spawn_box("super_upper_wall", [2.0, 0.0, 3.5], [0.2, 10.0, 3.0])
        if scenario == "channel":
            self.spawn_box("super_channel_low", [1.5, 0.0, 0.55], [0.2, 10.0, 1.1])
            self.spawn_box("super_channel_high", [4.0, 0.0, 3.55], [0.2, 10.0, 2.9])
        if scenario == "narrow":
            self.spawn_box("super_corridor_left", [2.0, 1.2, 1.7], [3.0, 0.2, 3.4])
            self.spawn_box("super_corridor_right", [2.0, -1.2, 1.7], [3.0, 0.2, 3.4])
        if scenario == "no-solution":
            self.spawn_box("super_full_wall", [2.0, 0.0, 2.5], [0.2, 10.0, 5.0])

    def navigate_target(self, target):
        self.stable_hover()
        self.report.setdefault("navigation_targets", []).append(target)
        began = time.monotonic()
        active = self.send(
            "/navigate",
            navigation.NavigateTo,
            navigation.NavigateTo.Goal(
                target_x=target[0], target_y=target[1], target_z=target[2], max_speed=0.6
            ),
        )
        self.report.setdefault('planning_requests', []).append(dict(
            goal_uuid=active['entry']['goal_uuid'], receipt=began,
            source_ns=self.node.get_clock().now().nanoseconds, target=target))
        return active

    def cancelled(self, active):
        self.wait(
            lambda: bool(self.command_events)
            and math.dist(self.state()["position"], self.command_events[0]["position"]) >= 0.25
            and math.sqrt(sum(v * v for v in self.state()["velocity"])) >= 0.10
            and navigation.trajectory_stream_observed(
                self.report["streams"], time.monotonic(), self.node.get_clock().now().nanoseconds
            ),
            40,
        )
        event = dict(scenario=self.args.scenario, steady=time.monotonic(), state=self.state())
        event["moving_distance_m"] = math.dist(
            self.state()["position"], self.command_events[0]["position"]
        )
        self.report["injection"] = event
        if self.args.scenario == "cancel":
            future = active["handle"].cancel_goal_async()
            self.wait(future.done, 5)
            result = future.result()
            identities = [bytes(g.goal_id.uuid).hex() for g in result.goals_canceling]
            if result.return_code != 0 or active["entry"]["goal_uuid"] not in identities:
                raise RuntimeError("exact cancellation rejected")
            event["cancel_ack_uuids"] = identities
        else:
            self.refresh_owned()
            suffix = (
                "/gcopter_voxel_map" if self.args.scenario == "map-loss" else "/super_navigation"
            )
            candidates = [
                (pid, data)
                for pid, data in self.owned.items()
                if data["executable"].endswith(suffix)
            ]
            if (
                len(candidates) != 1
                or navigation.children(self.process.pid).get(candidates[0][0]) != candidates[0][1]
            ):
                raise RuntimeError("unique owned injection target unavailable")
            pid, identity = candidates[0]
            event["pid"] = pid
            if self.args.scenario == "map-loss":
                os.kill(pid, signal.SIGSTOP)
                self.paused.append((pid, identity))
            else:
                os.kill(pid, signal.SIGINT)
        if self.args.scenario == "backend-exit":
            self.wait(active["result"].done, 20)
            value = active["result"].result()
            active["entry"].update(
                status=value.status,
                success=value.result.success,
                message=value.result.message,
                terminal_state=self.state(),
            )
            if value.status != 6 or value.result.success:
                raise RuntimeError("missing backend did not abort navigation")
            client = self.node.create_client(GetActionOwnership, "/navigation/ownership")
            self.wait(client.service_is_ready, 5)
            query = client.call_async(
                GetActionOwnership.Request(goal_id=active["entry"]["goal_uuid"])
            )
            self.wait(query.done, 5)
            reply = query.result()
            self.node.destroy_client(client)
            if (
                reply.state != GetActionOwnership.Response.OWNED
                or reply.goal_id != active["entry"]["goal_uuid"]
            ):
                raise RuntimeError("unconfirmed planner stop incorrectly released ownership")
            event["fault_lock_retains_exact_ownership"] = True
            rejected = navigation.ActionClient(self.node, navigation.NavigateTo, "/navigate")
            self.clients.append(rejected)
            self.wait(rejected.server_is_ready, 5)
            state = self.state()["position"]
            admission = rejected.send_goal_async(
                navigation.NavigateTo.Goal(
                    target_x=state[0] + 0.5, target_y=state[1], target_z=state[2], max_speed=0.6
                )
            )
            self.wait(admission.done, 5)
            if admission.result().accepted:
                raise RuntimeError("fault-locked coordinator admitted another goal")
            event["new_navigation_rejected"] = True
        else:
            self.finish(active, expected=5 if self.args.scenario == "cancel" else 6, seconds=20)
        event["terminal_wall_s"] = time.monotonic() - event["steady"]
        self.observe(2.0)
        stop = time.monotonic()
        count = self.command_count
        self.observe(1.0)
        if self.command_count != count:
            raise RuntimeError("trajectory commands continued after stop")
        event["quiet_after_steady"] = stop
        for pid, identity in self.paused:
            if navigation.children(self.process.pid).get(pid) != identity:
                raise RuntimeError("paused process identity changed")
            os.kill(pid, signal.SIGCONT)
        self.paused.clear()

    def run(self):
        self.start()
        if self.truth is None:
            raise RuntimeError("native independent truth observer missing")
        if self.args.scenario == "shutdown-diagnostic":
            self.observe(self.args.shutdown_diagnostic_hold_seconds)
            self.report["final_disarmed_state"] = self.state()
            return
        self.configure_simulated_battery()
        self.fixtures()
        self.observe(3.0)
        altitude = (
            2.2 if self.args.scenario == "under" else 1.2 if self.args.scenario == "over" else 1.5
        )
        failure = None
        try:
            takeoff = self.send(
                "/takeoff",
                navigation.Takeoff,
                navigation.Takeoff.Goal(target_altitude=altitude, climb_speed=0.3),
            )
            self.finish(takeoff, seconds=90)
            self.stable_hover(altitude)
            self.wait(self.map_ready, 20)
            home = list(self.state()["position"])
            distance = 5.5 if self.args.scenario == "channel" else 4.0
            target = [home[0] + distance, home[1], home[2]]
            first = self.navigate_target(target)
            if self.args.scenario in ["cancel", "map-loss", "backend-exit"]:
                self.cancelled(first)
                if self.args.scenario != "backend-exit":
                    self.observe(2.0)
                    self.wait(self.map_ready, 20)
                    recovery = self.navigate_target([home[0] + 1.5, home[1], home[2]])
                    self.finish(recovery, seconds=90)
                    returned = self.navigate_target(home)
                    self.finish(returned, seconds=90)
                    errors = [
                        math.dist(a["entry"]["terminal_state"]["position"], p)
                        for a, p in zip(
                            [recovery, returned], self.report["navigation_targets"][-2:]
                        )
                    ]
                    self.report["recovery_terminal_errors_m"] = errors
                    if max(errors) > 0.11:
                        raise RuntimeError("recovery arrival outside frozen tolerance")
                    self.report["explicit_recovery_completed"] = True
            elif self.args.scenario == "no-solution":
                self.finish(first, expected=6, seconds=20)
                if self.polynomials or self.command_count:
                    raise RuntimeError("no-solution case emitted an executable trajectory")
                self.report["no_solution_fail_closed"] = True
            else:
                if self.args.scenario == "map-change":
                    self.wait(
                        lambda: self.command_events and 0.25 <= self.state()["position"][0] <= 0.6,
                        40,
                    )
                    self.spawn_box(
                        "super_added_panel",
                        [2.0, 0.0, 1.3],
                        [0.2, 1.0, 2.6],
                        inflight=True,
                    )
                    self.observe(1.0)
                self.finish(first, seconds=120)
                if self.args.scenario == "map-change" and not self.replan_notices:
                    raise RuntimeError("physical map change did not exercise bounded replanning")
                back = self.navigate_target(home)
                self.finish(back, seconds=120)
                errors = [
                    math.dist(a["entry"]["terminal_state"]["position"], p)
                    for a, p in zip([first, back], self.report["navigation_targets"])
                ]
                self.report["navigation_terminal_errors_m"] = errors
                if max(errors) > 0.11:
                    raise RuntimeError("arrival outside frozen tolerance")
        except Exception as error:
            failure = error
        if failure:
            self.cleanup_actions()
        if self.state() and self.state()["armed"]:
            land = self.send("/land", navigation.Land, navigation.Land.Goal())
            self.finish(land, seconds=120)
        self.wait(lambda: self.ready(False), 20)
        self.report["final_disarmed_state"] = self.state()
        if self.args.scenario != "backend-exit":
            self.report["sessions_closed"] = navigation.sessions_closed(
                self.report["sessions"], self.navigation_goals
            )
            if (
                self.args.scenario == "no-solution"
                and not self.polynomials
                and not self.command_count
            ):
                rejected = [a for a in self.report["actions"] if a["name"] == "/navigate"]
                no_start = not any(s["active"] for s in self.report["sessions"])
                if (
                    rejected
                    and all(
                        a.get("status") == 6
                        and a.get("message", "").startswith("planner rejected session:")
                        for a in rejected
                    )
                    and no_start
                ):
                    # A rejected START has no planner ownership to STOP. Exact
                    # navigation/child release was still required by finish().
                    self.report["rejected_before_planner_ownership"] = True
                    self.report["sessions_closed"] = True
            if not self.report["sessions_closed"]:
                raise RuntimeError("planner sessions not stopped")
        if failure:
            raise failure

    def close(self):
        if self.comparison_map is not None:
            if self.comparison_map.poll() is None:
                proc = Path('/proc')/str(self.comparison_map.pid)
                if (proc/'stat').read_text().rsplit(')',1)[1].split()[19] != self.comparison_map_identity['start_ticks']:
                    raise RuntimeError('comparison map identity changed')
                self.comparison_map.send_signal(signal.SIGINT)
                try:
                    self.comparison_map.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    self.report['forced'] = True
                    self.comparison_map.terminate();self.comparison_map.wait(timeout=5)
            self.comparison_map_log.close()
            self.report['comparison_map_returncode'] = self.comparison_map.returncode
            if self.comparison_map.returncode:
                self.report['error'] = self.report['error'] or 'comparison map exit failure'
        self.refresh_owned()

        for pid, identity in self.paused:
            if navigation.children(self.process.pid).get(pid) == identity:
                os.kill(pid, signal.SIGCONT)
        self.paused.clear()
        if self.truth is not None:
            if self.truth.poll() is None:
                self.truth.send_signal(signal.SIGINT)
                try:
                    self.truth.wait(timeout=12)
                except subprocess.TimeoutExpired:
                    self.truth.kill()
                    self.truth.wait(timeout=3)
                    self.report["forced"] = True
            self.report["truth_returncode"] = self.truth.returncode
            self.truth_log.close()
        self.report.update(
            voxel_samples=self.map_count,
            polynomial_count=len(self.polynomials),
            command_samples=self.command_count,
            discarded=self.discarded,
            map_replan_notices=self.replan_notices,
        )
        self.report["optimizer_workers_reaped"] = all(
            not Path("/proc/" + owner.read_text().strip()).exists()
            for owner in self.directory.glob("super-*/job-*.owner")
        )
        finished = threading.Event()

        def capture_shutdown():
            if finished.wait(self.args.shutdown_stack_delay):
                return
            for pid, identity in self.owned.items():
                if identity["executable"].endswith("/easyfly_gzserver"):
                    if navigation.children(self.process.pid).get(pid) != identity:
                        continue
                    with (self.directory / "shutdown-stack.txt").open("x") as log:
                        try:
                            subprocess.run(
                                [
                                    "gdb",
                                    "-q",
                                    "-batch",
                                    "-p",
                                    str(pid),
                                    "-ex",
                                    "set pagination off",
                                    "-ex",
                                    "thread apply all bt",
                                    "-ex",
                                    "detach",
                                ],
                                stdout=log,
                                stderr=subprocess.STDOUT,
                                timeout=3.0,
                            )
                        except subprocess.TimeoutExpired:
                            log.write("stack capture deadline\n")

        watcher = None
        if self.args.shutdown_stack_diagnostic:
            watcher = threading.Thread(target=capture_shutdown)
            watcher.start()
        super().close()
        self.bspline_log.close()
        (self.directory/'process-budget.json').write_text(json.dumps(dict(
            samples=self.cpu_samples, clock_ticks_per_second=os.sysconf('SC_CLK_TCK'),
            page_size=os.sysconf('SC_PAGE_SIZE')),indent=2)+'\n')
        finished.set()
        if watcher:
            watcher.join(timeout=4.0)
        self.voxel_log.close()
        self.command_log.close()
        self.polynomial_log.close()
        path = self.directory / "console.log"
        log = path.read_text(errors="replace") if path.exists() else ""
        required = [
            "gcopter_voxel_map",
            "super_navigation",
            "navigation_coordinator_node",
            "flight_control_node",
            "mavros_node",
        ]
        if self.args.strategy != "super":
            required = [
                "ego_planner_node",
                "navigation_coordinator_node",
                "obstacle_supervisor_node",
                "flight_control_node",
                "mavros_node",
            ]
            if (
                self.polynomials
                or (self.voxel is not None and self.args.baseline_strategy != "ego")
                or any(
                    (p["executable"].endswith("/super_navigation") or
                     (p["executable"].endswith("/gcopter_voxel_map") and self.args.baseline_strategy != "ego"))
                    for p in self.owned.values()
                )
            ):
                # A legacy regression must not accidentally activate SUPER.
                self.report["error"] = "unexpected SUPER data in legacy baseline"
        self.report["selected_roles_started"] = all(
            re.search(r"\[" + name + r"-\d+\]: process started", log) for name in required
        )
        self.report["super_roles_started"] = (
            self.args.strategy == "super" and self.report["selected_roles_started"]
        )
        self.report["passed"] = (
            self.report["error"] is None
            and not self.report["forced"]
            and self.report.get("launch_returncode") == self.report.get("truth_returncode") == 0
            and len(self.report["started_processes"]) >= 20
            and self.report["selected_roles_started"]
            and set(self.report["normal_exits"]) == set(self.report["started_processes"])
            and not self.report["abnormal_lines"]
            and not self.report["leftovers"]
            and self.report["transport_drained"]
            and not self.discarded
            and "final_disarmed_state" in self.report
        )
        self.report["passed"] = self.report["passed"] and self.report["optimizer_workers_reaped"]
        (self.directory / "result.json").write_text(json.dumps(self.report, indent=2) + "\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--px4-dir", type=Path, required=True)
    parser.add_argument("--truth-executable", type=Path, required=True)
    parser.add_argument("--domain", type=int, default=222)
    parser.add_argument("--gazebo-port", type=int, default=12622)
    parser.add_argument(
        "--scenario",
        choices=[
            "smoke",
            "over",
            "under",
            "channel",
            "narrow",
            "no-solution",
            "cancel",
            "map-loss",
            "backend-exit",
            "map-change",
            "shutdown-diagnostic",
        ],
        default="smoke",
    )
    parser.add_argument("--managed-profile", action="store_true")
    parser.add_argument("--shutdown-stack-diagnostic", action="store_true")
    parser.add_argument("--shutdown-stack-delay", type=float, default=2.0)
    parser.add_argument("--shutdown-diagnostic-hold-seconds", type=float, default=5.0)
    parser.add_argument(
        "--baseline-strategy",
        choices=["combined", "ego"],
        help="Explicit legacy regression on the same empty lane",
    )
    parser.add_argument("--execute-simulation", action="store_true", required=True)
    args = parser.parse_args()
    if not 1 <= args.domain <= 232 or not 1024 <= args.gazebo_port <= 65535:
        parser.error("isolated local domain and port required")
    if args.baseline_strategy and args.scenario not in (["smoke", "narrow"] if args.baseline_strategy == "ego" else ["smoke"]):
        parser.error("baseline supports only the declared common empty/narrow physical routes")
    if args.scenario == "shutdown-diagnostic" and not args.shutdown_stack_diagnostic:
        parser.error("shutdown-only runs require explicit diagnostic stack capture")
    if (
        not 0.1 <= args.shutdown_stack_delay <= 3.0
        or not 1.0 <= args.shutdown_diagnostic_hold_seconds <= 120.0
    ):
        parser.error("bounded shutdown diagnostic timing required")
    args.strategy = args.baseline_strategy or "super"
    args.battery_drain_seconds = 1800.0
    args.output_dir.mkdir(parents=True, exist_ok=False)
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    probe = Probe(args)
    try:
        probe.run()
    except (Exception, KeyboardInterrupt) as error:
        probe.report["error"] = repr(error)
    finally:
        probe.close()
    print(
        json.dumps({k: probe.report[k] for k in ["passed", "error", "forced"]}, indent=2),
        flush=True,
    )
    return 0 if probe.report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
