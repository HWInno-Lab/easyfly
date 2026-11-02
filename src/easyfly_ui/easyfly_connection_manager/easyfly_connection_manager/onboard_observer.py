# Copyright 2026 EasyFly Project
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
"""Physical-clock observations and bounded access to the reviewed mission facade."""

import hashlib
import threading
import time

from easyfly_msgs.srv import GetMissionDetails, ManageMission, ManageReviewedMission
from rosidl_runtime_py.convert import message_to_ordereddict
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from .onboard_profiles import CATALOG, NODE_NAMES, observation_catalog
from .remote_observer import Observer


REQUIRED_NODES = {
    "navigation": ("easyfly_global_planner", "navigation_coordinator", "obstacle_supervisor"),
    "mission_fsm": ("mission_fsm", "search_area"),
    "local_planner": ("ego_planner_node", "traj_server"),
    "precision_landing": ("apriltag_detection", "landing_detection", "landing_transform"),
    "gcopter": ("gcopter_voxel_map", "gcopter_navigation"),
}
TASK_ACTIONS = {
    "takeoff": ("/takeoff", "flight_controller"),
    "waypoint": ("/goto_waypoint", "flight_controller"),
    "land": ("/land", "flight_controller"),
    "navigate": ("/navigate", "navigation_coordinator"),
    "search": ("/search_area", "search_area"),
    "track": ("/track_target", "easyfly_target_tracking"),
    "precision_land": ("/precision_land", "flight_controller"),
    "explore": ("/explore", "fuel_exploration"),
}
TASK_ACTION_TYPES = {
    "takeoff": "easyfly_msgs/action/Takeoff",
    "waypoint": "easyfly_msgs/action/GoToWaypoint",
    "land": "easyfly_msgs/action/Land",
    "navigate": "easyfly_msgs/action/NavigateTo",
    "search": "easyfly_msgs/action/SearchArea",
    "track": "easyfly_msgs/action/TrackTarget",
    "precision_land": "easyfly_msgs/action/PrecisionLand",
    "explore": "easyfly_msgs/action/Explore",
}


class PlatformObserver(Observer):
    """Reuse live FCU source epochs; startup additionally needs advancing output stamps."""

    catalog = CATALOG  # V1 default for legacy injected observers; real recipes override it.

    def __init__(self, recipe):
        self.output_stamps = {}
        self.output_ids = {}
        self.recipe = recipe
        self.catalog = observation_catalog(recipe)
        self.perception_gates = {}
        self.ever_connected = False
        profiles = {
            "platform": dict(output_topic="/uav_state", output_type="easyfly_msgs/msg/UAVState")
        }
        for name, entry in recipe["components"].items():
            if self.catalog[name]["message"]:
                profiles[name] = dict(
                    output_topic=entry["output_topic"], output_type=self.catalog[name]["message"],
                    camera_id=entry["parameters"].get("camera_id", "front")
                )
        super().__init__(dict(simulation_time=False, profiles=profiles))
        self.tf_buffer = Buffer(node=self.node)
        self.tf_listener = TransformListener(self.tf_buffer, self.node)
        self.mission_clients = {
            "manage": self.node.create_client(ManageMission, "/platform/mission/manage"),
            "details": self.node.create_client(GetMissionDetails, "/platform/mission/details"),
            "reviewed": self.node.create_client(
                ManageReviewedMission, "/platform/mission/reviewed"
            ),
        }

    def flight(self, key, message, generation):
        """Remember an actual FCU connection even if its later state becomes stale."""
        if key == "state" and message.connected:
            self.ever_connected = True
        super().flight(key, message, generation)

    def snapshot(self, name):
        """Startup cancellation before any FCU connection differs from flight admission."""
        result = super().snapshot(name)
        with self.lock:
            result["startup_abort_safe"] = bool(
                result["observer_alive"]
                and self.discovered
                and not self.ever_connected
                and not any(row[0] for row in self.actions.values())
                and all(
                    getattr(self, "counts", {}).get(topic, 0) <= 1
                    for topic in ("/mavros/state", "/mavros/extended_state", "/uav_state")
                )
            )
        return result

    def output(self, key, message):
        """Require actual source progress as well as receipt activity."""
        context = getattr(message, 'context', None)
        if context is not None:
            from easyfly_perception_contracts import SourceGate, context_valid
            gate = self.perception_gates.setdefault(key, SourceGate())
            if not context_valid(context) or not gate.observe(context):
                return
            if gate.changed or context.status != context.VALID:
                with self.lock: self.output_stamps.pop(key, None)
                if context.status != context.VALID: return
        header = context.header if context is not None else getattr(message, "header", None)
        stamp = header.stamp.sec * 10 ** 9 + header.stamp.nanosec if header else 0
        now = self.node.get_clock().now().nanoseconds
        with self.lock:
            if getattr(message, "valid", True) is not True:
                self.output_stamps.pop(key, None)
                self.outputs[key] = time.monotonic()
                return
            previous = self.output_stamps.get(key, (0, 0, 0.0))
            if stamp > previous[0] and -200000000 <= now - stamp <= 2500000000:
                self.output_stamps[key] = (stamp, previous[1] + 1, time.monotonic())
            elif stamp and stamp < previous[0]:
                self.output_stamps.pop(key, None)
            self.outputs[key] = time.monotonic()

    def ready(self, name, since):
        """Do not infer a sensor/map from a live process or an old DDS sample."""
        spec = self.catalog[name]
        if name == "mavlink" and not self.snapshot("platform")["grounded"]:
            return False
        if name == "fuel":
            if not self.ready("gcopter", 0.0) or not self.task_backend_ready("explore"):
                return False
            with self.lock:
                return self.fuel_provider_ok and time.monotonic() - self.fuel_idle[0] < 0.5
        if name == "map_odom":
            try:
                transform = self.tf_buffer.lookup_transform("map", "odom", Time())
                stamp = transform.header.stamp.sec * 10 ** 9 + transform.header.stamp.nanosec
                if (
                    not 0 < stamp
                    or not -200000000
                    <= self.node.get_clock().now().nanoseconds - stamp
                    <= 2500000000
                ):
                    return False
            except TransformException:
                return False
        if spec["message"]:
            if name in ("precision_landing", "gcopter", "fuel", "lidar_rays"):
                expected = self.required_nodes(name)
                nodes = self.node.get_node_names()
                if not all(nodes.count(node) == 1 for node in expected):
                    return False
                publishers = self.node.get_publishers_info_by_topic(
                    self.recipe["components"][name]["output_topic"])
                output_node = {"gcopter": "gcopter_voxel_map",
                               "precision_landing": "apriltag_detection",
                               "fuel": "fuel_exploration"}.get(name, NODE_NAMES[name])
                if (len(publishers) != 1 or publishers[0].node_namespace != "/" or
                        publishers[0].node_name != output_node):
                    return False
                identity = bytes(publishers[0].endpoint_gid)
                with self.lock:
                    if self.output_ids.get(name) != identity:
                        self.output_ids[name] = identity
                        self.output_stamps.pop(name, None)
                        return False
            info = self.snapshot(name)
            with self.lock:
                stamp, count, receipt = self.output_stamps.get(name, (0, 0, 0.0))
            return bool(
                info["observer_alive"]
                and info["output_publishers"] == 1
                and count >= 2
                and receipt >= since
                and time.monotonic() - receipt < 2.5
                and -200000000 <= self.node.get_clock().now().nanoseconds - stamp <= 2500000000
            )
        expected = self.required_nodes(name)
        nodes = self.node.get_node_names()
        return all(nodes.count(node) == 1 for node in expected)

    def required_nodes(self, name):
        """GCOPTER navigation owns one coordinator rather than the 2D A* stack."""
        recipe = getattr(self, "recipe", {})
        strategy = recipe.get("components", {}).get("navigation", {}).get(
            "parameters", {}).get("strategy", "astar")
        if name == "navigation" and strategy == "gcopter":
            return ("navigation_coordinator",)
        return REQUIRED_NODES.get(name, (NODE_NAMES.get(name, "onboard_" + name),))

    def source_conflict(self, name):
        """External publishers are never adopted as a managed sensor or algorithm."""
        if name == "mavlink" and getattr(self, "counts", {}).get("/mavros/state", 0):
            return True
        action = {
            "flight_control": "takeoff",
            "mission_fsm": "execute_mission",
            "navigation": "navigate",
            "target_tracking": "track_target",
            "fuel": "explore",
        }.get(name)
        if action and getattr(self, "counts", {}).get("/" + action + "/_action/status", 0):
            return True
        if name in self.profiles and self.snapshot(name)["output_publishers"]:
            return True
        expected = self.required_nodes(name)
        return any(node in self.node.get_node_names() for node in expected)

    def task_backend_ready(self, step_type):
        """Report unique expected action providers, never infer flight admission from discovery."""
        if time.monotonic() - self.last_discovery >= 0.5:
            return False
        if step_type == "hover":
            return self.ready("mission_fsm", 0.0)
        action, expected = TASK_ACTIONS[step_type]
        providers = []
        for name, namespace in self.node.get_node_names_and_namespaces():
            endpoints = self.node.get_action_server_names_and_types_by_node(name, namespace)
            if any(endpoint == action and TASK_ACTION_TYPES[step_type] in types
                   for endpoint, types in endpoints):
                providers.append((name, namespace))
        return providers == [(expected, "/")]

    def call(self, kind, values, timeout=1.5):
        """Send once to a unique service; a timed-out mutation is explicitly unresolved."""
        client = self.mission_clients[kind]
        providers = []
        for name, namespace in self.node.get_node_names_and_namespaces():
            services = self.node.get_service_names_and_types_by_node(name, namespace)
            if any(endpoint == client.srv_name for endpoint, _ in services):
                providers.append((name, namespace))
        if len(providers) != 1 or not client.service_is_ready():
            raise RuntimeError("需要唯一、可用的机载任务服务；不会回退到其他任务接口")
        request = client.srv_type.Request()
        for key, value in values.items():
            setattr(request, key, value)
        future = client.call_async(request)
        finished = threading.Event()
        future.add_done_callback(lambda _: finished.set())
        if not finished.wait(timeout):
            raise TimeoutError("机载任务请求结果未知，请查询原请求；不会自动重发")
        return dict(message_to_ordereddict(future.result()))

    def mission_details(self, name):
        """Bind the review to exact validated bytes returned by the onboard catalog."""
        result = self.call("details", dict(mission_name=name))
        if (
            not result["success"]
            or hashlib.sha256(result["content"].encode()).hexdigest() != result["content_sha256"]
        ):
            raise ValueError("任务读取/内容指纹检查失败：" + result.get("message", ""))
        return result

    def close(self):
        """Release TF subscriptions before closing the shared executor and context."""
        self.tf_listener.unregister()
        super().close()
