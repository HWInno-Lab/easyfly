#!/usr/bin/env python3
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

"""Static G6 detection observability and fallback contracts."""

from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
NODE = (PACKAGE_ROOT / "src" / "detection_node.cpp").read_text()
ALGORITHMS = PACKAGE_ROOT.parents[2] / "algorithms"
BACKEND = (ALGORITHMS / "detection/easyfly_yolo_onnx/src/onnx_backend.cpp").read_text()
BASE = (
    ALGORITHMS / "detection/easyfly_detection_api/include/easyfly_detection_api/detector.hpp"
).read_text()
LAUNCH = (PACKAGE_ROOT / "launch" / "detection.launch.py").read_text()
CONFIG = (PACKAGE_ROOT / "config" / "detection.yaml").read_text()


def require(text: str, needle: str, context: str) -> None:
    if needle not in text:
        raise AssertionError(f"{context}: missing {needle!r}")


def test_steady_clock_and_queue_counters() -> None:
    require(NODE, "std::chrono::steady_clock", "node timing clock")
    require(NODE, "++callback_counters_.received", "receive counter")
    require(NODE, "++callback_counters_.overwritten", "callback overwrite counter")
    require(NODE, "std::lock_guard<std::mutex> lk(pending_mtx_)", "counter lock")
    require(NODE, "snapshotInputCountersLocked()", "consistent counter snapshot")
    require(NODE, "++processed_frames_", "processed counter")
    require(NODE, "rclcpp::KeepLast", "bounded input QoS history")
    require(NODE, "input_qos.reliable()", "reliable input QoS")
    require(NODE, "input_qos.best_effort()", "best-effort input QoS fallback")
    require(NODE, "input_qos.durability_volatile()", "volatile input QoS")
    require(NODE, '"PERF steady window=', "periodic aggregate log")
    require(NODE, 'callback_overwritten=%" PRIu64', "callback overwrite metric in log")
    require(NODE, "callback_overwrite_pct", "callback-local overwrite percentage")
    require(NODE, "callback_received_wall_hz", "wall-clock receive rate")
    require(NODE, "processed_wall_hz", "wall-clock processed rate")
    require(NODE, "not DDS/publisher end-to-end loss", "drop scope in log")
    require(NODE, "queue_stats_", "receive-to-worker latency")


def test_counter_snapshot_window_baseline_contract() -> None:
    require(NODE, "struct CallbackCounters", "pending-protected counter structure")
    require(
        NODE,
        "InputCounterSnapshot snapshotInputCountersLocked() const",
        "lock-scoped counter snapshot helper",
    )
    require(NODE, "perf_last_callback_received_ = snapshot.received", "received baseline update")
    require(
        NODE, "perf_last_callback_overwritten_ = snapshot.overwritten", "overwrite baseline update"
    )
    require(NODE, "perf_window_start_ = now", "window boundary update")
    if "received_frames_" in NODE or "overwritten_frames_" in NODE:
        raise AssertionError("callback counters must not be sampled as independent atomics")
    if "perf_last_callback_received_before_log_" in NODE:
        raise AssertionError("log delta must use the baseline captured under pending_mtx_")
    if "perf_last_callback_overwritten_before_log_" in NODE:
        raise AssertionError("log delta must use the baseline captured under pending_mtx_")


def test_backend_stage_boundaries() -> None:
    require(BASE, "DetectionTiming * timing", "timing API")
    for field in (
        "timing->preprocess_ms",
        "timing->inference_ms",
        "timing->postprocess_ms",
    ):
        require(BACKEND, field, "ONNX stage timing")
    require(BACKEND, "const auto inference_start", "ORT timing start")
    require(BACKEND, "session_->Run", "ORT run boundary")
    require(BACKEND, "const auto postprocess_start", "postprocess timing start")


def test_base_preserves_caller_timing_fields() -> None:
    if "*timing = DetectionTiming{}" in BASE:
        raise AssertionError("DetectorBase must not reset caller-owned timing fields")
    require(BASE, "The caller owns the sample", "timing ownership contract")
    require(BASE, "detectImpl(bgr_img, detections, timing);", "timing forwarding")


def test_launch_fallback_and_observability_overrides() -> None:
    for argument in (
        '"use_gpu"',
        '"publish_debug_image"',
        '"performance_log_period_s"',
    ):
        require(LAUNCH, f"DeclareLaunchArgument(\n        {argument}", "launch argument")
    for argument in ('"input_qos_reliability"', '"input_qos_depth"'):
        require(LAUNCH, f"DeclareLaunchArgument(\n        {argument}", "QoS launch argument")
    require(LAUNCH, '"use_gpu_override"', "CPU fallback parameter")
    require(
        LAUNCH,
        'ParameterValue(\n                        LaunchConfiguration("use_gpu"), '
        "value_type=str)",
        "string-preserving CPU fallback override",
    )
    require(LAUNCH, '"publish_debug_image_override"', "debug image parameter")
    require(LAUNCH, '"performance_log_period_s_override"', "performance parameter")
    require(LAUNCH, '"input_qos_reliability_override"', "QoS reliability parameter")
    require(LAUNCH, '"input_qos_depth_override"', "QoS depth parameter")
    require(CONFIG, "performance_log_period_s:", "default performance window")


def main() -> None:
    tests = (
        test_steady_clock_and_queue_counters,
        test_counter_snapshot_window_baseline_contract,
        test_backend_stage_boundaries,
        test_base_preserves_caller_timing_fields,
        test_launch_fallback_and_observability_overrides,
    )
    for test in tests:
        test()
        print(f"PASS {test.__name__}")
    print(f"PASS {len(tests)}/{len(tests)} detection contracts")


if __name__ == "__main__":
    main()
