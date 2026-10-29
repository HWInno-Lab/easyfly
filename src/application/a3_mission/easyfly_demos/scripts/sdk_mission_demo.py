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

"""Shared C++ SDK example. Default performs offline validation, never flight."""
import argparse
import math
from pathlib import Path
import subprocess
import time
from ament_index_python.packages import get_package_share_directory


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--mission",
        default=str(
            Path(get_package_share_directory("easyfly_mission_scripts"))
            / "missions/takeoff_hold_land_v1.yaml"
        ),
    )
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--sim-time", action="store_true")
    parser.add_argument("--cancel-after", type=float)
    args = parser.parse_args()
    if args.cancel_after is not None and (
        not math.isfinite(args.cancel_after) or not 0 < args.cancel_after <= 60
    ):
        parser.error("cancel-after must be in (0,60] seconds")
    check = subprocess.run(
        ["ros2", "run", "easyfly_mission_fsm", "mission_validate", args.mission], check=False
    )
    if check.returncode:
        return check.returncode
    if not args.execute:
        print("validation only, no motion")
        return 0
    from easyfly_pybind import Client, Status

    client = Client(use_sim_time=args.sim_time, require_managed_ownership=True)

    def execute():
        deadline = time.monotonic() + 5
        while not client.server_ready("execute_mission") and time.monotonic() < deadline:
            time.sleep(0.02)
        if not client.server_ready("execute_mission"):
            print("mission/ownership endpoint unavailable")
            return 1
        operation = client.execute_mission(args.mission)
        if not operation.wait(args.cancel_after if args.cancel_after is not None else 180):
            operation.cancel()
            if not operation.wait(10):
                print("cancel/drain unconfirmed; external execution may remain")
                return 1
        result = operation.snapshot()
        print(result.status, result.message)
        return (
            0
            if result.success or args.cancel_after is not None and result.status == Status.CANCELED
            else 1
        )

    try:
        exit_code = execute()
    finally:
        drained = client.close(2)
        if not drained:
            print("runtime drain unconfirmed")
    return exit_code if drained else 1


if __name__ == "__main__":
    raise SystemExit(main())
