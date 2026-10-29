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

"""Test-only callback stall; never installed, never a flight or image node."""

import json
import math
import time

from easyfly_precision_landing.node import LandingTransform, run_observer, stamp
from std_msgs.msg import Float64, String


class QueueFixture(LandingTransform):
    """Run production observation logic plus one bounded, test-only blocker."""

    def __init__(self):
        super().__init__()
        self.marker = self.create_publisher(String, "/timing/block_state", 10)
        self.blocker = self.create_subscription(
            Float64, "/timing/block", self.block, 1
        )

    def block(self, message):
        """Stall only the executor while the actual DDS receiver remains alive."""
        if not math.isfinite(message.data) or not 0 < message.data <= 0.6:
            raise ValueError("test stall must be bounded to .6 seconds")
        self.marker.publish(String(data="started"))
        time.sleep(message.data)
        self.marker.publish(String(data="finished"))

    def target(self, message):
        """Trace each original source after its actual production callback."""
        super().target(message)
        print("timing_take " + json.dumps({
            "source_ns": stamp(message.header),
            "ros_age_s": (self.get_clock().now().nanoseconds - stamp(message.header)) * 1e-9,
            "rmw": self.diag_transport.get("/landing_target"),
        }), flush=True)


def main():
    """Own and clean only this observation process, with no flight interfaces."""
    run_observer(QueueFixture)


if __name__ == "__main__":
    main()
