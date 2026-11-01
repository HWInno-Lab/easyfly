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
"""One API for EasyFly running on the aircraft or on a ground computer."""

import uuid

from .remote_client import Client


class PlatformClient(Client):
    """Reuse verified TLS and identities; no direct remote shell or automatic mutations."""

    def __init__(self, config):
        super().__init__(config)
        self.timeout = 8.0

    def status(self):
        """Verify the expected onboard agent before exposing configuration controls."""
        result = self.request("GET", "/v2/platform/status")
        if result.get("v") != 2 or result.get("server_id") != self.config["server_id"]:
            raise ValueError("机载平台身份/协议不匹配")
        return result

    def operation(self, snapshot, operation, data, request_id=None):
        """Send one content-bound operation against the observed service incarnation."""
        return self.request(
            "POST",
            "/v2/platform/operation",
            dict(
                v=2,
                server_id=self.config["server_id"],
                boot_id=snapshot["boot_id"],
                id=request_id or uuid.uuid4().hex,
                client_id=self.config["client_id"],
                op=operation,
                data=data,
            ),
        )

    def logs(self, name):
        """Read diagnostics without consuming a mutation receipt or changing the platform."""
        return self.request("GET", "/v2/platform/log/" + name)
