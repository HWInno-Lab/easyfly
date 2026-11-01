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
"""Authenticated local/remote onboard platform management; no GUI is needed onboard."""

import argparse
import ipaddress
import json
import os
from pathlib import Path
import re
import signal
import socket
import subprocess
import threading
import time
import uuid

from .onboard_runtime import PlatformOwner
from .remote_profiles import private_json
from .remote_service import Handler, Server, atomic, private_directory


def validate_server(config):
    """Bind hardware ownership to one explicit domain and private configuration roots."""
    fields = {
        "server_id",
        "bind",
        "port",
        "certificate",
        "private_key",
        "token_file",
        "state_directory",
        "ros_domain_id",
        "local_only",
        "configuration_roots",
        "allow_mission_execution",
    }
    if (
        not isinstance(config, dict)
        or set(config) - {"advertised_host"} != fields
        or not re.fullmatch("[a-f0-9]{32}", config["server_id"])
        or type(config["port"]) is not int
        or not 1024 <= config["port"] <= 65535
        or type(config["ros_domain_id"]) is not int
        or not 0 <= config["ros_domain_id"] <= 232
        or type(config["local_only"]) is not bool
        or type(config["allow_mission_execution"]) is not bool
        or not isinstance(config["configuration_roots"], list)
        or not 1 <= len(config["configuration_roots"]) <= 8
    ):
        raise ValueError("invalid onboard service configuration")
    if not isinstance(config["bind"], str) or not re.fullmatch(
        "[A-Za-z0-9.-]{1,253}", config["bind"]
    ):
        raise ValueError("invalid service binding")
    for key in ("certificate", "private_key", "token_file", "state_directory"):
        path = Path(config[key])
        if not path.is_absolute() or path.resolve() != path:
            raise ValueError("onboard service paths must be canonical and absolute")
    for root in config["configuration_roots"]:
        path = Path(root)
        if (
            not path.is_absolute()
            or path.resolve() != path
            or not path.is_dir()
            or path.stat().st_uid != os.geteuid()
            or path.stat().st_mode & 0o022
        ):
            raise ValueError(
                "hardware configuration directories must be owned and non-writable by others"
            )
    return config


class PlatformHandler(Handler):
    """Keep the existing TLS/token boundary while offering whole-platform operations."""

    def do_GET(self):  # noqa: N802
        """Publish configuration and actual runtime state only to authenticated clients."""
        if not self.authenticated():
            return
        if self.path != "/v2/platform/status" and not self.path.startswith("/v2/platform/log/"):
            self.reply(404, dict(error="unknown platform endpoint"))
            return
        try:
            result = (
                self.server.owner.status()
                if self.path == "/v2/platform/status"
                else self.server.owner.logs(self.path.removeprefix("/v2/platform/log/"))
            )
            self.reply(200, result)
        except (OSError, RuntimeError, ValueError) as error:
            self.reply(503, dict(error=str(error)))

    def do_POST(self):  # noqa: N802
        """No arbitrary commands, process IDs, executable paths or mutation retries."""
        if not self.authenticated():
            return
        try:
            size = int(self.headers.get("Content-Length", "0"))
            if (
                self.path != "/v2/platform/operation"
                or not 0 < size <= 131072
                or self.headers.get("Transfer-Encoding")
                or self.headers.get("Content-Type") != "application/json"
            ):
                raise ValueError("invalid platform request endpoint, content type or size")
            data = self.rfile.read(size)
            if len(data) != size:
                raise ValueError("incomplete platform request")
            self.reply(200, self.server.owner.mutate(json.loads(data)))
        except (ValueError, KeyError, TypeError, OSError, RuntimeError) as error:
            self.reply(400, dict(error=str(error)))


def bootstrap_local(root, domain=0, allow_missions=False, host=""):
    """Provision local credentials automatically, never make the user start each algorithm."""
    root = Path(root).absolute()
    if host and not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9.-]{0,252}", host):
        raise ValueError("机载地址需要 IPv4 或 DNS 名称，不包含端口或路径")
    private_directory(root)
    server_path, client_path = root / "server.json", root / "client.json"
    if server_path.exists() or client_path.exists():
        if not server_path.exists() or not client_path.exists():
            raise ValueError("本机平台配置不完整，请检查原部署；未覆盖已有文件")
        validate_server(private_json(server_path))
        private_json(client_path)
        return server_path, client_path
    configuration = root / "configuration"
    private_directory(configuration)
    certificate, key, token = root / "server.pem", root / "server.key", root / "operator.token"
    if any(path.exists() for path in (certificate, key, token)):
        raise ValueError("已有未完成的凭据部署，请检查原目录；未覆盖")
    alternative_names = "DNS:localhost,IP:127.0.0.1"
    if host and host not in ("127.0.0.1", "localhost"):
        try:
            ipaddress.ip_address(host)
            alternative_names += ",IP:" + host
        except ValueError:
            alternative_names += ",DNS:" + host
    subprocess.run(
        [
            "openssl",
            "req",
            "-x509",
            "-newkey",
            "rsa:2048",
            "-nodes",
            "-keyout",
            str(key),
            "-out",
            str(certificate),
            "-days",
            "3650",
            "-subj",
            "/CN=localhost",
            "-addext",
            "subjectAltName=" + alternative_names,
        ],
        check=True,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        timeout=15,
    )
    key.chmod(0o600)
    certificate.chmod(0o600)
    import secrets

    with token.open("x") as stream:
        stream.write(secrets.token_urlsafe(48) + "\n")
    token.chmod(0o600)
    with socket.socket() as candidate:
        candidate.bind(("127.0.0.1", 0))
        port = candidate.getsockname()[1]
    identity = uuid.uuid4().hex
    config = dict(
        server_id=identity,
        bind="0.0.0.0" if host and host not in ("localhost", "127.0.0.1") else "127.0.0.1",
        advertised_host=host or "127.0.0.1",
        port=port,
        certificate=str(certificate),
        private_key=str(key),
        token_file=str(token),
        state_directory=str(root / "state"),
        ros_domain_id=domain,
        local_only=False,
        configuration_roots=[str(configuration)],
        allow_mission_execution=allow_missions,
    )
    atomic(server_path, validate_server(config))
    atomic(
        client_path,
        dict(
            url="https://127.0.0.1:" + str(port),
            ca_file=str(certificate),
            token_file=str(token),
            server_id=identity,
            client_id=uuid.uuid4().hex,
        ),
    )
    return server_path, client_path


def main():
    """Run the configured platform daemon; software autostart never starts a flight task."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    args = parser.parse_args()
    config = validate_server(private_json(args.config))
    os.environ["ROS_DOMAIN_ID"] = str(config["ros_domain_id"])
    os.environ["ROS_LOCALHOST_ONLY"] = "1" if config["local_only"] else "0"
    owner = PlatformOwner(config)
    server = None
    requested_exit = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: requested_exit.set())
    signal.signal(signal.SIGTERM, lambda *_: requested_exit.set())
    autostart = owner.recipe["autostart"] and owner.state["phase"] in ("stopped", "failed")
    autostart_deadline = time.monotonic() + 5
    try:
        server = Server(config, owner, PlatformHandler)
        print(
            json.dumps(
                dict(
                    server_id=config["server_id"],
                    port=server.server_port,
                    ros_domain_id=config["ros_domain_id"],
                )
            ),
            flush=True,
        )
        while True:
            server.handle_request()
            owner.tick()
            if autostart and owner.observer.snapshot("platform")["observer_alive"]:
                if (
                    owner.observer.snapshot("platform")["isolated"]
                    or owner.observer.snapshot("platform")["grounded"]
                ):
                    # Initial provisioning selects the same persistent operator;
                    # autostart and UI then share ownership rather than competing.
                    client_path = Path(args.config).with_name("client.json")
                    client_id = private_json(client_path)["client_id"]
                    request = dict(
                        v=2,
                        server_id=config["server_id"],
                        boot_id=owner.boot,
                        id=uuid.uuid4().hex,
                        client_id=client_id,
                        op="start",
                        data=dict(recipe_hash=owner.status()["recipe_hash"]),
                    )
                    print(json.dumps(owner.mutate(request), ensure_ascii=False), flush=True)
                    autostart = False
                elif time.monotonic() > autostart_deadline:
                    owner.state["error"] = "自动启动未获得明确平台状态；请在管理界面核对后手动启动"
                    owner.save()
                    autostart = False
            if (requested_exit.is_set() or owner.exit_requested) and owner.idle():
                break
    finally:
        if server:
            server.server_close()
        if owner.idle():
            owner.close()


if __name__ == "__main__":
    main()
