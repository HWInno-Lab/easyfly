#!/bin/bash
set -e
cd "$(dirname "$(realpath "$0")")"
echo "=== EasyFly 全量编译 ==="
colcon build --base-paths src --build-base build --install-base install --symlink-install --event-handlers console_direct+
echo "=== 编译完成 ==="
echo "请执行: source install/setup.bash"
