#!/bin/bash
# easyfly_clean.sh — 仿真环境一键清理
# 用途: 彻底清除所有 EasyFly 仿真相关进程 (含最易残留的 fastlio_mapping!)
#
# 什么时候用:
#   1. 启动新仿真前 (保险起见)
#   2. RViz 出现 path 乱画 / 点云飞机消失 (多实例残留的典型症状)
#   3. Ctrl+C 退出 launch 后想确认环境干净
#
# 背景 (2026-09-13 踩坑): fast_lio 的 fastlio_mapping 进程路径是
#   install/fast_lio/lib/fast_lio/fastlio_mapping (不含 "easyfly"),
#   常规 pkill 模式匹配不到 → 反复启停后累积僵尸实例 → 多个 SLAM
#   同时发布 /Odometry /path → RViz path 乱画、点云/飞机消失。
# 验证: pkill 后 `pgrep -cf fastlio_mapping` 应为 0 (无输出)。

# One matcher for inventory, termination and final verification. Match executable
# positions, not incidental text in a caller's shell command or an unrelated job.
names='fastlio_mapping|fastlivo_mapping|run_subscribe_msckf|openvins_guard|openvins_alignment|openvins_navigation|openvins_startup|fastlivo_guard|orbslam3_rgbd|super_worker|super_navigation|gcopter_worker|gcopter_navigation|gcopter_voxel_map|fuel_worker|fuel_exploration|gzserver|gzclient|easyfly_gzserver|px4|mavros_node|mavlink_bridge_node|rc_receiver_node|easyfly_flight_control|flight_control_node|easyfly_mission_fsm|mission_fsm_node|ego_planner_node|ego_to_mavros_node|traj_server|easyfly_pointcloud_filter|pointcloud_filter_node|easyfly_semantic_map|semantic_map_node|easyfly_global_planner|navigation_coordinator_node|global_planner_node|easyfly_target_tracking|target_tracking_node|component_container|component_container_mt|rviz2'
pattern="^([^ ]*/)?($names)( |$)|^([^ ]*/)?python3([.][0-9]+)? ([^ ]*/)?($names)( |$)|^/usr/bin/python3 /opt/ros/[^/]*/bin/ros2 launch "
matching_pids() {
  pgrep -u "$(id -u)" -f "$pattern" 2>/dev/null || true
}

echo "清理前:"
pgrep -a -u "$(id -u)" -f "$pattern" 2>/dev/null || true
# Give owned node shutdown handlers a chance to join their workers first.
for sig in INT TERM KILL; do
  mapfile -t pids < <(matching_pids)
  ((${#pids[@]})) || break
  kill -s "$sig" -- "${pids[@]}" 2>/dev/null || true
  for ((attempt=0; attempt<20; ++attempt)); do
    [[ -n "$(matching_pids)" ]] || break
    sleep 0.1
  done
done
mapfile -t remaining < <(matching_pids)
echo "清理完成, 残留进程: ${#remaining[@]}"
((${#remaining[@]} == 0))
