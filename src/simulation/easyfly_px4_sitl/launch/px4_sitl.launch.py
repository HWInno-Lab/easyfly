"""
px4_sitl.launch.py — 启动 Gazebo Classic + PX4 SITL

用法:
  ros2 launch easyfly_px4_sitl px4_sitl.launch.py
  ros2 launch easyfly_px4_sitl px4_sitl.launch.py world:=perception_demo.world   # 感知场景
  PX4_DIR=/path/to/PX4-Autopilot ros2 launch easyfly_px4_sitl px4_sitl.launch.py  # 自定义 PX4 目录

架构:
  gzserver (world)  →  gz model spawn (easyfly_iris: D435i 深度相机 + MID-360)
  gzclient (GUI)
  px4 (simulator_mavlink ↔ Gazebo mavlink_interface plugin via TCP 4560)

环境要求:
  - 先显式运行 prepare_px4_hrt，生成项目局部仿真兼容版；缺少或指纹不匹配拒绝启动
  - PX4_DIR 可指定已校验兼容输入或私有 staging，不允许静默回退未修补版本
  - Gazebo Classic 11 已安装 (gzserver / gzclient)
  - easyfly_gazebo_plugins 已编译 (MID-360 仿真插件 liblivox_laser_simulation.so)
"""

import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    SetEnvironmentVariable,
)
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch.conditions import IfCondition
from easyfly_px4_sitl.session_guard import guarded_launch
from easyfly_px4_sitl.hrt_compatibility import select_px4_directory
from easyfly_px4_sitl.scoped_timer import ScopedTimerAction as TimerAction
from easyfly_px4_sitl.telemetry import heartbeat_configuration_action
from easyfly_px4_sitl.scene_inputs import preflight_action, scene_value, selection_arguments


def _resolve_px4_dir():
    """Use only the reviewed local compatibility build or its verified private staging."""
    return select_px4_directory(os.environ.get("PX4_DIR", "").strip() or None)


def generate_launch_description():
    # ---- 参数 ----
    scene_args = selection_arguments('base_sim')
    gzclient_arg = DeclareLaunchArgument(
        "gzclient", default_value="true", description="Start the Gazebo GUI")

    # ---- 路径 ----
    pkg_px4_sitl      = get_package_share_directory("easyfly_px4_sitl")
    pkg_gazebo_models = get_package_share_directory("easyfly_gazebo_models")
    gazebo_plugins_lib = get_package_prefix("easyfly_gazebo_plugins") + "/lib"
    gazebo_server = gazebo_plugins_lib + "/easyfly_gazebo_plugins/easyfly_gzserver"

    px4_dir = _resolve_px4_dir()
    px4_build = str(px4_dir / "build" / "px4_sitl_default")
    px4_bin   = str(px4_dir / "build" / "px4_sitl_default" / "bin" / "px4")

    model_dir   = pkg_px4_sitl + "/models"
    # 预检查先解析资源；进程只读取随后设置的受管输入。
    world_file = scene_value('world')
    model_name  = "easyfly_iris"
    model_sdf_arg = DeclareLaunchArgument(
        "model_sdf", default_value=model_dir + "/" + model_name + "/" + model_name + ".sdf",
        description="Explicit simulation fixture SDF; default production Iris unchanged")
    model_sdf = scene_value('model_sdf')

    # ---- GAZEBO_MODEL_PATH ----
    px4_models_dir = str(
        px4_dir / "Tools" / "simulation" / "gazebo-classic" /
        "sitl_gazebo-classic" / "models")
    default_roots = [
        model_dir,
        pkg_gazebo_models,
        px4_models_dir,
    ]
    gazebo_model_path = scene_value('model_path')

    # ---- 插件路径: PX4 插件 + easyfly 传感器插件(livox) + 系统 gazebo_ros 插件 ----
    gazebo_plugin_dir = str(px4_dir / "build" / "px4_sitl_default" / "build_gazebo-classic")

    # ---- 1. Gazebo 环境变量 ----
    set_model_path = SetEnvironmentVariable(
        "GAZEBO_MODEL_PATH", gazebo_model_path)
    # /opt/ros/humble/lib 必须包含: libgazebo_ros_init.so / libgazebo_ros_camera.so 等系统插件
    set_plugin_path = SetEnvironmentVariable(
        "GAZEBO_PLUGIN_PATH",
        gazebo_plugin_dir + ":" + gazebo_plugins_lib + ":/opt/ros/humble/lib:" + os.environ.get("GAZEBO_PLUGIN_PATH", ""))
    # PX4 gazebo 插件依赖 libmav_msgs.so / libsensor_msgs.so 等, 位于 PX4 build_gazebo-classic
    set_ld_library_path = SetEnvironmentVariable(
        "LD_LIBRARY_PATH",
        [gazebo_plugin_dir + ":", EnvironmentVariable("LD_LIBRARY_PATH", default_value="")])

    # ---- 2. gzserver ----
    # -s libgazebo_ros_init.so: 加载 ROS2 时钟插件(SystemPlugin, 发布 /clock), 必须用 -s 方式加载
    gzserver = ExecuteProcess(
        cmd=[gazebo_server, "--verbose", "-s", "libgazebo_ros_init.so", world_file],
        output="screen",
        name="gzserver",
    )

    # ---- 3. 生成 SDF 并生成模型（使用 spawn_model 风格） ----
    # 注意：这里直接用已编译的 SDF，不需要 jinja 模板生成
    spawn_model = ExecuteProcess(
        cmd=[
            "gz", "model",
            ["--spawn-file=", model_sdf],
            "--model-name=" + model_name,
            "-x", scene_value('x'),
            "-y", scene_value('y'),
            "-z", scene_value('z'),
            "-Y", scene_value('yaw'),
        ],
        output="screen",
        name="spawn_model",
    )

    # ---- 4. gzclient（GUI） ----
    gzclient = ExecuteProcess(
        cmd=["gzclient"],
        output="screen",
        name="gzclient",
        condition=IfCondition(LaunchConfiguration("gzclient")),
    )

    # ---- 5. PX4 SITL ----
    # 使用 SYS_AUTOSTART=10015（gazebo-classic iris），
    # simulator_mavlink 通过 TCP 4560 与 Gazebo mavlink 插件通信
    sitl_env = {
        "PX4_SIM_MODEL": "gazebo-classic_iris",   # PX4 standard airframe (10015_gazebo-classic_iris)
        # 注意: 不要设置 PX4_HOME_LAT/LON — 自定义经纬度(如武汉)与 PX4 SITL 默认
        # home(苏黎世)的磁偏角差异会让 EKF local frame 相对 Gazebo 世界旋转 ~90°,
        # 导致 mavros 位置与 Gazebo/SLAM 坐标系不一致 (M6 踩坑: 航点(x,0)被飞成(0,x))
    }
    # 继承当前环境
    sitl_env.update(os.environ)

    px4_sitl = ExecuteProcess(
        cmd=[px4_bin, "-d"],
        cwd=px4_build,
        env=sitl_env,
        output="screen",
        name="px4_sitl",
    )

    # ---- 启动顺序 ----
    # gzserver 先启动 → 3s 后 spawn 模型 → 再 3s 后 gzclient + px4
    spawn_delayed = TimerAction(
        period=3.0,
        actions=[spawn_model],
    )

    gui_and_px4 = TimerAction(
        period=6.0,
        actions=[gzclient, px4_sitl],
    )

    actions = [
        *scene_args,
        model_sdf_arg,
        gzclient_arg,
        set_model_path,
        set_plugin_path,
        set_ld_library_path,
        heartbeat_configuration_action(px4_sitl, px4_dir),
        gzserver,
        spawn_delayed,
        gui_and_px4,
    ]
    return guarded_launch(actions, {
        gzserver: "Gazebo", px4_sitl: "PX4", spawn_model: "model spawn",
    }, preflight=[preflight_action(
        px4_dir=px4_dir, default_world='empty.world', default_roots=default_roots,
        default_model=model_dir + '/' + model_name + '/' + model_name + '.sdf',
        allowed_profiles=('base_sim',))])
