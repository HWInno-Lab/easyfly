"""
sensors.launch.py — 启动 D435i 深度相机 + 外参 TF.

启动项:
  1. realsense2_camera  (深度图 + 点云)
  2. static_transform_publisher  (相机 → base_link)
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # ── 相机参数 ──────────────────────────────
    depth_width_arg = DeclareLaunchArgument("depth_width", default_value="640")
    depth_height_arg = DeclareLaunchArgument("depth_height", default_value="480")
    depth_fps_arg = DeclareLaunchArgument("depth_fps", default_value="30")

    # ── D435i 外参 (相机在机体系下的位置) ──────
    # 默认: 相机朝前，无倾斜，位于机头前方 (0.15, 0, 0)
    # 用户应根据实际安装位置调整
    cam_x_arg = DeclareLaunchArgument(
        "cam_x", default_value="0.15", description="Camera X offset in base_link (forward, m)"
    )
    cam_y_arg = DeclareLaunchArgument(
        "cam_y", default_value="0.0", description="Camera Y offset in base_link (left, m)"
    )
    cam_z_arg = DeclareLaunchArgument(
        "cam_z", default_value="0.0", description="Camera Z offset in base_link (up, m)"
    )
    cam_roll_arg = DeclareLaunchArgument(
        "cam_roll", default_value="0.0", description="Camera roll (rad)"
    )
    cam_pitch_arg = DeclareLaunchArgument(
        "cam_pitch", default_value="0.0", description="Camera pitch (rad)"
    )
    cam_yaw_arg = DeclareLaunchArgument(
        "cam_yaw", default_value="0.0", description="Camera yaw (rad)"
    )

    # ── RealSense D435i ────────────────────────
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [get_package_share_directory("realsense2_camera"), "/launch/rs_launch.py"]
        ),
        launch_arguments={
            "depth_module.profile": [
                LaunchConfiguration("depth_width"),
                "x",
                LaunchConfiguration("depth_height"),
                "x",
                LaunchConfiguration("depth_fps"),
            ],
            "rgb_camera.profile": [
                LaunchConfiguration("depth_width"),
                "x",
                LaunchConfiguration("depth_height"),
                "x",
                LaunchConfiguration("depth_fps"),
            ],
            "enable_accel": "true",
            "enable_gyro": "true",
            "align_depth.enable": "true",
            "pointcloud.enable": "true",
            "initial_reset": "true",
        }.items(),
    )

    # ── 相机外参 TF: camera_link → base_link ────
    # 注意: D435i 出厂默认 ROS frame 是 "camera_link"
    # 此 TF 发布相机在机体上的位置 (camera_link 的父 = base_link)
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="camera_base_tf",
        arguments=[
            LaunchConfiguration("cam_x"),
            LaunchConfiguration("cam_y"),
            LaunchConfiguration("cam_z"),
            LaunchConfiguration("cam_yaw"),
            LaunchConfiguration("cam_pitch"),
            LaunchConfiguration("cam_roll"),
            "base_link",
            "camera_link",
        ],
    )

    return LaunchDescription(
        [
            depth_width_arg,
            depth_height_arg,
            depth_fps_arg,
            cam_x_arg,
            cam_y_arg,
            cam_z_arg,
            cam_roll_arg,
            cam_pitch_arg,
            cam_yaw_arg,
            realsense_launch,
            static_tf,
        ]
    )
