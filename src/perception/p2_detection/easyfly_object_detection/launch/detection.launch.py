"""P2 目标检测节点启动 (可独立于 perception_sitl 调试)."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    process_nice = DeclareLaunchArgument(
        'process_nice', default_value='0', choices=['0', '10'],
        description='CPU scheduling budget; simulation VIO uses 10 for perception work')
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="true", description="使用 ROS 仿真时钟")
    config_file_arg = DeclareLaunchArgument(
        "config_file", default_value="",
        description="可选检测 YAML；为空时使用包内 detection.yaml")
    use_gpu_arg = DeclareLaunchArgument(
        "use_gpu", default_value="auto",
        description="执行提供者覆盖：auto 跟随 YAML，false 强制 CPU，true 请求 CUDA")
    debug_image_arg = DeclareLaunchArgument(
        "publish_debug_image", default_value="auto",
        description="调试图像覆盖：auto 跟随 YAML，false 关闭发布")
    perf_period_arg = DeclareLaunchArgument(
        "performance_log_period_s", default_value="auto",
        description="steady_clock 性能日志窗口（秒）；auto 跟随 YAML，<=0 关闭")
    qos_reliability_arg = DeclareLaunchArgument(
        "input_qos_reliability", default_value="auto",
        description="输入图像 QoS：auto 跟随 YAML，或 reliable/best_effort")
    qos_depth_arg = DeclareLaunchArgument(
        "input_qos_depth", default_value="auto",
        description="输入图像 QoS KeepLast 深度：auto 跟随 YAML，范围 1..100")

    container = ComposableNodeContainer(
        name="detection_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        prefix=PythonExpression(["'nice -n 10' if '", LaunchConfiguration('process_nice'),
                                 "' == '10' else ''"]),
        composable_node_descriptions=[
            ComposableNode(
                package="easyfly_object_detection",
                plugin="easyfly_object_detection::DetectionNode",
                name="easyfly_object_detection",
                parameters=[{
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "config_file": ParameterValue(
                        LaunchConfiguration("config_file"), value_type=str),
                    "use_gpu_override": ParameterValue(
                        LaunchConfiguration("use_gpu"), value_type=str),
                    "publish_debug_image_override": ParameterValue(
                        LaunchConfiguration("publish_debug_image"), value_type=str),
                    "performance_log_period_s_override": ParameterValue(
                        LaunchConfiguration("performance_log_period_s"), value_type=str),
                    "input_qos_reliability_override": ParameterValue(
                        LaunchConfiguration("input_qos_reliability"), value_type=str),
                    "input_qos_depth_override": ParameterValue(
                        LaunchConfiguration("input_qos_depth"), value_type=str),
                }],
            ),
        ],
        output="screen",
    )
    return LaunchDescription([
        process_nice,
        use_sim_time_arg,
        config_file_arg,
        use_gpu_arg,
        debug_image_arg,
        perf_period_arg,
        qos_reliability_arg,
        qos_depth_arg,
        container,
    ])
