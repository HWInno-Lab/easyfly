"""M5 像素转 3D 节点启动"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    process_nice = DeclareLaunchArgument(
        'process_nice', default_value='0', choices=['0', '10'],
        description='CPU scheduling budget; simulation VIO uses 10 for perception work')
    container = ComposableNodeContainer(
        name="pixel_to_3d_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        prefix=PythonExpression(["'nice -n 10' if '", LaunchConfiguration('process_nice'),
                                 "' == '10' else ''"]),
        composable_node_descriptions=[
            ComposableNode(
                package="easyfly_pixel_to_3d",
                plugin="easyfly_pixel_to_3d::PixelTo3dNode",
                name="easyfly_pixel_to_3d",
                parameters=[{"use_sim_time": True}],
            ),
        ],
        output="screen",
    )
    return LaunchDescription([process_nice, container])
