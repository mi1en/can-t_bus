import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("vesc_can_driver")
    default_params = os.path.join(pkg, "config", "vesc_params.yaml")

    params_arg = DeclareLaunchArgument(
        "params_file", default_value=default_params,
        description="Путь к YAML с параметрами ноды.")

    node = Node(
        package="vesc_can_driver",
        executable="vesc_can_node",
        name="vesc_can_node",
        output="screen",
        parameters=[LaunchConfiguration("params_file")],
        emulate_tty=True,
    )

    return LaunchDescription([params_arg, node])
