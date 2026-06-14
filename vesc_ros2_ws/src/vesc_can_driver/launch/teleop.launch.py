import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Запускает основную ноду VESC + teleop (WASD) одной командой.

    Графики у teleop по умолчанию выключены; включить:
        ros2 launch vesc_can_driver teleop.launch.py enable_plot:=true
    """
    pkg = get_package_share_directory("vesc_can_driver")
    default_params = os.path.join(pkg, "config", "vesc_params.yaml")

    params_arg = DeclareLaunchArgument(
        "params_file", default_value=default_params,
        description="YAML с параметрами основной ноды.")
    plot_arg = DeclareLaunchArgument(
        "enable_plot", default_value="false",
        description="Живые графики телеметрии в teleop (true/false).")

    vesc = Node(
        package="vesc_can_driver", executable="vesc_can_node", name="vesc_can_node",
        output="screen", parameters=[LaunchConfiguration("params_file")],
        emulate_tty=True,
    )
    teleop = Node(
        package="vesc_can_driver", executable="vesc_teleop_node", name="vesc_teleop",
        output="screen", emulate_tty=True,
        parameters=[{"enable_plot": LaunchConfiguration("enable_plot")}],
    )

    return LaunchDescription([params_arg, plot_arg, vesc, teleop])
