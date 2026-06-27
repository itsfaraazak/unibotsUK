"""Launch the ultrasonic proximity / collision-avoidance node.

  ros2 launch unibots_ultrasonic ultrasonic.launch.py
  ros2 launch unibots_ultrasonic ultrasonic.launch.py use_mock:=true   # off-Pi dev
  ros2 launch unibots_ultrasonic ultrasonic.launch.py config:=/abs/path/to.yaml

Designed to be included from the top-level match launch alongside perception and
control; the fused /ultrasonic/obstacles feeds the same APF/MPC repulsion the
camera obstacles use.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("unibots_ultrasonic")
    default_config = os.path.join(pkg_share, "config", "ultrasonic.yaml")

    config_arg = DeclareLaunchArgument(
        "config", default_value=default_config,
        description="Path to the ultrasonic sensor YAML config.")
    use_mock_arg = DeclareLaunchArgument(
        "use_mock", default_value="false",
        description="true -> mock backend (no GPIO) for off-Pi dev / CI.")

    use_mock = LaunchConfiguration("use_mock")

    node = Node(
        package="unibots_ultrasonic",
        executable="ultrasonic_node",
        name="ultrasonic_node",
        output="screen",
        parameters=[
            LaunchConfiguration("config"),
            # CLI override wins over the YAML value for quick dev toggling.
            {"use_mock": PythonExpression(["'", use_mock, "'.lower() == 'true'"])},
        ],
    )

    return LaunchDescription([config_arg, use_mock_arg, node])
