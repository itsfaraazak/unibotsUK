"""Launch the Unibots behaviour-tree game controller.

Slots next to localization.launch.py / control.launch.py. Reads the tuned
parameter set from config/bt_game.yaml; home_zone may be overridden on the CLI:

    ros2 launch unibots_bt bt_game.launch.py home_zone:=north
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory("unibots_bt")
    config = os.path.join(pkg_share, "config", "bt_game.yaml")

    home_zone = LaunchConfiguration("home_zone")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription([
        DeclareLaunchArgument(
            "home_zone", default_value="north",
            description="Scoring wall: north | east | south | west"),
        DeclareLaunchArgument(
            "use_sim_time", default_value="false",
            description="Use simulation clock"),
        Node(
            package="unibots_bt",
            executable="bt_game_node",
            name="bt_game_node",
            output="screen",
            parameters=[
                config,
                {"home_zone": home_zone, "use_sim_time": use_sim_time},
            ],
        ),
    ])
