#!/usr/bin/env python3
"""Launch the C++ match orchestrator (high-level strategy state machine).

    ros2 launch unibots_orchestrator match_orchestrator.launch.py home:=south

Expects the rest of the stack running (camera -> perception -> spatial_memory,
the MPC controller, and the hardware motor/servo nodes).

Start the match (either works):
    ros2 topic pub --once /match/button std_msgs/msg/Bool '{data: true}'
    ros2 topic pub --qos-durability transient_local --once \
        /match/start std_msgs/msg/Bool '{data: true}'
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg = get_package_share_directory("unibots_orchestrator")
    default_cfg = os.path.join(pkg, "config", "match_orchestrator.yaml")

    home = LaunchConfiguration("home")
    use_sim_time = LaunchConfiguration("use_sim_time")
    config = LaunchConfiguration("config")

    return LaunchDescription([
        DeclareLaunchArgument("home", default_value="south",
                              description="home/start tile: south|east|north|west"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("config", default_value=default_cfg),
        Node(
            package="unibots_orchestrator",
            executable="match_orchestrator",
            name="match_orchestrator",
            output="screen",
            parameters=[config, {"home": home, "use_sim_time": use_sim_time}],
        ),
    ])
