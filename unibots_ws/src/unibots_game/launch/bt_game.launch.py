"""Launch the behaviour-tree game controller (bt_game_node).

Launch arguments:
    home_zone      — which wall has our net: north (default), east, south, west
    use_sim_time   — true (default) for Gazebo sim, false for real hardware

Examples:
    ros2 launch unibots_game bt_game.launch.py
    ros2 launch unibots_game bt_game.launch.py home_zone:=east use_sim_time:=false
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    home_zone    = LaunchConfiguration("home_zone")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription([
        DeclareLaunchArgument(
            "home_zone",
            default_value="north",
            description="Home/net wall the robot deposits against (north/east/south/west).",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulated Gazebo clock (true) or system clock (false).",
        ),
        Node(
            package="unibots_game",
            executable="bt_game_node",
            name="bt_game_node",
            output="screen",
            parameters=[{
                "home_zone":    home_zone,
                "use_sim_time": use_sim_time,
            }],
        ),
    ])
