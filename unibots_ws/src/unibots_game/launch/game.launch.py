"""Launch the top-level match FSM (``game_state_node``).

Exposes ``home_zone`` and ``use_sim_time`` as launch arguments so the same
launch file works in Gazebo simulation (sim time) and on hardware.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Build the launch description for the game state node."""
    home_zone = LaunchConfiguration("home_zone")
    use_sim_time = LaunchConfiguration("use_sim_time")

    declare_home_zone = DeclareLaunchArgument(
        "home_zone",
        default_value="north",
        description="Home/net wall the robot deposits against "
        "(north, east, south or west).",
    )
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulated (Gazebo) clock when true.",
    )

    game_state_node = Node(
        package="unibots_game",
        executable="game_state_node",
        name="game_state_node",
        output="screen",
        parameters=[
            {
                "home_zone": home_zone,
                "use_sim_time": use_sim_time,
            }
        ],
    )

    return LaunchDescription(
        [
            declare_home_zone,
            declare_use_sim_time,
            game_state_node,
        ]
    )
