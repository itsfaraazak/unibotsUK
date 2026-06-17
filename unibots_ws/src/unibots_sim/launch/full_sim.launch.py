# =============================================================================
# unibots_sim / launch/full_sim.launch.py
# =============================================================================
# MASTER launch file for the full Unibots UK 2026 simulation stack.
#
# Launch order (with a startup delay so Gazebo is up before the ROS stack):
#   1. sim.launch.py        -> Gazebo + world + robot spawn + bridge + RViz.
#   --- TimerAction (2 s) -------------------------------------------------
#   2. unibots_localization/localization.launch.py  -> EKF state estimation.
#   3. unibots_control/control.launch.py            -> motion controller (mpc/apf).
#   4. unibots_game game_state_node                 -> game/scoring logic.
#
# All ROS nodes run on the simulation clock (use_sim_time:=true).
#
# Usage:
#   ros2 launch unibots_sim full_sim.launch.py
#   ros2 launch unibots_sim full_sim.launch.py controller:=apf headless:=true \
#       home_zone:=south
# =============================================================================

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# Seconds to wait for Gazebo (and the in-sim controller_manager) to come up
# before starting the dependent ROS stack.
STARTUP_DELAY_S = 2.0


def generate_launch_description():
    pkg_sim = get_package_share_directory("unibots_sim")
    pkg_localization = get_package_share_directory("unibots_localization")
    pkg_control = get_package_share_directory("unibots_control")

    # ---- Launch arguments ----
    controller_arg = DeclareLaunchArgument(
        "controller",
        default_value="mpc",
        choices=["mpc", "apf"],
        description="Motion controller to launch from unibots_control "
                    "(mpc = model predictive, apf = artificial potential field).",
    )
    headless_arg = DeclareLaunchArgument(
        "headless",
        default_value="false",
        description="Run Gazebo headless (no GUI). Forwarded to sim.launch.py.",
    )
    home_zone_arg = DeclareLaunchArgument(
        "home_zone",
        default_value="north",
        description="Team home/scoring zone for the game_state_node.",
    )

    controller = LaunchConfiguration("controller")
    headless = LaunchConfiguration("headless")
    home_zone = LaunchConfiguration("home_zone")

    # Single source of truth for the simulation clock.
    # NOTE: launch_arguments values must be strings/substitutions ("true"), but a
    # node's `parameters` dict needs a real Python bool -- rclpy's internal
    # TimeSource declares `use_sim_time` as BOOL and rejects a STRING value.
    use_sim_time = "true"
    use_sim_time_param = True

    # ---- 1. Simulation (Gazebo + bridge + rsp + spawn + RViz) ----
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_sim, "launch", "sim.launch.py")
        ),
        launch_arguments={
            "headless": headless,
            "use_sim_time": use_sim_time,
        }.items(),
    )

    # ---- 2. Localization (EKF) ----
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_localization, "launch", "localization.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    # ---- 3. Control (selected motion controller) ----
    control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_control, "launch", "control.launch.py")
        ),
        launch_arguments={
            "controller": controller,
            "use_sim_time": use_sim_time,
        }.items(),
    )

    # ---- 4. Game state node ----
    game_state_node = Node(
        package="unibots_game",
        executable="game_state_node",
        name="game_state_node",
        output="screen",
        parameters=[
            {
                "home_zone": home_zone,
                "use_sim_time": use_sim_time_param,
            }
        ],
    )

    # ---- Delay the ROS stack so Gazebo / controller_manager start first ----
    delayed_stack = TimerAction(
        period=STARTUP_DELAY_S,
        actions=[
            localization,
            control,
            game_state_node,
        ],
    )

    return LaunchDescription(
        [
            controller_arg,
            headless_arg,
            home_zone_arg,
            sim,
            delayed_stack,
        ]
    )
