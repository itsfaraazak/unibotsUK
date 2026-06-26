#!/usr/bin/env python3
"""Top-level MATCH-DAY launch for the Unibots UK 2026 robot.

Single entry point that brings up the whole autonomy stack in pipeline order::

    camera_node ──/unibots/camera/image_raw──▶ perception_node
                                                   │ /vision/balls
                                                   ▼
    /odom/filtered ──▶ spatial_memory_node ──/spatial_memory/ball_map──▶ bt_game_node
                                                   │ /game/target            │ /servo/command
                                                   ▼                         ▼
                          control (mpc|apf) ──/cmd_vel──▶ hardware_motor_node
                                                              hardware_servo_node

Localization (AprilTag + EKF → /odom/filtered) is included by default; disable it
with localization:=false when an external pose source is used.

Usage::

    ros2 launch unibots_bt match.launch.py home_zone:=north
    ros2 launch unibots_bt match.launch.py controller:=apf hardware:=false
    ros2 launch unibots_bt match.launch.py use_sim_time:=true localization:=false

Start the match (rulebook §1.9 — robot is idle until this fires)::

    ros2 topic pub --qos-durability transient_local /match/start \\
        std_msgs/msg/Bool '{data: true}' --once
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    pkg_bt = get_package_share_directory("unibots_bt")
    pkg_perception = get_package_share_directory("unibots_perception")
    pkg_spatial = get_package_share_directory("unibots_spatial_memory")
    pkg_control = get_package_share_directory("unibots_control")
    pkg_localization = get_package_share_directory("unibots_localization")

    perception_cfg = os.path.join(pkg_perception, "config", "perception.yaml")
    spatial_cfg = os.path.join(pkg_spatial, "config", "spatial_memory.yaml")

    # ── Launch arguments ──────────────────────────────────────────────────────
    home_zone = LaunchConfiguration("home_zone")
    controller = LaunchConfiguration("controller")
    use_sim_time = LaunchConfiguration("use_sim_time")
    hardware = LaunchConfiguration("hardware")
    camera = LaunchConfiguration("camera")
    localization = LaunchConfiguration("localization")

    args = [
        DeclareLaunchArgument(
            "home_zone", default_value="north",
            description="Scoring wall: north | east | south | west"),
        DeclareLaunchArgument(
            "controller", default_value="mpc", choices=["mpc", "apf"],
            description="Motion controller: mpc (default) or apf fallback"),
        DeclareLaunchArgument(
            "use_sim_time", default_value="false", choices=["true", "false"],
            description="Use simulation clock (true for Gazebo)"),
        DeclareLaunchArgument(
            "hardware", default_value="true", choices=["true", "false"],
            description="Launch hardware motor + servo bridges (false for dev/sim)"),
        DeclareLaunchArgument(
            "camera", default_value="true", choices=["true", "false"],
            description="Launch the V4L2 camera_node (false if frames come from sim)"),
        DeclareLaunchArgument(
            "localization", default_value="true", choices=["true", "false"],
            description="Launch AprilTag + EKF localization (provides /odom/filtered)"),
    ]

    # rclpy declares use_sim_time as BOOL and rejects a STRING; wrap it.
    sim_time_param = {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)}

    # ── Perception pipeline ───────────────────────────────────────────────────
    camera_node = Node(
        package="unibots_camera",
        executable="camera_node",
        name="camera_node",
        output="screen",
        parameters=[sim_time_param],
        condition=IfCondition(camera),
    )

    perception_node = Node(
        package="unibots_perception",
        executable="perception_node",
        name="perception_node",
        output="screen",
        parameters=[perception_cfg, sim_time_param],
    )

    spatial_memory_node = Node(
        package="unibots_spatial_memory",
        executable="spatial_memory_node",
        name="spatial_memory_node",
        output="screen",
        parameters=[spatial_cfg, sim_time_param],
    )

    # ── Game controller (BT.CPP) ──────────────────────────────────────────────
    bt_game = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bt, "launch", "bt_game.launch.py")),
        launch_arguments={
            "home_zone": home_zone,
            "use_sim_time": use_sim_time,
        }.items(),
    )

    # ── Motion controller ─────────────────────────────────────────────────────
    control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_control, "launch", "control.launch.py")),
        launch_arguments={
            "controller": controller,
            "use_sim_time": use_sim_time,
        }.items(),
    )

    # ── Localization (optional) ───────────────────────────────────────────────
    localization_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_localization, "launch", "localization.launch.py")),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
        condition=IfCondition(localization),
    )

    # ── Hardware bridges (optional) ───────────────────────────────────────────
    hardware_motor = Node(
        package="unibots_control",
        executable="hardware_motor_node",
        name="hardware_motor_node",
        output="screen",
        parameters=[sim_time_param],
        condition=IfCondition(hardware),
    )

    hardware_servo = Node(
        package="unibots_control",
        executable="hardware_servo_node",
        name="hardware_servo_node",
        output="screen",
        parameters=[sim_time_param],
        condition=IfCondition(hardware),
    )

    return LaunchDescription(args + [
        camera_node,
        perception_node,
        spatial_memory_node,
        bt_game,
        control,
        localization_stack,
        hardware_motor,
        hardware_servo,
    ])
