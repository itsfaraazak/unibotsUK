"""match_day.launch.py — single entry point for competition hardware.

Usage:
    ros2 launch unibots_game match_day.launch.py

Launch arguments:
    home_zone       north|east|south|west   (default: north)
    controller      apf|mpc                 (default: apf)
    device_id       V4L2 camera index       (default: 0)
    fps             camera frame rate       (default: 30)
    use_sim_time    true|false              (default: false for hardware)

After launch, start the match with:
    ros2 topic pub /match/start std_msgs/msg/Bool '{data: true}' --once

Tune live without rebuild:
    ros2 param set /perception_node conf_threshold 0.40
    ros2 param set /spatial_memory_node prediction_mode friction
    ros2 param set /bt_game_node use_predicted_position true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    # ── Launch arguments ─────────────────────────────────────────────────────
    home_zone    = LaunchConfiguration("home_zone")
    controller   = LaunchConfiguration("controller")
    device_id    = LaunchConfiguration("device_id")
    fps          = LaunchConfiguration("fps")
    use_sim_time = LaunchConfiguration("use_sim_time")
    hardware     = LaunchConfiguration("hardware")

    args = [
        DeclareLaunchArgument("home_zone",    default_value="north",
            description="Arena wall with our deposit net (north/east/south/west)."),
        DeclareLaunchArgument("controller",   default_value="apf",
            description="Navigation controller to use (apf or mpc)."),
        DeclareLaunchArgument("device_id",    default_value="0",
            description="V4L2 camera device index (/dev/videoN)."),
        DeclareLaunchArgument("fps",          default_value="30",
            description="Camera capture frame rate."),
        DeclareLaunchArgument("use_sim_time", default_value="false",
            description="Use Gazebo simulation clock (false for real hardware)."),
        DeclareLaunchArgument("hardware",     default_value="true",
            description="Launch physical motor/servo hardware nodes (false for sim/dev)."),
    ]

    # ── Config file paths ────────────────────────────────────────────────────
    perception_cfg    = os.path.join(
        get_package_share_directory("unibots_perception"), "config", "perception.yaml")
    spatial_mem_cfg   = os.path.join(
        get_package_share_directory("unibots_spatial_memory"), "config", "spatial_memory.yaml")
    bt_cfg            = os.path.join(
        get_package_share_directory("unibots_bt"), "config", "bt_game.yaml")
    localization_share = get_package_share_directory("unibots_localization")
    ekf_cfg           = os.path.join(localization_share, "config", "ekf.yaml")
    apriltag_cfg      = os.path.join(localization_share, "config", "apriltag.yaml")

    # ── 1. Camera ────────────────────────────────────────────────────────────
    camera_node = Node(
        package="unibots_camera",
        executable="camera_node",
        name="camera_node",
        output="screen",
        parameters=[{
            "device_id":    device_id,
            "fps":          fps,
            "use_sim_time": use_sim_time,
        }],
    )

    # ── 2. YOLO Perception ───────────────────────────────────────────────────
    perception_node = Node(
        package="unibots_perception",
        executable="perception_node",
        name="perception_node",
        output="screen",
        parameters=[perception_cfg, {"use_sim_time": use_sim_time}],
    )

    # ── 3. AprilTag detection ────────────────────────────────────────────────
    apriltag_node = Node(
        package="apriltag_ros",
        executable="apriltag_node",
        name="apriltag_node",
        output="screen",
        parameters=[apriltag_cfg, {"use_sim_time": use_sim_time}],
        remappings=[
            ("image_rect",  "/unibots/camera/image_raw"),
            ("camera_info", "/camera/camera_info"),
            ("detections",  "/apriltag/detections"),
        ],
    )

    # ── 4. EKF bridge (apriltag → robot pose) ────────────────────────────────
    ekf_bridge_node = Node(
        package="unibots_localization",
        executable="ekf_bridge_node",
        name="ekf_bridge_node",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # ── 5. EKF filter (odom + IMU + AprilTag fusion) ─────────────────────────
    ekf_filter_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[ekf_cfg, {"use_sim_time": use_sim_time}],
    )

    # ── 6. Spatial memory (Kalman ball tracker) ───────────────────────────────
    spatial_memory_node = Node(
        package="unibots_spatial_memory",
        executable="spatial_memory_node",
        name="spatial_memory_node",
        output="screen",
        parameters=[spatial_mem_cfg, {"use_sim_time": use_sim_time}],
    )

    # ── 7. Behaviour tree game controller ─────────────────────────────────────
    bt_game_node = Node(
        package="unibots_bt",
        executable="bt_game_node",
        name="bt_game_node",
        output="screen",
        parameters=[bt_cfg, {
            "home_zone":    home_zone,
            "use_sim_time": use_sim_time,
        }],
    )

    # ── 8a. APF controller (default) ─────────────────────────────────────────
    apf_node = Node(
        package="unibots_control",
        executable="apf_controller_node",
        name="apf_controller_node",
        output="screen",
        condition=IfCondition(PythonExpression(["'", controller, "' == 'apf'"])),
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # ── 8b. MPC controller (alternative) ─────────────────────────────────────
    mpc_node = Node(
        package="unibots_control",
        executable="mpc_controller_node",
        name="mpc_controller_node",
        output="screen",
        condition=IfCondition(PythonExpression(["'", controller, "' == 'mpc'"])),
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # ── 9. Hardware motor driver (RPi GPIO — skip in sim) ─────────────────────
    hardware_motor_node = Node(
        package="unibots_control",
        executable="hardware_motor_node",
        name="hardware_motor_node",
        output="screen",
        condition=IfCondition(hardware),
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # ── 10. Hardware servo driver (RPi I2C — skip in sim) ────────────────────
    hardware_servo_node = Node(
        package="unibots_control",
        executable="hardware_servo_node",
        name="hardware_servo_node",
        output="screen",
        condition=IfCondition(hardware),
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return LaunchDescription(args + [
        camera_node,
        perception_node,
        apriltag_node,
        ekf_bridge_node,
        ekf_filter_node,
        spatial_memory_node,
        bt_game_node,
        apf_node,
        mpc_node,
        hardware_motor_node,
        hardware_servo_node,
    ])
