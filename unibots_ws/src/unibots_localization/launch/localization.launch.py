#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Launch the full localization stack for the Unibots UK 2026 robot.

Brings up, all sharing ``use_sim_time``:

    1. apriltag_node (apriltag_ros)  -- detects 36h11 tags, publishes
       /apriltag/detections and the camera->tag TFs.
    2. ekf_bridge_node (this pkg)    -- converts tag-relative detections into an
       absolute robot pose on /localization/robot_pose.
    3. ekf_filter_node (robot_localization) -- fuses /odom, /imu/data and the
       AprilTag pose into a smooth map->odom->base_link estimate.

Pipeline::

    camera --image--> apriltag_node --detections/tf--> ekf_bridge_node
                                                          |
                                                          v
                   /odom, /imu/data ----------------> ekf_filter_node --> /tf
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PACKAGE_NAME = "unibots_localization"

# Topic names the camera driver is expected to provide / the detector consumes.
#
# In SIMULATION the Gazebo camera sensor emits an ideal, undistorted pinhole image
# (its CameraInfo carries a zero distortion model), so there is no separate
# /image_rectifier stage -- the raw image IS already rectified. We therefore feed
# apriltag straight from /camera/image_raw. On the REAL robot, insert the
# image_rectifier node and point this back at /camera/image_rect.
# ASSUMPTION: sim camera == rectified. Change to "/camera/image_rect" on hardware.
CAMERA_IMAGE_RECT_TOPIC = "/camera/image_raw"
CAMERA_INFO_TOPIC = "/camera/camera_info"
APRILTAG_DETECTIONS_TOPIC = "/apriltag/detections"


def generate_launch_description() -> LaunchDescription:
    """Build and return the launch description.

    Returns:
        The composed :class:`LaunchDescription`.
    """
    pkg_share = get_package_share_directory(PACKAGE_NAME)
    ekf_config = os.path.join(pkg_share, "config", "ekf.yaml")
    apriltag_config = os.path.join(pkg_share, "config", "apriltag.yaml")

    # ---- Launch arguments ---------------------------------------------------
    use_sim_time = LaunchConfiguration("use_sim_time")
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock. Set false on real robot.",
    )

    # ---- apriltag_node (apriltag_ros) --------------------------------------
    # Remap the detector's generic image/camera_info inputs to the rectified
    # camera stream, and its outputs to our pipeline topics.
    apriltag_node = Node(
        package="apriltag_ros",
        executable="apriltag_node",
        name="apriltag_node",
        output="screen",
        parameters=[apriltag_config, {"use_sim_time": use_sim_time}],
        remappings=[
            ("image_rect", CAMERA_IMAGE_RECT_TOPIC),
            ("camera_info", CAMERA_INFO_TOPIC),
            ("detections", APRILTAG_DETECTIONS_TOPIC),
        ],
    )

    # ---- ekf_bridge_node (this package) ------------------------------------
    ekf_bridge_node = Node(
        package=PACKAGE_NAME,
        executable="ekf_bridge_node",
        name="ekf_bridge_node",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "detections_topic": APRILTAG_DETECTIONS_TOPIC,
            "output_topic": "/localization/robot_pose",
        }],
    )

    # ---- ekf_filter_node (robot_localization) ------------------------------
    # robot_localization's ekf_node publishes its fused estimate on
    # 'odometry/filtered' by default. The rest of the stack (MPC, game FSM)
    # subscribes to /odom/filtered, so remap the output accordingly.
    ekf_filter_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[ekf_config, {"use_sim_time": use_sim_time}],
        remappings=[("odometry/filtered", "/odom/filtered")],
    )

    return LaunchDescription([
        declare_use_sim_time,
        apriltag_node,
        ekf_bridge_node,
        ekf_filter_node,
    ])
