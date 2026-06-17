# =============================================================================
# unibots_description / display.launch.py
# =============================================================================
# Visualisation launch (no simulation): expands the robot xacro, starts
# robot_state_publisher, an optional joint_state_publisher_gui, and RViz2 with
# the bundled config.
#
# Usage:
#   ros2 launch unibots_description display.launch.py
#   ros2 launch unibots_description display.launch.py gui:=false
# =============================================================================

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("unibots_description")

    # ---- Launch arguments ----
    gui_arg = DeclareLaunchArgument(
        "gui",
        default_value="true",
        description="Launch joint_state_publisher_gui to jog the wheel joints.",
    )
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true.",
    )

    gui = LaunchConfiguration("gui")
    use_sim_time = LaunchConfiguration("use_sim_time")

    # ---- robot_description from xacro ----
    xacro_path = PathJoinSubstitution([pkg_share, "urdf", "robot.xacro"])
    robot_description = ParameterValue(
        Command(["xacro ", xacro_path]),
        value_type=str,
    )

    rviz_config = PathJoinSubstitution([pkg_share, "rviz", "robot.rviz"])

    # ---- Nodes ----
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": use_sim_time,
            }
        ],
    )

    # joint_state_publisher_gui supplies /joint_states for the continuous wheel
    # joints when not running ros2_control. Disable with gui:=false.
    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
        condition=IfCondition(gui),
        parameters=[{"use_sim_time": use_sim_time}],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return LaunchDescription(
        [
            gui_arg,
            use_sim_time_arg,
            robot_state_publisher_node,
            joint_state_publisher_gui_node,
            rviz_node,
        ]
    )
