#!/usr/bin/env python3
"""Launch file for the Unibots control stack.

Launches exactly ONE navigation controller -- either the iterative-MPC node or
the APF fallback node -- selected by the ``controller`` launch argument.

Launch arguments:
    controller    : 'mpc' (default) or 'apf'. Which controller node to start.
    use_sim_time  : 'true' (default) or 'false'. Passed to the chosen node.

Examples:
    ros2 launch unibots_control control.launch.py
    ros2 launch unibots_control control.launch.py controller:=apf
    ros2 launch unibots_control control.launch.py use_sim_time:=false
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Build and return the LaunchDescription for the control stack."""
    controller = LaunchConfiguration("controller")
    use_sim_time = LaunchConfiguration("use_sim_time")

    declare_controller = DeclareLaunchArgument(
        "controller",
        default_value="mpc",
        choices=["mpc", "apf"],
        description="Which controller to launch: 'mpc' (iterative MPC) or 'apf'.",
    )
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        choices=["true", "false"],
        description="Use simulation (Gazebo) clock if true.",
    )

    # Conditionally launch the MPC node when controller == 'mpc'.
    mpc_node = Node(
        package="unibots_control",
        executable="mpc_controller_node",
        name="mpc_controller_node",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(
            PythonExpression(["'", controller, "' == 'mpc'"])
        ),
    )

    # Conditionally launch the APF node when controller == 'apf'.
    apf_node = Node(
        package="unibots_control",
        executable="apf_controller_node",
        name="apf_controller_node",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(
            PythonExpression(["'", controller, "' == 'apf'"])
        ),
    )

    return LaunchDescription([
        declare_controller,
        declare_use_sim_time,
        mpc_node,
        apf_node,
    ])
