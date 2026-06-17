# =============================================================================
# unibots_sim / launch/sim.launch.py
# =============================================================================
# Brings up the Gazebo Harmonic simulation for the Unibots UK 2026 robot:
#
#   1. GZ_SIM_RESOURCE_PATH set so model:// includes (ping_pong_ball,
#      steel_bearing) and the unibots_description meshes resolve.
#   2. Gazebo Harmonic launched with the arena world (ros_gz_sim gz_sim.launch).
#   3. robot_state_publisher with the xacro-expanded URDF from
#      unibots_description, publishing /robot_description and TF.
#   4. The robot spawned into Gazebo (ros_gz_sim 'create') at (0.1, 0.1, 0.0).
#   5. controller_manager spawners: joint_state_broadcaster +
#      mecanum_drive_controller. (The controller_manager itself lives inside
#      Gazebo via the gz_ros2_control plugin declared in robot.xacro.)
#   6. ros_gz_bridge parameter_bridge for clock / camera / cmd_vel / odom / imu.
#   7. RViz2 with the unibots_description config.
#
# Usage:
#   ros2 launch unibots_sim sim.launch.py
#   ros2 launch unibots_sim sim.launch.py headless:=true
# =============================================================================

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


# ---- Robot start pose in the arena (SW corner area). ----
# NOTE on Z: base_link sits at the wheel-AXLE height (the wheel joints are at
# z=0 relative to base_link, so the wheels hang wheel_radius below it). Spawning
# at z=0 would bury the wheels under the floor and rest the chassis belly on the
# ground (no traction). Spawn at z = wheel_radius (0.03 m) so the wheel contact
# patches sit exactly on the floor and the chassis is clear of it.
ROBOT_START_X = "0.1"
ROBOT_START_Y = "0.1"
ROBOT_START_Z = "0.03"   # == wheel_radius in robot.xacro

# ---- Spawned model name in Gazebo. ----
ROBOT_MODEL_NAME = "unibots_robot"


def generate_launch_description():
    pkg_sim = get_package_share_directory("unibots_sim")
    pkg_description = get_package_share_directory("unibots_description")
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")

    desc_share = FindPackageShare("unibots_description")

    # ---- Launch arguments ----
    headless_arg = DeclareLaunchArgument(
        "headless",
        default_value="false",
        description="Run Gazebo without a GUI (server only, headless rendering).",
    )
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use the Gazebo simulation clock for all ROS nodes.",
    )

    headless = LaunchConfiguration("headless")
    use_sim_time = LaunchConfiguration("use_sim_time")

    # ---- Resource paths so model:// URIs resolve ----
    # GZ_SIM_RESOURCE_PATH must contain the directory that CONTAINS the model
    # folders, i.e. this package's 'models' dir, plus the description share for
    # any meshes referenced by the URDF.
    models_dir = os.path.join(pkg_sim, "models")
    set_resource_path = AppendEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=models_dir,
    )
    # Append the parent of the unibots_description share so package://-style
    # mesh lookups via model paths also resolve (harmless if unused).
    append_desc_path = AppendEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=os.path.dirname(pkg_description),
    )
    # Ensure SDF <plugin filename="gz-sim-...-system"> resolve (usually set by
    # the environment hooks, but make it explicit for headless/CI runs).
    set_sys_plugin_path = SetEnvironmentVariable(
        name="GZ_SIM_SYSTEM_PLUGIN_PATH",
        value=os.environ.get("GZ_SIM_SYSTEM_PLUGIN_PATH", ""),
    )

    # ---- World file ----
    world_path = os.path.join(pkg_sim, "worlds", "unibots_arena.sdf")

    # ---- Gazebo gz_args ----
    # '-r' starts simulation running immediately. In headless mode add '-s'
    # (server only) and '--headless-rendering' so camera sensors still render.
    gz_args = PythonExpression([
        "'-r -v 3 ",
        world_path.replace("\\", "/"),
        "'",
        " + (' -s --headless-rendering' if '",
        headless,
        "' == 'true' else '')",
    ])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": gz_args}.items(),
    )

    # ---- robot_description (xacro -> URDF string) ----
    xacro_path = PathJoinSubstitution([desc_share, "urdf", "robot.xacro"])
    robot_description = ParameterValue(
        Command(["xacro ", xacro_path]),
        value_type=str,
    )

    robot_state_publisher = Node(
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

    # ---- Spawn the robot from /robot_description ----
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_unibots_robot",
        output="screen",
        arguments=[
            "-topic", "/robot_description",
            "-name", ROBOT_MODEL_NAME,
            "-x", ROBOT_START_X,
            "-y", ROBOT_START_Y,
            "-z", ROBOT_START_Z,
            "-Y", "0.0",
        ],
    )

    # ---- controller_manager spawners ----
    # The controller_manager runs inside Gazebo (gz_ros2_control plugin). These
    # spawners simply load + activate the controllers once it is available.
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="joint_state_broadcaster_spawner",
        output="screen",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
        ],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # The mecanum_drive_controller subscribes to TwistStamped on its own
    # ~/reference topic (i.e. /mecanum_drive_controller/reference). Our motion
    # controllers (mpc/apf) publish to /cmd_vel, so we remap the controller's
    # reference onto /cmd_vel via --controller-ros-args. This keeps /cmd_vel as
    # the single robot velocity interface (same on real hardware).
    mecanum_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="mecanum_drive_controller_spawner",
        output="screen",
        arguments=[
            "mecanum_drive_controller",
            "--controller-manager", "/controller_manager",
            # Remap the controller's topics:
            #   reference    -> /cmd_vel  (TwistStamped velocity command in)
            #   odometry     -> /odom     (nav_msgs/Odometry, fused by the EKF)
            #   tf_odometry  -> /tf       (the odom->base_link transform; the mecanum
            #                              controller publishes it on ~/tf_odometry,
            #                              NOT /tf, so without this remap the EKF
            #                              cannot find odom->base_link and never
            #                              produces /odom/filtered)
            "--controller-ros-args",
            "-r /mecanum_drive_controller/reference:=/cmd_vel "
            "-r /mecanum_drive_controller/odometry:=/odom "
            "-r /mecanum_drive_controller/tf_odometry:=/tf",
        ],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # ---- ros_gz_bridge ----
    # Direction syntax:
    #   @  bidirectional
    #   [  gz  -> ros
    #   ]  ros -> gz
    # Format: <topic>@<ros_msg>[<gz_msg>  (gz->ros) etc.
    #
    # Gazebo publishes the camera on '/camera' (+ '/camera_info') and the IMU
    # on '/imu' (sensor <topic> values in robot.xacro). We bridge those gz
    # topics and REMAP them to the ROS-side names the stack expects
    # (/camera/image_raw, /camera/camera_info, /imu/data).
    #
    # NOTE: /cmd_vel is NOT bridged. In simulation the mecanum_drive_controller
    # runs inside gz_ros2_control and commands the wheel hardware interfaces
    # directly from the ROS-side /cmd_vel (TwistStamped) -- there is no gz-side
    # consumer. Bridging /cmd_vel as gz.msgs.Twist would also collide with the
    # controller's TwistStamped on the same topic name.
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=[
            # /clock  gz -> ros
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            # camera image  gz -> ros
            "/camera@sensor_msgs/msg/Image[gz.msgs.Image",
            # camera info   gz -> ros
            "/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            # NOTE: /odom is NOT bridged from gz -- the world has no gz
            # OdometryPublisher. Wheel odometry comes from mecanum_drive_controller
            # (remapped to /odom above).
            # imu  gz -> ros
            "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
        ],
        remappings=[
            ("/camera", "/camera/image_raw"),
            ("/camera_info", "/camera/camera_info"),
            ("/imu", "/imu/data"),
        ],
    )

    # ---- RViz2 ----
    rviz_config = PathJoinSubstitution([desc_share, "rviz", "robot.rviz"])
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
        # Skip RViz GUI when running headless.
        condition=IfCondition(
            PythonExpression(["'", headless, "' != 'true'"])
        ),
    )

    return LaunchDescription(
        [
            headless_arg,
            use_sim_time_arg,
            set_resource_path,
            append_desc_path,
            set_sys_plugin_path,
            gazebo,
            robot_state_publisher,
            spawn_robot,
            joint_state_broadcaster_spawner,
            mecanum_drive_controller_spawner,
            bridge,
            rviz,
        ]
    )
