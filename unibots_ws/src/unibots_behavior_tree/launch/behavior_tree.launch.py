import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # --- 1. Behavior Tree Configuration ---
    config = os.path.join(
        get_package_share_directory('unibots_behavior_tree'),
        'config',
        'bt_config.yaml'
    )

    bt_node = Node(
        package='unibots_behavior_tree',
        executable='main_tree_node',
        name='behavior_tree_node',
        output='screen',
        # Set use_sim_time to True so the tree stays in sync with Gazebo's clock
        parameters=[config, {'use_sim_time': True}] 
    )

    return LaunchDescription([
        bt_node
    ])