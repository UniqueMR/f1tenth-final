from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory('final_pkg'),
        'config',
        'final_params.yaml'
    )
    print(config_path)

    sim_map_node = Node(
        package='final_pkg',
        executable='sim_map_node.py',
        name='sim_map_node',
        parameters=[config_path],
        output='screen'
    )

    get_obs_node = Node(
        package='final_pkg',
        executable='get_obs_node.py',
        name='get_obs_node',
        parameters=[config_path],
        output='screen'
    )

    return LaunchDescription([
        sim_map_node, get_obs_node
    ])
