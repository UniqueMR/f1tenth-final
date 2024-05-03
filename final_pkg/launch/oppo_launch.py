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

    oppnent_node = Node(
        package='final_pkg',
        executable='opponent_node',
        name='opponent_node',
        parameters=[config_path],
        output='screen'
    )

    return LaunchDescription([
        oppnent_node
    ])
