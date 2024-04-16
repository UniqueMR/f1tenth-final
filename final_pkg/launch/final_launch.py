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

    planner_node = Node(
        package='final_pkg',
        executable='planner_node',
        name='planner_node',
        parameters=[config_path],
        # parameters=['/home/runlong/sim_ws/install/pure_pursuit/share/pure_pursuit/config/pure_pursuit_params.yaml'],
        # prefix=['gdbserver localhost:3000'],
        # prefix='gdb -ex run --args',
        output='screen'
    )

    executer_node = Node(
        package='final_pkg',
        executable='executer_node',
        name='executer_node',
        parameters=[config_path],
        output='screen'
    )

    return LaunchDescription([
        planner_node, executer_node
    ])
