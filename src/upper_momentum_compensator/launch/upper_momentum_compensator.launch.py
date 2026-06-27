from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('upper_momentum_compensator'),
        'config',
        'upper_momentum_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='upper_momentum_compensator',
            executable='upper_momentum_compensator_node',
            name='upper_momentum_compensator_node',
            output='screen',
            parameters=[params]
        )
    ])
