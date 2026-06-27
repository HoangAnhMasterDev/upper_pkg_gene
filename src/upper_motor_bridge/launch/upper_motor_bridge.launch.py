from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='upper_motor_bridge',
            executable='upper_motor_bridge_node',
            name='upper_motor_bridge_node',
            output='screen',
            parameters=[{
                'publish_rate_hz': 200.0,
                'state_shm_name': '/upper_motor_state',
                'command_shm_name': '/upper_motor_command',
            }]
        )
    ])
