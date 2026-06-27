from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='upper_motor_test',
            executable='upper_sine_test_node',
            name='upper_sine_test_node',
            output='screen',
            parameters=[
                {
                    'topic_name': '/joint_cmds',
                    'amplitude': 0.2,
                    'frequency': 0.1,
                    'q_center': 0.0,
                    'kp': 5.0,
                    'kd': 0.5,
                    'publish_rate_hz': 100.0,
                }
            ]
        )
    ])